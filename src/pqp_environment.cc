#include "pqp_environment.h"

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

PqpEnvironment::PqpEnvironment(const std::vector<std::string>&
                                   robot_model_files,
                               const std::string& dh_table_file,
                               const std::string& obstacles_model_file,
                               RandomSpaceGeneratorInterface *random_generator,
                               const int sample_space_size) :
                               obstacles_(new PQP_Model),
                               conf_sample_space_(nullptr),
                               sample_space_size_(sample_space_size) {
  if(!LoadRobotModel(robot_model_files)) throw "Robot model problem!";
  if(!LoadDhTable(dh_table_file)) throw "DH table problem!";
  if(!LoadObstacles(obstacles_model_file)) throw "Obstacles problem!";
  if(!GenerateSampleSpace(random_generator, sample_space_size))
    throw "Sample space not generated!";
  dimension_ = segments_.size();
}

//TODO: Link number of segments
bool PqpEnvironment::LoadRobotModel(
    const std::vector<std::string>& robot_model_files) {
  try {
    for (const auto& model_file : robot_model_files)
      segments_.emplace_back(ParseModel(model_file));

    return true;
  }
  catch (...) {
    throw;
  }
  return false; //Input file not present
}

bool PqpEnvironment::LoadDhTable(const std::string& dh_table_file) {
  std::ifstream input_file (dh_table_file.c_str());
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            //End of file
      std::streampos length (input_file.tellg());    //Read the size
      input_file.seekg(0, std::ios::beg);            //Return to beginning

      std::vector<char> buffer (length);
      input_file.read(&buffer[0], length);

      //Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);

      double theta, d, a, alpha;
      while (!parser.eof())
      {
        parser >> theta >> d >> a >> alpha;
        dh_table_.emplace_back(theta, d, a, alpha);
      }
      return true;
    }
    catch(...) {
      throw "File error!";
    }
  }
  return false; //Input file not present
}

//Hopefully no modifications needed for general case
bool PqpEnvironment::LoadObstacles(const std::string& obstacles_model_file) {
  try {
    obstacles_ = std::unique_ptr<PQP_Model>(ParseModel(obstacles_model_file));
    return true;
  }
  catch (...) {
    throw;
  }
  return false; //Input file not present
}

bool PqpEnvironment::GenerateSampleSpace(
    RandomSpaceGeneratorInterface* random_generator,
    const int sample_space_size) {
  try {
    // Initialize points_array from sample space generator
    std::unique_ptr<double[]> points_array (
      std::move(
        (random_generator->CreateSampleSpace(sample_space_size))));
    // Create configuration sample space
    conf_sample_space_ = new FlannPointArray (
      flann::Matrix<double> (points_array.release(), sample_space_size,
      static_cast<int>(segments_.size())), flann::KDTreeIndexParams(4));
    conf_sample_space_->buildIndex();

    return true;
  } catch (...) {
    return false;
  }
}

PQP_Model* PqpEnvironment::ParseModel(const std::string& model_file) {
  std::ifstream input_file (model_file.c_str(), std::ios::binary);
  try {
    std::unique_ptr<PQP_Model> model (new PQP_Model);

    char header[80] = "";         // Reads STL binary header
    input_file.read(header, 80);

    unsigned num_tris (0);  // Reads number of triangles
    input_file.read(reinterpret_cast<char*>(&num_tris), sizeof(num_tris));

    float vertex[3][3];  // Triangle represented as three vertices
    float surf_vec[3];

    model->BeginModel();

    short temp (0);  // Used for taking two bits of data after a triangle
    unsigned long counter (0);
    while(counter < num_tris) {
      input_file.read(reinterpret_cast<char*>(surf_vec), sizeof(surf_vec));

      input_file.read(reinterpret_cast<char*>(&vertex[0]), sizeof(vertex[0]));
      input_file.read(reinterpret_cast<char*>(&vertex[1]), sizeof(vertex[1]));
      input_file.read(reinterpret_cast<char*>(&vertex[2]), sizeof(vertex[2]));
      model->AddTri(vertex[0], vertex[1], vertex[2], counter);

      input_file.read(reinterpret_cast<char*>(&temp), sizeof(temp));
      ++counter;
    }

    model->EndModel();
    return model.release();
  }
  catch(...) {
    throw "File " + model_file + " error!";
  }
}

size_t PqpEnvironment::AddPoint(EVectorXd& q) {
  conf_sample_space_->addPoints(flann::Matrix<double> (q.data(), 1, q.size()));
  return conf_sample_space_->size() - 1;
}

double PqpEnvironment::CheckCollision(EVectorXd& q) {
  typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> EMatrix;
  typedef Eigen::Vector3f EVector;

  EMatrix R = EMatrix::Identity();
  EVector T (0.0, 0.0, 0.0);
  // Static environment
  EMatrix R_obs = EMatrix::Identity();
  EVector T_obs (0.0, 0.0, 0.0);

  PQP_DistanceResult distance_res;
  double min_distance (INFINITY); // Initialized at infinity
  for (size_t i (0); i < segments_.size(); ++i) {
    R = R * Eigen::AngleAxisf(q[i], EVector::UnitZ());
    PQP_Distance(&distance_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
      T.data(), segments_.at(i).get(),
      reinterpret_cast<PQP_REAL(*)[3]>(R_obs.data()), T_obs.data(),
      obstacles_.get(), 0.0, 0.0);

    if (distance_res.Distance() < min_distance)
      min_distance = distance_res.Distance();
    dh_table_.at(i).Transform(R, T);
  }
  return min_distance;
}

std::vector<int> PqpEnvironment::KnnQuery(EVectorXd& q, int k) {
  flann::Matrix<double> query (q.data(), 1, q.size());

  std::vector<int> vector_indices (k);
  flann::Matrix<int> indices (vector_indices.data(), 1, k);
  flann::Matrix<double> dists (new double[k], 1, k);

  conf_sample_space_->knnSearch(query, indices, dists, k,
                               flann::SearchParams(128));

  delete[] dists.ptr();
  return vector_indices;
}