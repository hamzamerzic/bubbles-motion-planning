#include "pqp_environment.h"

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

PqpEnvironment::PqpEnvironment(const std::string& robot_model_file,
                               const std::string& dh_table_file,
                               const std::string& obstacles_model_file,
                               RandomSpaceGeneratorInterface *random_generator,
                               const int sample_space_size) :
                               obstacles_(new PQP_Model),
                               conf_sample_space_(nullptr),
                               sample_space_size_(sample_space_size) {
  if(!LoadRobotModel(robot_model_file)) throw "Robot model problem!";
  if(!LoadDhTable(dh_table_file)) throw "DH table problem!";
  if(!LoadObstacles(obstacles_model_file)) throw "Obstacles problem!";
  if(!GenerateSampleSpace(random_generator, sample_space_size))
    throw "Sample space not generated!";
}

//TODO: Link number of segments
bool PqpEnvironment::LoadRobotModel(const std::string& robot_model_file) {
  std::ifstream input_file (robot_model_file.c_str());
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

                                                     //TODO: Make a function
      PQP_REAL tri1[3][3], tri2[3][3];               //to read these numbers
      //segments_.resize(num_segments);
      while (!parser.eof())
      {
        //Need to know number of segments and number of triangles
        //for each segment.
        //For now I assume each segment holds two triangles.
        int last (segments_.size());
        segments_.emplace_back(std::unique_ptr<PQP_Model> (new PQP_Model));
        segments_.at(last)->BeginModel();
        parser >> tri1[0][0] >> tri1[0][1] >> tri1[0][2] >>
                  tri1[1][0] >> tri1[1][1] >> tri1[1][2] >>
                  tri1[2][0] >> tri1[2][1] >> tri1[2][2];
        segments_.at(last)->AddTri(tri1[0], tri1[1], tri1[2], 0);
        parser >> tri2[0][0] >> tri2[0][1] >> tri2[0][2] >>
                  tri2[1][0] >> tri2[1][1] >> tri2[1][2] >>
                  tri2[2][0] >> tri2[2][1] >> tri2[2][2];
        segments_.at(last)->AddTri(tri2[0], tri2[1], tri2[2], 1);
        segments_.at(last)->EndModel();
      }
      //TODO: Add parsing options for two segment hand and test the results
      return true;
    }
    catch(...) { //TODO: Improve this catch block
      throw "File error!";
    }
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
  std::ifstream input_file (obstacles_model_file.c_str());
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

      obstacles_->BeginModel();
      int counter (0);
      PQP_REAL tri[3][3];
      while(!parser.eof()) {
      parser >> tri[0][0] >> tri[0][1] >> tri[0][2] >>
                tri[1][0] >> tri[1][1] >> tri[1][2] >>
                tri[2][0] >> tri[2][1] >> tri[2][2];
        obstacles_->AddTri(tri[0], tri[1], tri[2], counter);
      }
      obstacles_->EndModel();
      return true;
    }
    catch(...) {
      throw "File error!";
    }
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
    conf_sample_space_ = std::unique_ptr<FlannPointArray>(
      new FlannPointArray (
      flann::Matrix<double> (points_array.release(), sample_space_size,
      segments_.size()), flann::KDTreeIndexParams(4)));
    return true;
  } catch (...) {
    return false;
  }
}

double PqpEnvironment::CheckCollision(EVectorXd& q) {
  typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> EMatrix;
  typedef Eigen::Vector3d EVector;

  EMatrix R = EMatrix::Identity();
  EVector T (0.0, 0.0, 0.0);
  // Static environment
  EMatrix R_obs = EMatrix::Identity();
  EVector T_obs (0.0, 0.0, 0.0);

  PQP_DistanceResult distance_res;
  double min_distance (INFINITY); // Initialized at infinity
  for (size_t i (0); i < segments_.size(); ++i) {
    R = R * Eigen::AngleAxisd(q[i], EVector::UnitZ());
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