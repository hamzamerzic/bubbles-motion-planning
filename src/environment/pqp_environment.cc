/*
 * Copyright (C) 2015 Hamza Merzić
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "pqp_environment.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iostream>

PqpEnvironment::PqpEnvironment(const std::vector<std::string>&
                                   robot_model_files,
                               const std::string& dh_table_file,
                               const std::string& obstacles_model_file,
                               RandomSpaceGeneratorInterface *random_generator,
                               const int sample_space_size)
    : obstacles_(new PQP_Model), conf_sample_space_(nullptr),
      sample_space_size_(sample_space_size), bubble_counter_(0) {
  if (!LoadRobotParameters(dh_table_file)) throw "DH table problem!";
  if (!LoadRobotModel(robot_model_files)) throw "Robot model problem!";
  if (!LoadObstacles(obstacles_model_file)) throw "Obstacles problem!";
  if (!GenerateSampleSpace(random_generator, sample_space_size))
    throw "Sample space not generated!";

  dimension_ = segments_.size();
  try {
    cylinder_ = std::unique_ptr<PQP_Model>(parser_.GetModel("cylinder.stl"));
  } catch(std::string& s) {
    std::cout << s;
    throw s;
  }
}

bool PqpEnvironment::LoadRobotModel(
      const std::vector<std::string>& robot_model_files) {
  std::cout << "Loading robot model... ";
  try {
    EMatrix R = EMatrix::Identity();
    EVector3f T (0.0, 0.0, 0.0);

    for (const auto& model_file : robot_model_files) {
      if (segments_.size() >= 1)
        dh_table_[segments_.size() - 1].InverseTransform(R, T);

      segments_.emplace_back(parser_.GetTransformModel(model_file, R, T));
    }

    std::cout << "Robot model successfully loaded!" << std::endl;
    return true;
  } catch (...) {
    throw;
  }
  return false;  // Input file not present
}

// TODO(hamza): Add limits parsing
bool PqpEnvironment::LoadRobotParameters(const std::string& parameters_file) {
  std::ifstream input_file (parameters_file.c_str());
  std::cout << "Loading robot parameters... ";
  if (input_file) {
    try {
      input_file.seekg(0, std::ios::end);            // End of file
      std::streampos length (input_file.tellg());    // Read the size
      input_file.seekg(0, std::ios::beg);            // Return to beginning

      std::vector<char> buffer (length);
      input_file.read(&buffer[0], length);

      // Move buffer to stringstream parser
      std::stringstream parser;
      parser.rdbuf()->pubsetbuf(&buffer[0], length);

      double theta, d, a, alpha;
      while (!parser.eof()) {
        parser >> theta >> d >> a >> alpha;
        dh_table_.emplace_back(theta, d, a, alpha);
      }

      std::cout << "Robot parameters successfully loaded!" << std::endl;
      return true;
    }
    catch(...) {
      throw "File error!";
    }
  }
  return false;  // Input file not present
}

bool PqpEnvironment::LoadObstacles(const std::string& obstacles_model_file) {
  std::cout << "Loading obstacles model... ";
  try {
    obstacles_ = std::unique_ptr<PQP_Model>(
      parser_.GetModel(obstacles_model_file));

    std::cout << "Obstacles model successfully loaded!" << std::endl;
    return true;
  }
  catch (...) {
    throw;
  }
  return false;  // Input file not present
}

bool PqpEnvironment::GenerateSampleSpace(
    RandomSpaceGeneratorInterface* random_generator,
    const int sample_space_size) {
  std::cout << "Generating sample space... ";
  try {
    // Create configuration sample space
    conf_sample_space_ = std::unique_ptr<FlannPointArray> (new FlannPointArray (
        flann::Matrix<double> (
        random_generator->CreateSampleSpace(sample_space_size).release(),
        sample_space_size, static_cast<int>(segments_.size())),
        flann::KDTreeIndexParams(4)));
    conf_sample_space_->buildIndex();

    std::cout << "Sample space successfully generated!" << std::endl;
    return true;
  } catch (...) {
    return false;
  }
}

size_t PqpEnvironment::AddPoint(EVectorXd& q) {
  conf_sample_space_->addPoints(flann::Matrix<double> (q.data(), 1, q.size()));
  return conf_sample_space_->size() - 1;
}

void PqpEnvironment::RemovePoint(int point_index) {
  conf_sample_space_->removePoint(point_index);
}

bool PqpEnvironment::MakeBubble(const EVectorXd& coordinates,
    std::shared_ptr<Bubble>& bubble) {

  ++bubble_counter_;
  EMatrix R = EMatrix::Identity();
  EVector3f T (0.0, 0.0, 0.0);
  // Static environment/cylinder transform
  EMatrix R_temp = EMatrix::Identity();
  EVector3f T_temp (0.0, 0.0, 0.0);

  PQP_DistanceResult distance_res;
  bubble = std::shared_ptr<Bubble>(new Bubble(coordinates));

  // Defining effective radius for amortizing the resolution of the 3D cylinder
  // model
  double effective_radius = kCylinderRadius - 1.204543795;
  double axis_distance = 0;

  // Finding minimal distance to obstacles and updating bubble's first
  // dimension
  for (size_t i = 0; i < dimension_; ++i) {
    R = R * Eigen::AngleAxisf(coordinates[i], EVector::UnitZ());
    PQP_Distance(&distance_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
        T.data(), segments_.at(i).get(),
        reinterpret_cast<PQP_REAL(*)[3]>(R_temp.data()), T_temp.data(),
        obstacles_.get(), 0.0, 0.0);

    if (distance_res.Distance() < kMinDistanceToObstacles) {
      bubble.reset();
      return false;  // Too close to obstacle
    }
    if (distance_res.Distance() < bubble->distance())
      bubble->distance() = distance_res.Distance();

    PQP_Distance(&distance_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
        T.data(), segments_.at(i).get(),
        reinterpret_cast<PQP_REAL(*)[3]>(R_temp.data()), T_temp.data(),
        cylinder_.get(), 0.0, 0.0);

    if (effective_radius - distance_res.Distance() > axis_distance)
      axis_distance = effective_radius - distance_res.Distance();
    dh_table_.at(i).Transform(R, T);
  }

  bubble->SetDimension(0, bubble->distance() / axis_distance);

  // Update bubble dimensions
  for (size_t i = 1; i < dimension_; ++i) {
    R_temp = R_temp * Eigen::AngleAxisf(coordinates[i - 1], EVector::UnitZ());
    dh_table_.at(i - 1).Transform(R_temp, T_temp);
    R = R_temp;
    T = T_temp;
    for (size_t k = i; k < dimension_; ++k) {
      R = R * Eigen::AngleAxisf(coordinates[k], EVector::UnitZ());
      PQP_Distance(&distance_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
          T.data(), segments_.at(k).get(),
          reinterpret_cast<PQP_REAL(*)[3]>(R_temp.data()), T_temp.data(),
          cylinder_.get(), 0.0, 0.0);

      axis_distance = effective_radius - distance_res.Distance();
      if (bubble->distance() / axis_distance < bubble->GetDimension(i)) {
        bubble->SetDimension(i, bubble->distance() / axis_distance);
      }
      dh_table_.at(k).Transform(R, T);
    }
  }
  return true;
}

double PqpEnvironment::DistanceQuery(EVectorXd& q) {
  EMatrix R = EMatrix::Identity();
  EVector3f T (0.0, 0.0, 0.0);
  // Static environment transform
  EMatrix R_temp = EMatrix::Identity();
  EVector3f T_temp (0.0, 0.0, 0.0);

  PQP_DistanceResult distance_res;
  double min_distance = INFINITY;  // Initialized at infinity

  for (size_t i = 0; i < dimension_; ++i) {
    R = R * Eigen::AngleAxisf(q[i], EVector::UnitZ());
    PQP_Distance(&distance_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
      T.data(), segments_.at(i).get(),
      reinterpret_cast<PQP_REAL(*)[3]>(R_temp.data()), T_temp.data(),
      obstacles_.get(), 0.0, 0.0);

    if (distance_res.Distance() < kMinDistanceToObstacles)
      return 0;  // Too close to obstacle, return 0
    if (distance_res.Distance() < min_distance)
      min_distance = distance_res.Distance();

    dh_table_.at(i).Transform(R, T);
  }

  return min_distance;
}

bool PqpEnvironment::CollisionQuery(EVectorXd& q) {
  EMatrix R = EMatrix::Identity();
  EVector3f T (0.0, 0.0, 0.0);
  // Static environment transform
  EMatrix R_temp = EMatrix::Identity();
  EVector3f T_temp (0.0, 0.0, 0.0);

  PQP_CollideResult collision_res;

  for (size_t i = 0; i < dimension_; ++i) {
    R = R * Eigen::AngleAxisf(q[i], EVector::UnitZ());
    PQP_Collide(&collision_res, reinterpret_cast<PQP_REAL(*)[3]>(R.data()),
      T.data(), segments_.at(i).get(),
      reinterpret_cast<PQP_REAL(*)[3]>(R_temp.data()), T_temp.data(),
      obstacles_.get(), PQP_FIRST_CONTACT);

    if (collision_res.NumPairs())
      return false;  // Collision

    dh_table_.at(i).Transform(R, T);
  }

  return true;
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
