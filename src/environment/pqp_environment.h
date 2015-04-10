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
#ifndef PQP_ENVIRONMENT_H_INCLUDED
#define PQP_ENVIRONMENT_H_INCLUDED

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include "../bubble.h"

#include "dh_parameter.h"
#include "random_generator/random_space_generator_interface.h"
#include "model_parser.h"

class PqpEnvironment {
  typedef flann::Index<flann::L2<double>> FlannPointArray;
  typedef Eigen::VectorXd EVectorXd;
  typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> EMatrix;
  typedef Eigen::Vector3f EVector3f;

public:
  PqpEnvironment(const std::vector<std::string>& robot_model_files,
                 const std::string& dh_table_file,
                 const std::string& obstacles_model_file,
                 RandomSpaceGeneratorInterface* random_generator,
                 const int sample_space_size = 1000000);

  int sample_space_size() { return conf_sample_space_->size(); }
  int dimension() { return dimension_; }
  double* GetPoint(int point_index) const {
    return conf_sample_space_->getPoint(point_index);
  }
  // Adds a point and returns it's index
  size_t AddPoint(EVectorXd& q);
  // Collision query - returns distance
  double CheckCollision(EVectorXd& q);
  // Creates bubble - returns false if unable
  bool MakeBubble(EVectorXd& q, Bubble& bubble);
  // Knn query - returns indices
  std::vector<int> KnnQuery(EVectorXd& q, int k);

private:
  bool LoadRobotModel(const std::vector<std::string>& robot_mode_files);
  bool LoadRobotParameters(const std::string& parameters_file);
  bool LoadObstacles(const std::string& obstacles_model_file);
  bool GenerateSampleSpace(
    RandomSpaceGeneratorInterface* random_generator,
    const int sample_space_size);

  std::unique_ptr<PQP_Model> obstacles_;
  std::vector<std::unique_ptr<PQP_Model>> segments_;
  std::vector<DhParameter> dh_table_;
  ModelParser parser_;
  FlannPointArray* conf_sample_space_;
  int sample_space_size_;
  size_t dimension_;
  // Cylinder needed for generating bubbles
  std::unique_ptr<PQP_Model> cylinder_;
};

#endif // PQP_ENVIRONMENT_H_INCLUDED
