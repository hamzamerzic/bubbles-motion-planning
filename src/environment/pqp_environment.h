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

#include <cmath>
#include <PQP/PQP.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <flann/flann.hpp>
#include "../bubble.h"

#include "dh_parameter.h"
#include "random_generator/random_space_generator_interface.h"
#include "model_parser.h"

class PqpEnvironment {
 public:
  typedef flann::Index<flann::L2<double>> FlannPointArray;
  typedef Eigen::VectorXd EVectorXd;
  typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> EMatrix;
  typedef Eigen::Vector3f EVector3f;

  PqpEnvironment(const std::vector<std::string>& robot_model_files,
                 const std::string& dh_table_file,
                 const std::string& obstacles_model_file,
                 RandomSpaceGeneratorInterface* random_generator,
                 const int sample_space_size = 10000);

  int sample_space_size() { return conf_sample_space_->size(); }
  int dimension() { return dimension_; }
  double* GetPoint(int point_index) const {
    return conf_sample_space_->getPoint(point_index);
  }
  // Adds a point and returns it's index
  size_t AddPoint(EVectorXd& q);
  // Removes a point to potentially speed up the search
  void RemovePoint(int point_index);
  // Creates bubble - returns false upon failure
  bool MakeBubble(const EVectorXd& coordinates,
    std::shared_ptr<Bubble>& bubble);
  // Performs distance query - returns smallest distance to obstacles
  double DistanceQuery(EVectorXd& q);
  // Performs collision check - returns false upon collision
  bool CollisionQuery(EVectorXd& q);
  // Knn query - returns indices
  std::vector<int> KnnQuery(EVectorXd& q, int k);
  size_t CreatedBubbles() { return bubble_counter_; }

 private:
  const double kCylinderRadius = 1000.0;
  const double kMinDistanceToObstacles = 0.1;

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
  std::unique_ptr<FlannPointArray> conf_sample_space_;
  int sample_space_size_;
  size_t bubble_counter_;
  size_t dimension_;
  // Cylinder needed for generating bubbles
  std::unique_ptr<PQP_Model> cylinder_;
};

#endif  // PQP_ENVIRONMENT_H_INCLUDED
