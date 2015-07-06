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
#ifndef PRM_TREE_H_INCLUDED
#define PRM_TREE_H_INCLUDED

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "environment/pqp_environment.h"

class PrmTree {
 public:
  typedef Eigen::VectorXd EVectorXd;
  PrmTree(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end,
          int knn_num)
      : pqp_environment_(pqp_environment), start_(start), end_(end),
        start_index_(pqp_environment_->AddPoint(start)),
        end_index_(pqp_environment_->AddPoint(end)), knn_num_(knn_num),
        space_size_(pqp_environment->sample_space_size() + 2),
        visited_(std::vector<bool>(space_size_, false)) {}

  virtual EVectorXd GetCoordinates(int point_index) const {
    return EVectorXd::Map(pqp_environment_->GetPoint(point_index),
        pqp_environment_->dimension());
  }
  virtual bool ConnectPoints(int point1_index, int point2_index) = 0;
  virtual bool AddPointToTree(int point_index, double extra_weight) = 0;
  virtual bool BuildTree(const std::string& log_filename) = 0;
  virtual void GeneratePath(const std::string& filename) = 0;

 protected:
  std::unique_ptr<PqpEnvironment> pqp_environment_;
  EVectorXd start_, end_;
  int start_index_, end_index_;
  int knn_num_, space_size_;
  std::vector<bool> visited_;
};

#endif  // PRM_TREE_H_INCLUDED
