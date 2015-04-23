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
#ifndef TWO_SEG_PRM_H_INCLUDED
#define TWO_SEG_PRM_H_INCLUDED

#include <vector>
#include <queue>
#include <Eigen/Dense>

#include "prm_tree.h"
#include "environment/pqp_environment.h"

struct Edge {
  Edge(int point_index, double distance): point_index (point_index),
                                             distance (distance) {}
  int point_index;
  double distance;
};

struct EdgeCompareFunctor {
  bool operator() (const Edge& e1, const Edge& e2) const {
    return e1.distance > e2.distance;
  }
};

class TwoSegPrm : PrmTree {
public:
  TwoSegPrm(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end,
      double step_size, double collision_limit = 0.01):
      PrmTree (pqp_environment, start, end),
      parents_ (std::vector<int>(space_size_, -1)),
      step_size_ (step_size), collision_limit_ (collision_limit) {}

  virtual bool AddPointToTree(int point_index);
  virtual bool BuildTree();
  virtual bool ConnectPoints(int point1_index, int point2_index);
  virtual void LogResults();
  EVectorXd GetPoint(int point_index) {
    return EVectorXd::Map(pqp_environment_->GetPoint(point_index),
      pqp_environment_->dimension());
  }
  int InsertPoint(EVectorXd& point) {
    parents_.push_back(-1); ++space_size_; visited_.push_back(false);
    return pqp_environment_->AddPoint(point);
  }

private:
  std::vector<int> parents_;
  double step_size_;
  double collision_limit_;
  std::priority_queue<Edge, std::vector<Edge>, EdgeCompareFunctor> pq_;
};

#endif // TWO_SEG_PRM_H_INCLUDED