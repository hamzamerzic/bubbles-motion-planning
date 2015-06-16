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
#ifndef BUBBLE_PRM_H_INCLUDED
#define BUBBLE_PRM_H_INCLUDED

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <memory>
#include <string>

#include "prm_tree.h"
#include "environment/pqp_environment.h"
#include "bubble.h"

struct Edge {
  Edge(int point1_index, int point2_index, double weight,
       double extra_weight)
      : point1_index(point1_index), point2_index(point2_index),
        weight(weight), extra_weight(extra_weight) {}

  int point1_index, point2_index;
  double weight, extra_weight;
};

struct EdgeCompareFunctor {
  bool operator() (const Edge& e1, const Edge& e2) const {
    return e1.weight > e2.weight;
  }
};

class BubblePrm : PrmTree {
 public:
  typedef Eigen::VectorXd EVectorXd;
  BubblePrm(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end,
            int knn_num,  // Number of nearest neighbors
            int max_connect_param = 128  // Max binary splits for ConnectPoints
            )
      : PrmTree(pqp_environment, start, end, knn_num),
        bubbles_(space_size_, nullptr), max_connect_param_(max_connect_param) {}

  virtual bool ConnectPoints(int point1_index, int point2_index);
  virtual bool AddPointToTree(int point_index, double extra_weight = 0);
  virtual bool BuildTree();
  virtual void LogResults(const std::string& filename);

 private:
  std::vector<std::shared_ptr<Bubble>> bubbles_;
  double step_size_, collision_limit_;
  int max_connect_param_;
  std::priority_queue<Edge, std::vector<Edge>, EdgeCompareFunctor> pq_;
};

#endif  // BUBBLE_PRM_H_INCLUDED
