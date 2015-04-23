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

#include <vector>
#include <queue>
#include <Eigen/Dense>

#include "prm_tree.h"
#include "environment/pqp_environment.h"

struct Edge {
  Edge(std::shared_ptr<Bubble>& b1, std::shared_ptr<Bubble>& b2,
      double weight): b1 (b1), b2 (b2), weight (weight);

  std::shared_ptr<Bubble> b1, b2;
  double weight;
};

struct EdgeCompareFunctor {
  bool operator() (const Edge& e1, const Edge& e2) const {
    return e1.weight > e2.weight;
  }
};

class BubblePrm : PrmTree {
public:
  BubblePrm(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end,
      double step_size, double collision_limit = 0.01):
      PrmTree (pqp_environment, start, end),
      step_size_ (step_size), collision_limit_ (collision_limit) {}

  virtual bool AddPoint(int point_index);
  virtual bool BuildTree();
  bool Connect(Bubble& b1, Bubble& b2);

private:
  EVectorXd HullIntersection(const Bubble& b1, const Bubble& b2) const;

  double step_size_;
  double collision_limit_;
  std::priority_queue<Edge, std::vector<Edge>, EdgeCompareFunctor> pq_;
};

#endif // BUBBLE_PRM_H_INCLUDED
