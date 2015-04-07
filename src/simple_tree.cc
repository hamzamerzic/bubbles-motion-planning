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
#include "simple_tree.h"

bool SimpleTree::TryConnect (EVectorXd& point1, EVectorXd& point2) {
  EVectorXd point_step = (point2 - point1).normalized() * step_size_;
  EVectorXd point_iterator (point1);

  int step_number (static_cast<int>((point2 - point1).norm() / step_size_));
  int steps (0);
  while (steps <= step_number) {
    if (pqp_environment_->CheckCollision(point_iterator) < collision_limit_)
      return false;
    point_iterator += point_step;
    ++steps;
  }

  return pqp_environment_->CheckCollision(point2) >= collision_limit_;
}

bool SimpleTree::AddPoint(int point_index) {
  if (visited_.at(point_index)) return true;
  visited_.at(point_index) = true;

  int space_dimension (pqp_environment_->dimension());
  EVectorXd current_point = EVectorXd::Map(
    pqp_environment_->GetPoint(point_index), space_dimension);
  std::vector<int> query_indexes (pqp_environment_->KnnQuery(current_point, 5));

  for (auto& index : query_indexes) {
    EVectorXd temp = EVectorXd::Map(pqp_environment_->GetPoint(index),
      space_dimension);
    if(!visited_.at(index) && TryConnect(current_point, temp))
    {
      pq_.push(Edge(index, (end_ - temp).norm()));
      parents_.at(index) = point_index;
    }
  }
  return true;
}

bool SimpleTree::BuildTree() {
  size_t start_index (pqp_environment_->AddPoint(start_)),
    end_index (pqp_environment_->AddPoint(end_));
  pq_.push(Edge(start_index, (end_ - start_).norm()));
  parents_.at(start_index) = start_index;

  while (!pq_.empty()) {
    Edge current_edge (pq_.top()); pq_.pop();
    if (current_edge.point_index == end_index) return true;
    AddPoint(current_edge.point_index);
  }
  throw "Empty!";
}