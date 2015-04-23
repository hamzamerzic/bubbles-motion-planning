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
#include "two_seg_prm.h"

#include <iostream>

bool TwoSegPrm::ConnectPoints(int point1_index, int point2_index) {
  EVectorXd point1 (GetPoint(point1_index)), point2 (GetPoint(point2_index));
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

bool TwoSegPrm::AddPointToTree(int point_index) {
  if (visited_.at(point_index)) return true;
  visited_.at(point_index) = true;

  EVectorXd current_point = GetPoint(point_index);
  std::vector<int> query_indices (pqp_environment_->KnnQuery(
      current_point, 10));

  for (auto& query_index : query_indices) {
    EVectorXd temp = GetPoint(query_index);
    if (!visited_.at(query_index) && ConnectPoints(point_index, query_index)) {
      pq_.push(Edge(query_index, (end_ - temp).norm()));
      parents_.at(query_index) = point_index;
    }
  }
  return true;
}

bool TwoSegPrm::BuildTree() {
  pq_.push(Edge(start_index_, (end_ - start_).norm()));
  parents_.at(start_index_) = start_index_;

  while (!pq_.empty()) {
    Edge current_edge (pq_.top()); pq_.pop();
    if (current_edge.point_index == end_index_) return true;
    AddPointToTree(current_edge.point_index);
  }
  throw "Empty!";
}

void TwoSegPrm::LogResults() {
  if(parents_.at(end_index_) == -1) {
    std::cout << "Not connected!" << std::endl;
    return;
  }
  int index (end_index_);
  while (index != start_index_) {
    std::cout << "Index: " << index << std::endl <<
      GetPoint(index).transpose() << std::endl;
    index = parents_.at(index);
  }
  std::cout << "Index: " << start_index_ << std::endl <<
    GetPoint(start_index_).transpose() << std::endl;
}

TwoSegPrm::EVectorXd TwoSegPrm::GetPoint(int point_index) {
  return EVectorXd::Map(pqp_environment_->GetPoint(point_index),
    pqp_environment_->dimension());
}

int TwoSegPrm::InsertPoint(EVectorXd& point) {
  parents_.push_back(-1); ++space_size_; visited_.push_back(false);
  return pqp_environment_->AddPoint(point);
}