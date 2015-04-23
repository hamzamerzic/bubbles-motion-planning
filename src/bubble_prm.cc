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
#include "bubble_prm.h"

bool BubblePrm::AddPoint(int point_index) {
  if (visited_.at(point_index)) return true;
  visited_.at(point_index) = true;

  int space_dimension (pqp_environment_->dimension());
  EVectorXd current_point = EVectorXd::Map(
    pqp_environment_->GetPoint(point_index), space_dimension);
  std::vector<int> query_indices (pqp_environment_->KnnQuery(current_point, 5));

  for (auto& index : query_indices) {
    EVectorXd temp = EVectorXd::Map(pqp_environment_->GetPoint(index),
      space_dimension);
    if (!visited_.at(index) && Connect(current_point, temp)) {
      pq_.push(Edge(index, (end_ - temp).norm()));
      parents_.at(index) = point_index;
    }
  }
  return true;
}

bool BubblePrm::BuildTree() {
  return true;
}

bool BubblePrm::Connect() {

  return true;
}