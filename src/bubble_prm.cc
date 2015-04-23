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
#include <cmath>

bool BubblePrm::ConnectPoints(int point1_index, int point2_index) {

  return true;
}

bool BubblePrm::AddPointToTree(int point_index) {
  if (visited_.at(point_index)) return true;
  visited_.at(point_index) = true;

  EVectorXd current_point = GetCoordinates(point_index);
  std::vector<int> query_indices (pqp_environment_->KnnQuery(
      current_point, 5));

  for (auto& query_index : query_indices) {
    EVectorXd temp = GetCoordinates(query_index);
    if (!visited_.at(query_index) && ConnectPoints(point_index, query_index)) {
      pq_.push(Edge(point_index, query_index, (end_ - temp).norm()));
      // parents_.at(query_index) = point_index;
    }
  }
  return true;
}

bool BubblePrm::BuildTree() {
  return true;
}

void BubblePrm::LogResults() {

}

BubblePrm::EVectorXd BubblePrm::GetCoordinates(int point_index) const {
  return EVectorXd::Map(pqp_environment_->GetPoint(point_index),
    pqp_environment_->dimension());
}

BubblePrm::EVectorXd BubblePrm::HullIntersection(const Bubble& b1,
    const Bubble& b2) const {
  EVectorXd direction (b2.coordinates() - b1.coordinates());
  return direction / fabs((direction.cwiseQuotient(b1.dimensions())).sum());
}