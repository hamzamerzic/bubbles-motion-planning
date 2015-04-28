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
#include <memory>
#include <queue>
#include <tuple>
#include <iostream>

const int max_connect_param = 16;



bool BubblePrm::ConnectPoints(int point1_index, int point2_index) {
  if (visited_.at(point2_index))
    return false;
  std::shared_ptr<Bubble> b1 (bubbles_.at(point1_index)),
    b2 (bubbles_.at(point2_index));

  // We store bubble intersections, because those must be computed only once,
  // along with pointers to bubbles in order to be able to connect the bubbles
  std::queue<std::tuple<EVectorXd, EVectorXd, Bubble*, Bubble*>> q_connector;
  q_connector.emplace(HullIntersection(*b1, b2->coordinates()),
    HullIntersection(*b2, b1->coordinates()), b1.get(), b2.get());

  int counter (0);
  while (!q_connector.empty() && counter++ < max_connect_param) {
    auto& q_edge (q_connector.front()); q_connector.pop();

    EVectorXd mid ((std::get<0>(q_edge) + std::get<1>(q_edge)) / 2);
    std::shared_ptr<Bubble> mid_bubble;
    if (!pqp_environment_->MakeBubble(mid, mid_bubble.get())) {
      b2->SetParent(nullptr);  // Still no parents
      return false;
    }

    EVectorXd left_intersect (HullIntersection(*mid_bubble,
        std::get<0>(q_edge))),
      right_intersect (HullIntersection(*mid_bubble, std::get<1>(q_edge)));

    if ((left_intersect - mid).norm() < (std::get<0>(q_edge) - mid).norm()) {
      q_connector.push(std::make_tuple(std::get<0>(q_edge), left_intersect,
        std::get<2>(q_edge), mid_bubble.get()));
    }
    else {
      mid_bubble->SetParent(std::get<2>(q_edge));
    }
    if ((right_intersect - mid).norm() < (std::get<1>(q_edge) - mid).norm()) {
      q_connector.push(std::make_tuple(right_intersect, std::get<1>(q_edge),
        mid_bubble.get(), std::get<3>(q_edge)));
    }
    else {
      std::get<3>(q_edge)->SetParent(mid_bubble.get());
    }
  }

  return counter < max_connect_param;
}

bool BubblePrm::AddPointToTree(int point_index) {
  if (visited_.at(point_index)) return true;
  visited_.at(point_index) = true;

  EVectorXd current_point_coordinates = GetCoordinates(point_index);
  std::vector<int> query_indices (pqp_environment_->KnnQuery(
      current_point_coordinates, 15));

  for (auto& query_index : query_indices) {
    if (visited_.at(query_index))  // Checks for previous Connect attempts
      continue;

    EVectorXd query_cords = GetCoordinates(query_index);
    if (!pqp_environment_->MakeBubble(query_cords,
        bubbles_.at(query_index).get())) {
      visited_.at(query_index) = true;  // Indicate that a bubble cannot be
                                        // created at query_coo
      continue;
    }

    if (ConnectPoints(point_index, query_index)) {
      pq_.push(Edge(point_index, query_index, (end_ - query_cords).norm()));
    }
  }

  return true;
}

bool BubblePrm::BuildTree() {
  if (!pqp_environment_->MakeBubble(start_, bubbles_.at(start_index_).get()))
    return false;
  AddPointToTree(start_index_);

  while (!pq_.empty() && !visited_.at(end_index_)) {
    auto& temp (pq_.top()); pq_.pop();
    AddPointToTree(temp.point2_index);
  }

  return !pq_.empty();
}

void BubblePrm::LogResults() {
  auto it (bubbles_.at(end_index_));
  if (it == nullptr) {
    std::cout << "Tree creation unsuccessful!" << std::endl;
    return;
  }

  while (it != nullptr) {
    std::cout << it->coordinates() << std::endl;
    it = it->parent();
  }
}

BubblePrm::EVectorXd BubblePrm::GetCoordinates(int point_index) const {
  return EVectorXd::Map(pqp_environment_->GetPoint(point_index),
    pqp_environment_->dimension());
}

BubblePrm::EVectorXd BubblePrm::HullIntersection(const Bubble& b1,
    const EVectorXd& b2_coordinates) {
  EVectorXd direction (b2_coordinates - b1.coordinates());
  return direction / fabs((direction.cwiseQuotient(b1.dimensions())).sum());
}