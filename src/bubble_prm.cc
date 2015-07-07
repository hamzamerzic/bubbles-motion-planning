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
#include <fstream>
#include <iostream>
#include <chrono>

struct QueueConnectorBubbleContainer {
  QueueConnectorBubbleContainer(const std::shared_ptr<Bubble>& bubble1,
                                const BubblePrm::EVectorXd& hull_intersect1,
                                const std::shared_ptr<Bubble>& bubble2,
                                const BubblePrm::EVectorXd& hull_intersect2)
      : bubble1(bubble1), bubble2(bubble2), hull_intersect1(hull_intersect1),
        hull_intersect2(hull_intersect2) {}

  std::shared_ptr<Bubble> bubble1, bubble2;
  BubblePrm::EVectorXd hull_intersect1, hull_intersect2;
};

bool BubblePrm::ConnectPoints(int point1_index, int point2_index) {
  ++connects_;
  std::shared_ptr<Bubble> b1 = bubbles_.at(point1_index),
                          b2 = bubbles_.at(point2_index);

  // Bubble intersections are stored, because they can be computed only once,
  // along with bubble pointers in order to be able to connect the bubbles
  std::queue<QueueConnectorBubbleContainer> q_connector;

  EVectorXd left_endpoint = b1->HullIntersection(
        b2->coordinates() - b1->coordinates()),
    right_endpoint = b2->HullIntersection(
        b1->coordinates() - b2->coordinates());

  if ((left_endpoint - b1->coordinates()).norm() +
      (right_endpoint - b2->coordinates()).norm() >
      (b2->coordinates() - b1->coordinates()).norm()) {
    b2->SetParent(b1);
    return true;
  }
  q_connector.emplace(b1, left_endpoint, b2, right_endpoint);

  int counter = 0;
  while (!q_connector.empty() && counter++ < max_connect_param_) {
    auto q_edge = q_connector.front(); q_connector.pop();

    EVectorXd mid_coordinates =
        (q_edge.hull_intersect1 + q_edge.hull_intersect2) / 2;
    std::shared_ptr<Bubble> mid_bubble;

    if (!pqp_environment_->MakeBubble(mid_coordinates, mid_bubble)) {
      b2->parent().reset();  // Still no parents
      return false;
    }

    EVectorXd left_intersect = mid_bubble->HullIntersection(
          q_edge.hull_intersect1 - mid_coordinates),
      right_intersect = mid_bubble->HullIntersection(
          q_edge.hull_intersect2 - mid_coordinates);

    if ((left_intersect - mid_coordinates).norm() <
        (q_edge.hull_intersect1 - mid_coordinates).norm()) {
      q_connector.emplace(q_edge.bubble1, q_edge.hull_intersect1, mid_bubble,
          left_intersect);
      q_connector.emplace(mid_bubble, right_intersect, q_edge.bubble2,
          q_edge.hull_intersect2);
    } else {
      q_edge.bubble2->SetParent(mid_bubble);
      mid_bubble->SetParent(q_edge.bubble1);
    }
  }
  if (counter >= max_connect_param_) {
    b2->parent().reset();
    return false;
  }
  return true;
}

bool BubblePrm::AddPointToTree(int point_index, double extra_weight) {
  if (visited_.at(point_index)) return false;
  ++adds_;
  visited_.at(point_index) = true;
  pqp_environment_->RemovePoint(point_index);

  EVectorXd current_point_coordinates = GetCoordinates(point_index);
  std::vector<int> query_indices = pqp_environment_->KnnQuery(
      current_point_coordinates, knn_num_);

  for (auto& query_index : query_indices) {
    if (visited_.at(query_index))
      // Skips visited points
      continue;

    EVectorXd query_cords = GetCoordinates(query_index);

    if (bubbles_.at(query_index) != nullptr ||
        pqp_environment_->MakeBubble(query_cords, bubbles_.at(query_index)))
      pq_.emplace(point_index, query_index,
          (end_ - query_cords).norm() +
          (start_ - query_cords).norm()
          /*((end_ - query_cords).cwiseQuotient(
          (4 * bubbles_.at(query_index)->dimensions() +
          current_point_coordinates) / 5)).norm() +
          (start_ - query_cords).norm() * 0.1*/
          /*+ extra_weight + 0.25*/,
          0 /*extra_weight + 0.25 // hard-coded weight gain*/);
    else {
      visited_.at(query_index) = true;  // Indicate that a bubble cannot be
                                        // created at query_cords
      pqp_environment_->RemovePoint(query_index);
    }
  }
  return true;
}

bool BubblePrm::BuildTree(const std::string& log_filename) {
  std::cout << "**********BUILD STARTED**********" << std::endl;
  if (!pqp_environment_->MakeBubble(start_, bubbles_.at(start_index_))) {
    std::cout << "Collision at the initial configuration!" << std::endl;
    return false;
  }
  if (!pqp_environment_->MakeBubble(end_, bubbles_.at(end_index_))) {
    std::cout << "Collision at the final configuration!" << std::endl;
    return false;
  }
  bubbles_.at(end_index_).reset();

  std::ofstream log (log_filename);

  auto start_t = std::chrono::steady_clock::now();
  AddPointToTree(start_index_);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  log << std::chrono::duration<double, std::milli> (duration).count() <<
      std::endl;

  while (!visited_.at(end_index_) && !pq_.empty()) {
    Edge temp = pq_.top(); pq_.pop();
    if (visited_.at(temp.point2_index))
      continue;

    start_t = std::chrono::steady_clock::now();
    if (ConnectPoints(temp.point1_index, temp.point2_index))
      AddPointToTree(temp.point2_index, temp.extra_weight);

    end_t = std::chrono::steady_clock::now();
    duration = end_t - start_t;
    log << std::chrono::duration<double, std::milli> (duration).count() <<
        std::endl;
  }

  if (!pq_.empty() && bubbles_.at(end_index_)->parent() != nullptr) {
    std::cout << "**********BUILD SUCCESSFULL**********" << std::endl;
    std::cout << "Bubbles generated: " << pqp_environment_->CreatedBubbles() <<
      std::endl;
    std::cout << "Current q size: " << pq_.size() << std::endl;
    log << "Bubbles: " <<pqp_environment_->CreatedBubbles() << std::endl <<
      "Connects: " << connects_ << std::endl << "Adds: " << adds_ <<
      std::endl << "Q size: " << pq_.size() << std::endl << 1;
    return true;
  } else {
    std::cout << "**********BUILD UNSUCCESSFULL**********" << std::endl;
    std::cout << "Bubbles generated: " << pqp_environment_->CreatedBubbles() <<
      std::endl;
    std::cout << "Current q size: " << pq_.size() << std::endl;
    log << "Bubbles: " <<pqp_environment_->CreatedBubbles() << std::endl <<
      "Connects: " << connects_ << std::endl << "Adds: " << adds_ <<
      std::endl << "Q size: " << pq_.size() << std::endl << 0;
    return false;
  }
}

void BubblePrm::GeneratePath(const std::string& filename) {
  auto trajectory_it = bubbles_.at(end_index_);
  if (trajectory_it == nullptr) {
    std::cout << "Trajectory writing unsuccessful!" << std::endl;
    return;
  }

  std::cout << "Writing trajectory script to " << filename << "..." <<
    std::endl;

  std::ofstream file (filename);
  file << "# Autogenerated script. Copyright (C) 2015 Hamza Merzic." <<
    std::endl << "# Bubbles motion planning." << std::endl;

  file << "from robolink import *" << std::endl << "from robodk import *" <<
    std::endl << "RL = Robolink()" << std::endl << std::endl <<
    "robot = RL.Item('ABB IRB 120-3/0.6')" << std::endl;

  std::vector<EVectorXd> trajectory_deg;
  while (trajectory_it != nullptr) {
    trajectory_deg.push_back(180 * trajectory_it->coordinates() / M_PI);
    trajectory_it = trajectory_it->parent();
  }

  file << "robot.setJoints([";
  for (unsigned i = 0; i < trajectory_deg.rbegin()->size() - 1; ++i) {
    file << (*trajectory_deg.rbegin())[i] << ", ";
  }
  file << (*trajectory_deg.rbegin())[trajectory_deg.rbegin()->size() - 1] <<
    "])" << std::endl;

  for (auto it = trajectory_deg.rbegin() + 1; it != trajectory_deg.rend();
      ++it) {
    file << "robot.MoveJ([";
    for (int i = 0; i < it->size() - 1; ++i) {
      file << (*it)[i] << ", ";
    }
    file << (*it)[it->size() - 1] << "])" << std::endl;
  }

  std::cout << "Trajectory successfully written!" << std::endl <<
               "--------------------------------" << std::endl << std::endl;
}
