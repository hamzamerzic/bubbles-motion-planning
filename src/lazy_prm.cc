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
#include "lazy_prm.h"
#include <cmath>
#include <memory>
#include <queue>
#include <fstream>
#include <iostream>
#include <chrono>

namespace lazyprm {

bool LazyPrm::ConnectPoints(int point1_index, int point2_index) {
  ++connects_;
  EVectorXd point1 = GetCoordinates(point1_index),
            point2 = GetCoordinates(point2_index),
            direction = (point2 - point1).normalized();

  EVectorXd temp_point = point1 + direction * step_size_;
  while ((point2 - temp_point).norm() > step_size_) {
    if (!pqp_environment_->CollisionQuery(temp_point))
      return false;
    temp_point += direction * step_size_;
  }

  parents_.at(point2_index) = point1_index;
  return true;
}

bool LazyPrm::AddPointToTree(int point_index, double extra_weight) {
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
    if (pqp_environment_->CollisionQuery(query_cords)) {
      pq_.emplace(point_index, query_index,
          (end_ - query_cords).norm() +
          (start_ - query_cords).norm(),
          0 /*extra_weight + 0.25 // hard-coded weight gain*/);
    }
    else {
      visited_.at(query_index) = true;  // Indicate that a bubble cannot be
                                        // created at query_cords
      pqp_environment_->RemovePoint(query_index);
    }
  }
  return true;
}

bool LazyPrm::BuildTree(const std::string& log_filename) {
  std::cout << "**********BUILD STARTED**********" << std::endl;
  EVectorXd start_coordinates = GetCoordinates(start_index_),
            end_coordinates = GetCoordinates(end_index_);
  if (!pqp_environment_->CollisionQuery(start_coordinates)) {
    std::cout << "Collision at the initial configuration!" << std::endl;
    return false;
  }
  if (!pqp_environment_->CollisionQuery(end_coordinates)) {
    std::cout << "Collision at the final configuration!" << std::endl;
    return false;
  }

  std::ofstream log (log_filename);

  auto start_t = std::chrono::steady_clock::now();
  AddPointToTree(start_index_);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  log << std::chrono::duration<double, std::milli> (duration).count() <<
      std::endl;

  while (!pq_.empty() && !visited_.at(end_index_)) {
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

  if (parents_.at(end_index_) != -1) {
    std::cout << "**********BUILD SUCCESSFULL**********" << std::endl;
    std::cout << "Collision checks: " << pqp_environment_->CollisionChecks() <<
      std::endl;
    std::cout << "Current q size: " << pq_.size() << std::endl;
    log << "Collision checks: " <<pqp_environment_->CollisionChecks() <<
      std::endl << "Connects: " << connects_ << std::endl << "Adds: " <<
      adds_ << std::endl << "Q size: " << pq_.size() << std::endl << 1;
    return true;
  } else {
    std::cout << "**********BUILD UNSUCCESSFULL**********" << std::endl;
    std::cout << "Collision checks: " << pqp_environment_->CollisionChecks() <<
      std::endl;
    std::cout << "Current q size: " << pq_.size() << std::endl;
    log << "Collision checks: " <<pqp_environment_->CollisionChecks() <<
      std::endl << "Connects: " << connects_ << std::endl << "Adds: " <<
      adds_ << std::endl << "Q size: " << pq_.size() << std::endl << 0;
    return false;
  }
}

void LazyPrm::GeneratePath(const std::string& filename) {
  auto trajectory_it = parents_.at(end_index_);
  if (trajectory_it == -1) {
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
  trajectory_deg.push_back(180 * GetCoordinates(end_index_) / M_PI);
  while (trajectory_it != -1) {
    trajectory_deg.push_back(180 * GetCoordinates(trajectory_it) / M_PI);
    trajectory_it = parents_.at(trajectory_it);
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

}  // namespace lazyprm
