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
#ifndef BUBBLE_H_INCLUDED
#define BUBBLE_H_INCLUDED

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <memory>

class Bubble {
 public:
  typedef Eigen::VectorXd EVectorXd;
  Bubble() : distance_(INFINITY), parent_(nullptr) {}
  explicit Bubble(const EVectorXd& coordinates)
      : coordinates_(coordinates),
        dimensions_(INFINITY * EVectorXd::Ones(coordinates_.size())),
        distance_(INFINITY),
        parent_(nullptr) {}

  const EVectorXd& coordinates() const { return coordinates_; }
  const EVectorXd& dimensions() const { return dimensions_; }
  std::shared_ptr<Bubble>& parent() { return parent_; }
  double& distance() { return distance_; }
  const double GetCoordinate(size_t i) const { return coordinates_[i]; }
  const double GetDimension(size_t i) const { return dimensions_[i]; }
  void SetDimension(size_t i, double value) { dimensions_[i] = value; }
  void SetParent(const std::shared_ptr<Bubble>& parent) {
    parent_ = parent; }

  void Resize(size_t dimension) {
    coordinates_.resize(dimension);
    dimensions_.resize(dimension);
  }
  EVectorXd HullIntersection(const EVectorXd& direction) {
    return coordinates() +
      direction / ((direction.cwiseQuotient(dimensions())).cwiseAbs()).sum();
  }

 private:
  EVectorXd coordinates_, dimensions_;
  double distance_;
  std::shared_ptr<Bubble> parent_;
};

#endif  // BUBBLE_H_INCLUDED
