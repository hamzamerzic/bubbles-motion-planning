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

#include <vector>
#include <cmath>
#include <utility>
#include <Eigen/Dense>

class Bubble {
public:
  typedef Eigen::VectorXd EVectorXd;
  Bubble(): distance_(INFINITY), parent_(nullptr) {}
  Bubble(EVectorXd& coordinates): coordinates_ (coordinates),
      dimensions_ (EVectorXd(coordinates_.size())), distance_(INFINITY),
      parent_(nullptr) {}

  EVectorXd coordinates() const { return coordinates_; }
  EVectorXd dimensions() const { return dimensions_; }
  double& distance() { return distance_; }
  const double Get(size_t i) const { return dimensions_[i]; }
  void Set(size_t i, double value) { dimensions_[i] = value; }
  void SetParent(Bubble* parent) { parent_ = std::shared_ptr<Bubble> (parent); }
  void Resize(size_t dimension) {
    coordinates_.resize(dimension);
    dimensions_.resize(dimension);
  }

private:
  EVectorXd coordinates_, dimensions_;
  double distance_;
  std::shared_ptr<Bubble> parent_;
};

#endif // BUBBLE_H_INCLUDED