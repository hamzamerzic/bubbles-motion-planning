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

class Bubble {
public:
  Bubble(): distance_(INFINITY) {}

  double& distance() { return distance_; }
  const double Get(size_t i) const { return coordinates_.at(i); }
  void Set(size_t i, double value) { coordinates_.at(i) = value; }
  void Resize(size_t n) { coordinates_.resize(n); }

private:
  std::vector<double> coordinates_;
  double distance_;
};

#endif // BUBBLE_H_INCLUDED