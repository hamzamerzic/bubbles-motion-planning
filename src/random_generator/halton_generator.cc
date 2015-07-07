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
#include "halton_generator.h"

HaltonGenerator::HaltonGenerator(
    const std::vector<std::pair<double, double>>& limits)
    : limits_(limits), space_dimension_(limits.size()) {
  std::random_device rd;
  halton_index_ = 2 + (rd() % 1001);

  int counter = 3;
  primes_.push_back(2);
  while(primes_.size() < space_dimension_) {
    bool prime = true;
    for (int i = 3; i * i <= counter; i += 2) {
      if (counter % i == 0)
        prime = false;
    }
    if (prime)
      primes_.push_back(counter);
    counter += 2;
  }
}

std::vector<double> HaltonGenerator::CreatePoint() {
  std::vector<double> point (space_dimension_);
  for (unsigned i = 0; i < space_dimension_; ++i) {
    point[i] = limits_[i].first;
    double result = 0.0, f = 1.0;
    size_t index = halton_index_;
    while (index > 0) {
      f /= primes_[i];
      result += f * (index % primes_[i]);
      index /= primes_[i];
    }
    point[i] += (limits_[i].second - limits_[i].first) * result;
  }

  ++halton_index_;
  return point;
}

std::unique_ptr<double[]> HaltonGenerator::CreateSampleSpace(size_t num_points) {
  std::unique_ptr<double[]> sample_space (
      new double[num_points * space_dimension_]);
  for (size_t i = 0; i < num_points; ++i) {
    std::vector<double> point = CreatePoint();
    for (size_t k = 0; k < space_dimension_; ++k)
      sample_space[i * space_dimension_ + k] = point[k];
  }
  return sample_space;
}