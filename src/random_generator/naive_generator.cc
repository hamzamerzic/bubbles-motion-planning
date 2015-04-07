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
#include "naive_generator.h"

NaiveGenerator::NaiveGenerator(
    const std::vector<std::pair<double, double>>& limits):
    space_dimension_ (limits.size()) {
  for(auto& limit : limits)
    distributions_.emplace_back(limit.first, limit.second);

  std::random_device rd;
  random_engine_.seed(rd());
}

std::vector<double> NaiveGenerator::CreatePoint() {
  std::vector<double> point;
  for (auto& distribution : distributions_)
    point.push_back(distribution(random_engine_));
  return point;
}

std::unique_ptr<double[]> NaiveGenerator::CreateSampleSpace(size_t num_points) {
  std::unique_ptr<double[]> sample_space (
    new double[num_points * space_dimension_]);
  for (size_t i (0); i < num_points; ++i) {
    for (size_t k (0); k < distributions_.size(); ++k)
      sample_space[i * space_dimension_ + k] =
        distributions_.at(k)(random_engine_);
  }
  return sample_space;
}
