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
#ifndef HALTON_GENERATOR_H_INCLUDED
#define HALTON_GENERATOR_H_INCLUDED

#include <random>
#include <vector>
#include <utility>
#include <memory>

#include "random_space_generator_interface.h"

class HaltonGenerator : public RandomSpaceGeneratorInterface {
 public:
  explicit HaltonGenerator(
     const std::vector<std::pair<double, double>>& limits);
  std::vector<double> CreatePoint();
  // Sample space is kept as single array of points
  // Array is of [num_points * dimension x 1] dimension
  std::unique_ptr<double[]> CreateSampleSpace(size_t num_points);

 private:
  std::vector<std::pair<double, double>> limits_;
  size_t space_dimension_, halton_index_;
  std::vector<int> primes_;
};

#endif  // HALTON_GENERATOR_H_INCLUDED
