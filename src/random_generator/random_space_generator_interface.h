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
#ifndef RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED
#define RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED

#include <vector>
#include <memory>

class RandomSpaceGeneratorInterface {
 public:
  virtual ~RandomSpaceGeneratorInterface() {}
  virtual std::vector<double> CreatePoint() = 0;
  virtual std::unique_ptr<double[]> CreateSampleSpace(size_t num_points) = 0;

 protected:
  RandomSpaceGeneratorInterface() {}
};

#endif  // RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED
