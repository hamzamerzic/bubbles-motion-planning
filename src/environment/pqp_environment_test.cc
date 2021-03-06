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
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PqpTest

#include "pqp_environment.h"

#include <vector>
#include <utility>
#include <memory>
#include <cmath>

#include "random_generator/naive_generator.h"
#include <boost/test/unit_test.hpp>

typedef Eigen::VectorXd EVectorXd;

BOOST_AUTO_TEST_CASE(constructor) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ({"../models/robot1_seg1.stl",
      "../models/robot1_seg2.stl"}, "../models/dh_table_test.txt",
      "../models/obstacles_test.stl", generator.get());
}

BOOST_AUTO_TEST_CASE(distance_query1) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ({"../models/robot1_seg1.stl",
      "../models/robot1_seg2.stl"}, "../models/dh_table_test.txt",
      "../models/obstacles_test.stl", generator.get());

  EVectorXd q (2); q << M_PI_2, 0;
  double test = pqp.DistanceQuery(q);
  BOOST_CHECK_CLOSE(test, 0.9989, 0.0001);
}

BOOST_AUTO_TEST_CASE(distance_query2) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ({"../models/robot1_seg1.stl",
      "../models/robot1_seg2.stl"}, "../models/dh_table_test2.txt",
      "../models/obstacles_test2.stl", generator.get());


  EVectorXd q (2); q << M_PI_2, M_PI_2;
  double test = pqp.DistanceQuery(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(distance_query3) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ({"../models/robot2_seg1.stl", "../models/robot2_seg2.stl",
      "../models/robot2_seg3.stl"}, "../models/dh_table_test3.txt",
      "../models/obstacles_test3.stl", generator.get());

  EVectorXd q (3); q << M_PI_2, M_PI_2, M_PI_2;
  double test = pqp.DistanceQuery(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(knn_query) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ({"../models/robot1_seg1.stl",
      "../models/robot1_seg2.stl"}, "../models/dh_table_test2.txt",
      "../models/obstacles_test2.stl", generator.get(), 1000);

  EVectorXd q (2); q << M_PI_2, M_PI_2;
  std::vector<int> indices (pqp.KnnQuery(q, 5));
  for (size_t i = 0; i < indices.size(); ++i)
  {
    printf("%d\n", indices.at(i));
  }
}