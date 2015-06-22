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
#define BOOST_TEST_MODULE LazyPrmTest

#include "lazy_prm.h"

#include <vector>
#include <memory>
#include <cmath>
#include <iostream>

#include "environment/pqp_environment.h"
#include "random_generator/naive_generator.h"
#include <boost/test/unit_test.hpp>

typedef Eigen::VectorXd EVectorXd;

BOOST_AUTO_TEST_CASE(build) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_trivial.stl",
      generator.release(), 1000));
  EVectorXd start (6); start << -0.7330382858,   // -42
                                -0.5235987756,   // -30
                                -0.03490658504,  // -2
                                 0.5585053606,   // 30
                                -0.03490658504,  // -2
                                 0.8203047484;   // 47
  EVectorXd end (6); end << 2.094395102,   // 120
                            0.2792526803,  // 16
                            0.2443460953,  // 14
                            1.570796327,   // 90
                           -0.2443460953,  // -14
                           -0.5235987756;  // -30

  LazyPrm lazy_prm (pqp.release(), start, end, 20);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), true);
  lazy_prm.LogResults("lazy_rdk_trivial.py");
}


BOOST_AUTO_TEST_CASE(build1) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_easy.stl",
      generator.release(), 2000));
  EVectorXd start (6); start << 0.959931089,  // 55
                                1.221730476,  // 70
                               -0.907571211,  // -52
                                0.0,          // 0
                               -0.523598776,  // -30
                               -0.523598776;  // -30
  EVectorXd end (6); end << -0.785398163,  // -45
                             0.41887902,   // 24
                            -0.34906585,   // -20
                            -1.570796327,  // -90
                             0.20943951,   // 12
                            -2.35619449;   // -135

  LazyPrm lazy_prm (pqp.release(), start, end, 20);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), true);
  lazy_prm.LogResults("lazy_rdk_easy.py");
}

BOOST_AUTO_TEST_CASE(build2) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_hard.stl",
      generator.release(), 20000));
  EVectorXd start (6); start << -1.570796327,  // -90
                                 1.134464014,  // 65
                                -1.308996939,  //-75
                                 0.523598776,  // 30
                                 0.209439510,  // 12
                                 0.0;          // 0
  EVectorXd end (6); end << 0.174532925,   // 10
                            0.872664626,   // 50
                           -1.134464014,   // -65
                            0.959931089,   // 55
                            0.7330382858,  // 42
                            3.839724354;   // 220

  LazyPrm lazy_prm (pqp.release(), start, end, 100);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), true);
  lazy_prm.LogResults("lazy_rdk_hard.py");
}

BOOST_AUTO_TEST_CASE(build3) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_hard.stl",
      generator.release(), 20000));
  EVectorXd start (6); start << -1.570796327,  // -90
                                 1.134464014,  // 65
                                -1.308996939,  //-75
                                 0.523598776,  // 30
                                 0.209439510,  // 12
                                 0.0;          // 0
  EVectorXd end (6); end << -0.20943951,    // -12
                             0.610865238,   // 35
                            -0.087266463,   // -5
                             0.610865238,   // 35
                            -0.5235987756,  // -30
                            -4.36332313;    // -250

  LazyPrm lazy_prm (pqp.release(), start, end, 100);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), true);
  lazy_prm.LogResults("lazy_rdk_hard2.py");
}

BOOST_AUTO_TEST_CASE(build4) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_hard.stl",
      generator.release(), 5000));
  EVectorXd start (6); start << 0.959931089,
                                1.221730476,
                               -0.907571211,
                                0.0,
                               -0.523598776,
                               -0.523598776;
  EVectorXd end (6); end << -0.785398163,
                             0.41887902,
                            -0.34906585,
                            -1.570796327,
                             0.20943951,
                            -2.35619449;

  LazyPrm lazy_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), false);
}

BOOST_AUTO_TEST_CASE(build5) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_easy.stl",
      generator.release(), 5000));

  EVectorXd start (6); start << 0.959931089,
                                1.221730476,
                               -0.907571211,
                                0.0,
                               -0.523598776,
                               -0.523598776;
  EVectorXd end (6); end << 0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0;

  LazyPrm lazy_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), false);
}

BOOST_AUTO_TEST_CASE(build6) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_easy.stl",
      generator.release(), 5000));
  EVectorXd start (6); start << 0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0;
  EVectorXd end (6); end << -0.785398163,
                             0.41887902,
                            -0.34906585,
                            -1.570796327,
                             0.20943951,
                            -2.35619449;

  LazyPrm lazy_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(lazy_prm.BuildTree(), false);
}
