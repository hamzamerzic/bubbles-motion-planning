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
#define BOOST_TEST_MODULE BubblePrmTest

#include "bubble_prm.h"

#include <vector>
#include <memory>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>

#include "environment/pqp_environment.h"
#include "random_generator/naive_generator.h"
#include "random_generator/halton_generator.h"
#include <boost/test/unit_test.hpp>

using namespace bubbleprm;

typedef Eigen::VectorXd EVectorXd;

BOOST_AUTO_TEST_CASE(hull) {
  EVectorXd b2_coordinates (2), b1_coordinates (2), b1_dimensions (2),
    result (2);
  b2_coordinates << 2, 0;
  b1_coordinates << 0, 0;
  b1_dimensions << 1.5, 1;
  result << 1.5, 0;
  EVectorXd direction (b2_coordinates - b1_coordinates);
  direction = direction / fabs((direction.cwiseQuotient(b1_dimensions)).sum());
  BOOST_CHECK_EQUAL(direction == result, true);
}

BOOST_AUTO_TEST_CASE(hull1) {
  EVectorXd b2_coordinates (2), b1_coordinates (2), b1_dimensions (2),
    result (2);
  b2_coordinates << 0, 2;
  b1_coordinates << 0, 0;
  b1_dimensions << 1.5, 1;
  result << 0, 1;
  EVectorXd direction (b2_coordinates - b1_coordinates);
  direction = direction / fabs((direction.cwiseQuotient(b1_dimensions)).sum());
  BOOST_CHECK_EQUAL(direction == result, true);
}

BOOST_AUTO_TEST_CASE(hull2) {
  EVectorXd b2_coordinates (2), b1_coordinates (2), b1_dimensions (2),
    result (2);
  b2_coordinates << 3, 3;
  b1_coordinates << 0, 0;
  b1_dimensions << 2, 1;
  result << 0.66667, 0.66667;
  EVectorXd direction (b2_coordinates - b1_coordinates);
  direction = direction / fabs((direction.cwiseQuotient(b1_dimensions)).sum());
  BOOST_CHECK_SMALL((direction - result).norm(), 0.01);
}

BOOST_AUTO_TEST_CASE(build) {
  auto start_t = std::chrono::steady_clock::now();
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

  BubblePrm bubble_prm (pqp.release(), start, end, 10);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_trivial"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_trivial.py");
}

BOOST_AUTO_TEST_CASE(buildh) {
  auto start_t = std::chrono::steady_clock::now();
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new HaltonGenerator(limits));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 10);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_trivialh"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_trivialh.py");
}

BOOST_AUTO_TEST_CASE(build1) {
  auto start_t = std::chrono::steady_clock::now();
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

  BubblePrm bubble_prm (pqp.release(), start, end, 20);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_easy"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_easy.py");
}

BOOST_AUTO_TEST_CASE(build1h) {
  auto start_t = std::chrono::steady_clock::now();
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new HaltonGenerator(limits));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 20);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_easyh"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_easyh.py");
}

BOOST_AUTO_TEST_CASE(build2) {
  auto start_t = std::chrono::steady_clock::now();
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
      generator.release(), 10000));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 100);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_hard"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_hard.py");
}

BOOST_AUTO_TEST_CASE(build2h) {
  auto start_t = std::chrono::steady_clock::now();
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new HaltonGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_hard.stl",
      generator.release(), 10000));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 100);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_hardh"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_hardh.py");
}

BOOST_AUTO_TEST_CASE(build3) {
  auto start_t = std::chrono::steady_clock::now();
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
      generator.release(), 6000));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 40);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_hard2"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_hard2.py");
}

BOOST_AUTO_TEST_CASE(build3h) {
  auto start_t = std::chrono::steady_clock::now();
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new HaltonGenerator(limits));
  std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
      {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
      "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
      "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
      "models/abb-irb-120/parameters.txt",
      "models/environment/obstacles_hard.stl",
      generator.release(), 6000));
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

  BubblePrm bubble_prm (pqp.release(), start, end, 40);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_hard2h"), true);
  auto end_t = std::chrono::steady_clock::now();
  auto duration = end_t - start_t;
  std::cout << "Elapsed time: " <<
    std::chrono::duration <double, std::milli> (duration).count() << " ms" <<
    std::endl;
  bubble_prm.GeneratePath("bubble_rdk_hard2h.py");
}

BOOST_AUTO_TEST_CASE(initialcol4) {
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

  BubblePrm bubble_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_col4"), false);
}

BOOST_AUTO_TEST_CASE(finalcol5) {
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

  BubblePrm bubble_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_col5"), false);
}

BOOST_AUTO_TEST_CASE(initialcol6) {
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

  BubblePrm bubble_prm (pqp.release(), start, end, 15);
  BOOST_CHECK_EQUAL(bubble_prm.BuildTree("bubble_col6"), false);
}
