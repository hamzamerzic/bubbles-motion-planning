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
#include "bubble_prm.h"
#include "lazy_prm.h"

#include <vector>
#include <memory>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>

#include "environment/pqp_environment.h"
#include "random_generator/naive_generator.h"
#include "random_generator/halton_generator.h"

using ::bubbleprm::BubblePrm;
using ::lazyprm::LazyPrm;

typedef Eigen::VectorXd EVectorXd;

int main() {
  // Limits
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(-2.879793266, 2.879793266);
  limits.emplace_back(-1.919862177, 1.919862177);
  limits.emplace_back(-1.570796327, 1.221730476);
  limits.emplace_back(-2.792526803, 2.792526803);
  limits.emplace_back(-2.094395102, 2.094395102);
  limits.emplace_back(-6.981317008, 6.981317008);

  // Configurations trivial 1
  EVectorXd start_trivial1 (6); start_trivial1 << -0.7330382858,   // -42
                                                  -0.5235987756,   // -30
                                                  -0.03490658504,  // -2
                                                   0.5585053606,   // 30
                                                  -0.03490658504,  // -2
                                                   0.8203047484;   // 47
  EVectorXd end_trivial1 (6); end_trivial1 << 2.094395102,   // 120
                                              0.2792526803,  // 16
                                              0.2443460953,  // 14
                                              1.570796327,   // 90
                                             -0.2443460953,  // -14
                                             -0.5235987756;  // -30

  // Configurations trivial 2
  EVectorXd start_trivial2 (6); start_trivial2 << -0.698131700797732,   // -40
                                                   1.221730476396031,   //  70
                                                   -1.396263401595464,  // -80
                                                   0.261799387799149,   //  15
                                                   -0.314159265358979,  // -18
                                                   -5.235987755982989;  // -300
  EVectorXd end_trivial2 (6); end_trivial2 <<  0.610865238198015,   //  35
                                               1.047197551196598,   //  60
                                               -0.523598775598299,  // -30
                                               -1.047197551196598,  // -60
                                               -0.872664625997165,  // -50
                                               3.490658503988659;   //  200

  // Configurations easy 1
  EVectorXd start_easy1 (6); start_easy1 << 0.959931089,  // 55
                                            1.221730476,  // 70
                                           -0.907571211,  // -52
                                            0.0,          // 0
                                           -0.523598776,  // -30
                                           -0.523598776;  // -30
  EVectorXd end_easy1 (6); end_easy1 << -0.785398163,  // -45
                                         0.41887902,   // 24
                                        -0.34906585,   // -20
                                        -1.570796327,  // -90
                                         0.20943951,   // 12
                                        -2.35619449;   // -135

  // Configurations easy 2
  EVectorXd start_easy2 (6); start_easy2 << 0.698131700797732,   //  40
                                            1.047197551196598,   //  60
                                            -0.523598775598299,  // -30
                                            -1.047197551196598,  // -60
                                            -0.872664625997165,  // -50
                                            3.490658503988659;   //  200
  EVectorXd end_easy2 (6); end_easy2 << -1.5707963267948966,   // -90
                                         0.0872664625997165,   //  5
                                         -1.3962634015954636,  // -80
                                         1.5707963267948966,   //  90
                                         -0.0872664625997165,  // -5
                                         -4.3633231299858233;  // -250

  // Configuration hard 1
  EVectorXd start_hard1 (6); start_hard1 << -1.570796327,  // -90
                                             1.134464014,  // 65
                                            -1.308996939,  //-75
                                             0.523598776,  // 30
                                             0.209439510,  // 12
                                             0.0;          // 0
  EVectorXd end_hard1 (6); end_hard1 << 0.174532925,   // 10
                                        0.872664626,   // 50
                                       -1.134464014,   // -65
                                        0.959931089,   // 55
                                        0.7330382858,  // 42
                                        3.839724354;   // 220

  // Configuration hard 2
  EVectorXd start_hard2 (6); start_hard2 << -1.570796327,  // -90
                                             1.134464014,  // 65
                                            -1.308996939,  //-75
                                             0.523598776,  // 30
                                             0.209439510,  // 12
                                             0.0;          // 0
  EVectorXd end_hard2 (6); end_hard2 << 0.174532925,   // 10
                                        0.872664626,   // 50
                                       -1.134464014,   // -65
                                        0.959931089,   // 55
                                        0.7330382858,  // 42
                                        3.839724354;   // 220

  // TEST BUBBLE TRIVIAL 1
  double timing = 0.0;
  std::string logtimings;
  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1000));
    BubblePrm bubble_prm (pqp.release(), start_trivial1, end_trivial1, 10);

    std::string logname ("logs/bubble_trivial1/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE TRIVIAL 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1000));
    BubblePrm bubble_prm (pqp.release(), start_trivial1, end_trivial1, 10);

    std::string logname ("logs/bubble_trivial1h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE TRIVIAL 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1200));
    BubblePrm bubble_prm (pqp.release(), start_trivial2, end_trivial2, 15);

    std::string logname ("logs/bubble_trivial2/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE TRIVIAL 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1200));
    BubblePrm bubble_prm (pqp.release(), start_trivial2, end_trivial2, 15);

    std::string logname ("logs/bubble_trivial2h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE EASY 1
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2000));
    BubblePrm bubble_prm (pqp.release(), start_easy1, end_easy1, 20);

    std::string logname ("logs/bubble_easy1/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE EASY 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2000));
    BubblePrm bubble_prm (pqp.release(), start_easy1, end_easy1, 20);

    std::string logname ("logs/bubble_easy1h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE EASY 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2200));
    BubblePrm bubble_prm (pqp.release(), start_easy2, end_easy2, 25);

    std::string logname ("logs/bubble_easy2/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE EASY 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2200));
    BubblePrm bubble_prm (pqp.release(), start_easy2, end_easy2, 25);

    std::string logname ("logs/bubble_easy2h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE HARD 1
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    BubblePrm bubble_prm (pqp.release(), start_hard1, end_hard1, 60);

    std::string logname ("logs/bubble_hard1/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE HARD 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    BubblePrm bubble_prm (pqp.release(), start_hard1, end_hard1, 60);

    std::string logname ("logs/bubble_hard1h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE HARD 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    BubblePrm bubble_prm (pqp.release(), start_hard2, end_hard2, 60);

    std::string logname ("logs/bubble_hard2/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST BUBBLE HARD 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    BubblePrm bubble_prm (pqp.release(), start_hard2, end_hard2, 60);

    std::string logname ("logs/bubble_hard2h/bubble" + std::to_string(i));
    if(bubble_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY TRIVIAL 1
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1000));
    LazyPrm lazy_prm (pqp.release(), start_trivial1, end_trivial1, 10);

    std::string logname ("logs/lazy_trivial1/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY TRIVIAL 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1000));
    LazyPrm lazy_prm (pqp.release(), start_trivial1, end_trivial1, 10);

    std::string logname ("logs/lazy_trivial1h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY TRIVIAL 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1200));
    LazyPrm lazy_prm (pqp.release(), start_trivial2, end_trivial2, 15);

    std::string logname ("logs/lazy_trivial2/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY TRIVIAL 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_trivial.stl",
        generator.release(), 1200));
    LazyPrm lazy_prm (pqp.release(), start_trivial2, end_trivial2, 15);

    std::string logname ("logs/lazy_trivial2h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY EASY 1
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2000));
    LazyPrm lazy_prm (pqp.release(), start_easy1, end_easy1, 20);

    std::string logname ("logs/lazy_easy1/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY EASY 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2000));
    LazyPrm lazy_prm (pqp.release(), start_easy1, end_easy1, 20);

    std::string logname ("logs/lazy_easy1h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY EASY 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2200));
    LazyPrm lazy_prm (pqp.release(), start_easy2, end_easy2, 25);

    std::string logname ("logs/lazy_easy2/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY EASY 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_easy.stl",
        generator.release(), 2200));
    LazyPrm lazy_prm (pqp.release(), start_easy2, end_easy2, 25);

    std::string logname ("logs/lazy_easy2h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY HARD 1
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    LazyPrm lazy_prm (pqp.release(), start_hard1, end_hard1, 60);

    std::string logname ("logs/lazy_hard1/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY HARD 1H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    LazyPrm lazy_prm (pqp.release(), start_hard1, end_hard1, 60);

    std::string logname ("logs/lazy_hard1h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY HARD 2
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new NaiveGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    LazyPrm lazy_prm (pqp.release(), start_hard2, end_hard2, 60);

    std::string logname ("logs/lazy_hard2/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  // TEST LAZY HARD 2H
  timing = 0.0;

  for (unsigned i = 0; i < 100; ++i) {
    auto start_t = std::chrono::steady_clock::now();
    std::unique_ptr<RandomSpaceGeneratorInterface> generator (
      new HaltonGenerator(limits));
    std::unique_ptr<PqpEnvironment> pqp (new PqpEnvironment(
        {"models/abb-irb-120/1link.stl", "models/abb-irb-120/2link.stl",
        "models/abb-irb-120/3link1.stl", "models/abb-irb-120/4link1.stl",
        "models/abb-irb-120/5link.stl", "models/abb-irb-120/6link.stl"},
        "models/abb-irb-120/parameters.txt",
        "models/environment/obstacles_hard.stl",
        generator.release(), 4000));
    LazyPrm lazy_prm (pqp.release(), start_hard2, end_hard2, 60);

    std::string logname ("logs/lazy_hard2h/bubble" + std::to_string(i));
    if(lazy_prm.BuildTree(logname)) {
      auto end_t = std::chrono::steady_clock::now();
      auto duration = end_t - start_t;
      timing += std::chrono::duration <double, std::milli> (duration).count();
    }
    else
      --i;
  }
  logtimings += std::to_string(timing) + '\n';

  std::ofstream logfile ("logs/logtimings");
  logfile << logtimings;
  return 0;
}