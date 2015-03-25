#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PqpTest

#include "pqp_environment.h"
#include "random_generator/naive_generator.h"
#include <vector>
#include <utility>
#include <memory>
#include <cmath>

#include <boost/test/unit_test.hpp>


BOOST_AUTO_TEST_CASE(constructor) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomPointGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test.txt",
    "obstacles_test.txt", generator.get());

  std::vector<double> q {M_PI_2, M_PI_2};
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(query1) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomPointGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test.txt",
    "obstacles_test.txt", generator.get());

  std::vector<double> q {M_PI_2, 0};
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_CLOSE(test, 0.9989, 0.0001);
}

BOOST_AUTO_TEST_CASE(query2) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomPointGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test2.txt",
    "obstacles_test2.txt", generator.get());

  std::vector<double> q {M_PI_2, M_PI_2};
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(query3) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomPointGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test3.txt", "dh_table_test3.txt",
    "obstacles_test3.txt", generator.get());

  std::vector<double> q {M_PI_2, M_PI_2, M_PI_2};
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}