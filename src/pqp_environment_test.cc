#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PqpTest

#include "pqp_environment.h"
#include "random_generator/naive_generator.h"

#include <vector>
#include <utility>
#include <memory>
#include <cmath>

#include <boost/test/unit_test.hpp>

typedef Eigen::VectorXd EVectorXd;

BOOST_AUTO_TEST_CASE(constructor) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test.txt",
    "obstacles_test.stl", generator.get());

  EVectorXd q (2); q << M_PI_2, M_PI_2;
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(collision_query1) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test.txt",
    "obstacles_test.stl", generator.get());

  EVectorXd q (2); q << M_PI_2, 0;
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_CLOSE(test, 0.9989, 0.0001);
}

BOOST_AUTO_TEST_CASE(collision_query2) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test2.txt",
    "obstacles_test2.stl", generator.get());

  EVectorXd q (2); q << M_PI_2, M_PI_2;
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(collision_query3) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test3.txt", "dh_table_test3.txt",
    "obstacles_test3.stl", generator.get());

  EVectorXd q (3); q << M_PI_2, M_PI_2, M_PI_2;
  double test = pqp.CheckCollision(q);
  BOOST_CHECK_SMALL(test, 0.0001);
}

BOOST_AUTO_TEST_CASE(knn_query) {
  std::vector<std::pair<double, double>> limits;
  limits.emplace_back(0, 2 * M_PI);
  limits.emplace_back(0, 2 * M_PI);
  std::unique_ptr<RandomSpaceGeneratorInterface> generator (
    new NaiveGenerator(limits));
  PqpEnvironment pqp ("robot_test.txt", "dh_table_test2.txt",
    "obstacles_test2.stl", generator.get(), 1000);

  EVectorXd q (2); q << M_PI_2, M_PI_2;
  std::vector<int> indices (pqp.KnnQuery(q, 5));
  for (size_t i = 0; i < indices.size(); ++i)
  {
    printf("%d\n", indices.at(i));
  }
}