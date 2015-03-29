#ifndef NAIVE_GENERATOR_H_INCLUDED
#define NAIVE_GENERATOR_H_INCLUDED

#include <random>
#include <vector>
#include <utility>
#include <memory>

#include "random_space_generator_interface.h"

class NaiveGenerator : public RandomSpaceGeneratorInterface {
public:
  NaiveGenerator(const std::vector<std::pair<double, double> >& limits);
  std::vector<double> CreatePoint();
  // Sample space is kept as single array of points
  // Array is of [num_points * dimension x 1] dimension
  std::unique_ptr<double[]> CreateSampleSpace(size_t num_points);

private:
  std::vector<std::uniform_real_distribution<double>> distributions_;
  std::mt19937 random_engine_;
  size_t space_dimension_;
};

#endif // NAIVE_GENERATOR_H_INCLUDED