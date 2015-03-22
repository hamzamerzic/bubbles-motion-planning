#ifndef NAIVE_GENERATOR_H_INCLUDED
#define NAIVE_GENERATOR_H_INCLUDED

#include <random>
#include <vector>
#include <utility>

#include "random_point_generator_interface.h"

class NaiveGenerator : public RandomPointGeneratorInterface {
public:
  NaiveGenerator(const std::vector<std::pair<double, double> >& limits);
  std::vector<double> NextPoint();

private:
  std::vector<std::uniform_real_distribution<>> distributions_;
  std::random_device random_device_;
};

#endif // NAIVE_GENERATOR_H_INCLUDED