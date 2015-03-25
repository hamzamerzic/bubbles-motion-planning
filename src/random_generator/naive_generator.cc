#include "naive_generator.h"

NaiveGenerator::NaiveGenerator(
    const std::vector<std::pair<double, double> >& limits) {
  for(auto& limit : limits) {
    distributions_.emplace_back(limit.first, limit.second);
  }
}

std::vector<double> NaiveGenerator::NextPoint() {
  std::vector<double> point;
  for (auto& distribution : distributions_)
    point.push_back(distribution(random_device_));
  return point;
}