#include "naive_generator.h"

NaiveGenerator::NaiveGenerator(
    const std::vector<std::pair<double, double> >& limits) :
    space_dimension_ (limits.size()) {
  for(auto& limit : limits) {
    distributions_.emplace_back(limit.first, limit.second);
  }
  std::random_device rd;
  random_engine_.seed(rd());
}

std::vector<double> NaiveGenerator::CreatePoint() {
  std::vector<double> point;
  for (auto& distribution : distributions_)
    point.push_back(distribution(random_engine_));
  return point;
}

std::unique_ptr<double[]> NaiveGenerator::CreateSampleSpace(size_t num_points) {
  std::unique_ptr<double[]> sample_space (
    new double[num_points * space_dimension_]);
  for (size_t i (0); i < num_points; ++i) {
    for (size_t k (0); k < distributions_.size(); ++k)
      sample_space[i * space_dimension_ + k] =
        distributions_.at(k)(random_engine_);
  }
  return sample_space;
}
