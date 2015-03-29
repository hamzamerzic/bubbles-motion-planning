#ifndef RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED
#define RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED

#include <vector>
#include <memory>

class RandomSpaceGeneratorInterface {
public:
  virtual ~RandomSpaceGeneratorInterface() {};
  virtual std::vector<double> CreatePoint() = 0;
  virtual std::unique_ptr<double[]> CreateSampleSpace(size_t num_points) = 0;

protected:
  RandomSpaceGeneratorInterface() {};
};

#endif  // RANDOM_SPACE_GENERATOR_INTERFACE_H_INCLUDED