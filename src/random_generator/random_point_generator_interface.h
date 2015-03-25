#ifndef RANDOM_POINT_GENERATOR_INTERFACE_H_INCLUDED
#define RANDOM_POINT_GENERATOR_INTERFACE_H_INCLUDED

#include <vector>

class RandomPointGeneratorInterface {
public:
  virtual ~RandomPointGeneratorInterface() {};
  virtual std::vector<double> NextPoint() = 0;

protected:
  RandomPointGeneratorInterface() {};
};

#endif  // RANDOM_POINT_GENERATOR_INTERFACE_H_INCLUDED