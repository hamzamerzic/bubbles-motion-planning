#ifndef BUBBLE_H_INCLUDED
#define BUBBLE_H_INCLUDED

#include <vector>
#include <cmath>

class Bubble {
public:
  Bubble(): distance_(INFINITY) {}

  double& distance() { return distance_; }
  const double Get(size_t i) const { return coordinates_.at(i); }
  void Set(size_t i, double value) { coordinates_.at(i) = value; }
  void Resize(size_t n) { coordinates_.resize(n); }

private:
  std::vector<double> coordinates_;
  double distance_;
};

#endif // BUBBLE_H_INCLUDED