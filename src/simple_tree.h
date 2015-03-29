#ifndef SIMPLE_TREE_H_INCLUDED
#define SIMPLE_TREE_H_INCLUDED

#include <vector>
#include <Eigen/Dense>

#include "prm_tree.h"
#include "pqp_environment.h"

class SimpleTree : PrmTree {
public:
  SimpleTree(PqpEnvironment* pqp_environment, double step_size,
             double collision_limit = 0.01):
      PrmTree(pqp_environment), step_size_(step_size),
      collision_limit_(collision_limit) {}

  virtual bool TryConnect(EVectorXd& point1, EVectorXd& point2);
  virtual bool AddPoint(int point_index);

private:
  std::vector<int> parents_;
  double step_size_;
  double collision_limit_;
};

#endif // SIMPLE_TREE_H_INCLUDED