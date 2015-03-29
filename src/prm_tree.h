#ifndef PRM_TREE_H_INCLUDED
#define PRM_TREE_H_INCLUDED

#include <vector>
#include <memory>

#include <Eigen/Dense>
#include "pqp_environment.h"

class PrmTree {
public:
  typedef Eigen::VectorXd EVectorXd;
  PrmTree(PqpEnvironment* pqp_environment):
    pqp_environment_(pqp_environment),
    visited_(std::vector<bool>(pqp_environment->sample_space_size(), false)) {}

  virtual bool TryConnect(EVectorXd& point1, EVectorXd& point2) = 0;
  virtual bool AddPoint(int point_index) = 0;

protected:
  std::unique_ptr<PqpEnvironment> pqp_environment_;
  std::vector<bool> visited_;
};

#endif // PRM_TREE_H_INCLUDED