#ifndef PRM_TREE_H_INCLUDED
#define PRM_TREE_H_INCLUDED

#include <vector>
#include <memory>

#include <Eigen/Dense>
#include "pqp_environment.h"

class PrmTree {
public:
  typedef Eigen::VectorXd EVectorXd;
  PrmTree(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end):
    pqp_environment_(pqp_environment), start_(start), end_(end),
    space_size_(pqp_environment->sample_space_size() + 2),
    visited_(std::vector<bool>(space_size_, false)) {}

  virtual bool TryConnect(EVectorXd& point1, EVectorXd& point2) = 0;
  virtual bool AddPoint(int point_index) = 0;
  virtual bool BuildTree() = 0;

protected:
  std::unique_ptr<PqpEnvironment> pqp_environment_;
  EVectorXd start_, end_;
  int space_size_;
  std::vector<bool> visited_;

};

#endif // PRM_TREE_H_INCLUDED