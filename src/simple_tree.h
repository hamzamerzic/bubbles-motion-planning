#ifndef SIMPLE_TREE_H_INCLUDED
#define SIMPLE_TREE_H_INCLUDED

#include <vector>
#include <queue>
#include <Eigen/Dense>

#include "prm_tree.h"
#include "pqp_environment.h"

struct Edge {
  Edge(size_t point_index, double distance): point_index(point_index),
                                             distance(distance) {}
  size_t point_index;
  double distance;
};

struct EdgeCompareFunctor {
  bool operator() (const Edge& e1, const Edge& e2) const {
    return e1.distance > e2.distance;
  }
};

class SimpleTree : PrmTree {
public:
  SimpleTree(PqpEnvironment* pqp_environment, EVectorXd& start, EVectorXd& end,
      double step_size,double collision_limit = 0.01):
      PrmTree(pqp_environment, start, end),
      parents_(std::vector<int>(space_size_, -1)),
      step_size_(step_size), collision_limit_(collision_limit) {}

  virtual bool TryConnect(EVectorXd& point1, EVectorXd& point2);
  virtual bool AddPoint(int point_index);
  virtual bool BuildTree();

private:
  std::vector<int> parents_;
  double step_size_;
  double collision_limit_;
  std::priority_queue<Edge, std::vector<Edge>, EdgeCompareFunctor> pq_;
};

#endif // SIMPLE_TREE_H_INCLUDED