#include "simple_tree.h"

bool SimpleTree::TryConnect (EVectorXd& point1, EVectorXd& point2) {
  EVectorXd point_step = (point2 - point1).normalized() * step_size_;
  EVectorXd point_iterator (point1);

  int step_number (static_cast<int>((point2 - point1).norm() / step_size_));
  int steps (0);
  while (steps <= step_number) {
    if (pqp_environment_->CheckCollision(point_iterator) < collision_limit_)
      return false;
    point_iterator += point_step;
    ++steps;
  }

  return pqp_environment_->CheckCollision(point2) >= collision_limit_;
}

bool SimpleTree::AddPoint(int point_index) {
  return true;
}