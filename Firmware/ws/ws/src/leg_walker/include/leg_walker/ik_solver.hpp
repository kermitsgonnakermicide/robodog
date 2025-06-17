#pragma once
#include <Eigen/Dense>

class IKSolver {
public:
  IKSolver(double L1, double L2);
  bool solve(const Eigen::Vector3d &foot_pos, Eigen::Vector3d &joint_angles);

private:
  double L1_, L2_;
};
