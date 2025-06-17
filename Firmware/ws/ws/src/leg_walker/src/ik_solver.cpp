#include "leg_walker/ik_solver.hpp"
#include <cmath>

IKSolver::IKSolver(double L1, double L2) : L1_(L1), L2_(L2) {}

bool IKSolver::solve(const Eigen::Vector3d &p, Eigen::Vector3d &angles) {
  double rot = atan2(p.y(), p.x());
  double r = hypot(p.x(), p.y());
  double x = r;
  double z = p.z();

  double D = (x*x + z*z - L1_*L1_ - L2_*L2_) / (2 * L1_ * L2_);
  if (fabs(D) > 1.0) return false;

  double th2 = atan2(-sqrt(1 - D*D), D);
  double th1 = atan2(z, x) - atan2(L2_*sin(th2), L1_ + L2_*cos(th2));

  angles[0] = rot;
  angles[1] = th1;
  angles[2] = th2;
  return true;
}
