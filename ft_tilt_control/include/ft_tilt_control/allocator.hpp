#pragma once

#include <Eigen/Dense>

#include "ft_tilt_control/model.hpp"

namespace ft_tilt
{

struct Allocation
{
  Vector6d force;  // N, failed motors are exactly 0
  double margin;   // smallest distance from a working motor to its bounds (N); < 0 means clamped
};

// Solves [Mx, My, Mz, Fz]ᵀ = A·f with lower ≤ f ≤ upper.
// A motor with upper ≤ 0 has failed: its column is zeroed (the paper's Ã_i) and f_i = 0.
//
// use_null_space = false: Moore-Penrose solution, clamped to the bounds (nominal flight).
// use_null_space = true:  every solution f = Ã†·v + N·z (RPIC17 eq. (5), ICRIS18 eq. (4)).
//   z = 0 (minimum energy) when that is within bounds, else the smallest z that is, i.e. the
//   minimum-norm f within the bounds. If none is, the least-violating z, then clamped.
//
// lower is the running floor (motors must never stop, ICRIS18 §V). It has to be small: the
// paper's linear k_f·14 % ≈ 1.1 N floor makes hovering after a failure impossible (see tests).
Allocation allocate(
  const Matrix46d & a, const Eigen::Vector4d & wrench,
  const Vector6d & lower, const Vector6d & upper, bool use_null_space);

}  // namespace ft_tilt
