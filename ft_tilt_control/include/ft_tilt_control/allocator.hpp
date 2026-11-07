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

// Solves [Mx, My, Mz, Fz]ᵀ = A·f with lower ≤ f ≤ upper. A motor with upper ≤ 0 has failed:
// its column is zeroed (Ã_i) and f_i = 0. Without the null space: pseudo-inverse + clamp.
// With it: f = Ã†·v + N·z (RPIC17 eq. (5)), z = 0 if within bounds, else the minimum-norm f
// within bounds, else the least-violating z, clamped.
// lower is the running floor (ICRIS18 §V). Keep it small: k_f·14 % ≈ 1.1 N leaves no hover after a failure.
Allocation allocate(
  const Matrix46d & a, const Eigen::Vector4d & wrench,
  const Vector6d & lower, const Vector6d & upper, bool use_null_space);

}  // namespace ft_tilt
