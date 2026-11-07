#pragma once

#include <algorithm>
#include <cmath>

namespace ft_tilt
{

// PWM (%) ↔ thrust (N), thrust ~ PWM² through F_M at u_max = F_M / k_f (derived, not in the
// papers). It gives RPIC17's measured ≈ 60 % hover PWM, where the linear f = k_f·u gives 46 %.
inline double pwm_to_force(double pwm, double kf, double max_force)
{
  const double r = pwm * kf / max_force;
  return max_force * r * r;
}

inline double force_to_pwm(double force, double kf, double max_force)
{
  return std::sqrt(std::max(force, 0.0) * max_force) / kf;
}

}  // namespace ft_tilt
