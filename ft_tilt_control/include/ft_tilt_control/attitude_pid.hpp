#pragma once

#include <Eigen/Dense>

namespace ft_tilt
{

// Per-axis gains for roll, pitch, yaw. Output is torque (N·m).
struct PidGains
{
  Eigen::Vector3d kp;  // N·m/rad
  Eigen::Vector3d ki;  // N·m/(rad·s)
  Eigen::Vector3d kd;  // N·m/(rad/s)
};

// ICRIS18 §IV gain sets. Roll/pitch use "aggressive" or "mild"; yaw always uses the yaw set.
inline PidGains aggressive_gains()
{
  return {{5.0, 5.0, 0.8}, {6.0, 6.0, 0.2}, {0.9, 0.9, 0.6}};
}

inline PidGains mild_gains()
{
  return {{1.4, 1.4, 0.8}, {1.0, 1.0, 0.2}, {0.9, 0.9, 0.6}};
}

// One PID per Euler angle (ICRIS18 "loop-at-a-time"). The papers don't say how the
// derivative is taken; here it acts on the measured body rate, so reference steps don't kick.
class AttitudePid
{
public:
  void set_gains(const PidGains & gains) {gains_ = gains;}
  void reset() {integral_.setZero();}

  // rpy_ref, rpy: ZYX Euler angles (rad). body_rates: p, q, r (rad/s). Returns [Mx, My, Mz].
  Eigen::Vector3d update(
    const Eigen::Vector3d & rpy_ref, const Eigen::Vector3d & rpy,
    const Eigen::Vector3d & body_rates, double dt);

private:
  PidGains gains_ = aggressive_gains();
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();  // ∫ki·e, so gain switches are bumpless
};

}  // namespace ft_tilt
