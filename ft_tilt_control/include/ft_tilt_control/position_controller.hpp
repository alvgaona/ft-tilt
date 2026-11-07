#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace ft_tilt
{

// Outer loop standing in for the papers' RC pilot (they flew position and altitude by hand,
// so none of this is in the papers; gains derived in the sim).
struct PositionGains
{
  double kp_xy = 0.2, ki_xy = 0.02, kd_xy = 0.6;  // horizontal, (m/s²) per m, m·s, m/s
  double kp_z = 3.0, ki_z = 0.5, kd_z = 3.0;      // vertical, same units
  double max_tilt = 10.0 * M_PI / 180.0;          // rad
  double max_thrust = 30.0;                        // N
  double mass = 2.1;                               // kg, RPIC17 Table I
};

struct AttitudeThrust
{
  double roll, pitch, thrust;  // rad (FLU, yaw frame), N along body z
};

// Position PID → acceleration → roll/pitch (small angles) and thrust along body z.
// World frame FLU, z up. Pitching back needs −My, which is weak with motor 2 off, hence the
// soft horizontal gains. The horizontal integral cancels the sideways force tilted rotors make
// after a fault (≈ 0.2 m/s², derived).
class PositionController
{
public:
  explicit PositionController(const PositionGains & gains = {}) : g_(gains) {}

  void reset() {integral_.setZero();}

  // p, v: world position and velocity. v_ref is a velocity feedforward (0 to hold p_ref).
  // cos_tilt = cos(roll)·cos(pitch), the share of body-z thrust that points up.
  AttitudeThrust update(
    const Eigen::Vector3d & p_ref, const Eigen::Vector3d & v_ref, const Eigen::Vector3d & p,
    const Eigen::Vector3d & v, double yaw, double cos_tilt, double dt)
  {
    constexpr double g = 9.80665;
    // Error capped at 2 m so a far-off vehicle flies back at ≈ kp·2/kd instead of overshooting.
    Eigen::Vector3d e = p_ref - p;
    e.x() = std::clamp(e.x(), -2.0, 2.0);
    e.y() = std::clamp(e.y(), -2.0, 2.0);
    const double lim = 0.5 / std::max(g_.ki_xy, 1e-9);  // integral term ≤ 0.5 m/s² (anti-windup)
    integral_ += e * dt;
    integral_.x() = std::clamp(integral_.x(), -lim, lim);
    integral_.y() = std::clamp(integral_.y(), -lim, lim);

    const Eigen::Vector3d dv = v_ref - v;
    const double ax = g_.kp_xy * e.x() + g_.ki_xy * integral_.x() + g_.kd_xy * dv.x();
    const double ay = g_.kp_xy * e.y() + g_.ki_xy * integral_.y() + g_.kd_xy * dv.y();
    const double az = g_.kp_z * e.z() + g_.ki_z * integral_.z() + g_.kd_z * dv.z();

    const double fwd = std::cos(yaw) * ax + std::sin(yaw) * ay;
    const double left = -std::sin(yaw) * ax + std::cos(yaw) * ay;
    return {
      std::clamp(-std::atan2(left, g), -g_.max_tilt, g_.max_tilt),  // +roll → −y
      std::clamp(std::atan2(fwd, g), -g_.max_tilt, g_.max_tilt),    // +pitch → +x
      std::clamp(g_.mass * (g + az) / std::max(cos_tilt, 0.5), 0.0, g_.max_thrust)};
  }

  PositionGains & gains() {return g_;}

private:
  PositionGains g_;
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
};

}  // namespace ft_tilt
