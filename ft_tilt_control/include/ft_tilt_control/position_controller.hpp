#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace ft_tilt
{

// Stands in for the papers' RC pilot (not in the papers, gains tuned in the sim).
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

// World frame FLU, z up. Soft horizontal gains because −My (pitching back) is weak with motor 2
// off. The integral cancels the sideways force tilted rotors make after a fault (≈ 0.2 m/s², derived).
class PositionController
{
public:
  explicit PositionController(const PositionGains & gains = {}) : gains_(gains) {}

  void reset() {integral_.setZero();}

  // p, v: world position and velocity. v_ref is a velocity feedforward (0 to hold p_ref).
  // cos_tilt = cos(roll)·cos(pitch), the share of body-z thrust that points up.
  AttitudeThrust update(
    const Eigen::Vector3d & p_ref, const Eigen::Vector3d & v_ref, const Eigen::Vector3d & p,
    const Eigen::Vector3d & v, double yaw, double cos_tilt, double dt)
  {
    constexpr double g = 9.80665;
    // Capped so a far-off vehicle flies back at ≈ kp·2/kd instead of overshooting.
    Eigen::Vector3d e = p_ref - p;
    e.x() = std::clamp(e.x(), -2.0, 2.0);
    e.y() = std::clamp(e.y(), -2.0, 2.0);
    // Anti-windup: integral term ≤ 0.5 m/s² sideways, ≤ 1 m/s² (~10 % of weight) vertically.
    const double lim = 0.5 / std::max(gains_.ki_xy, 1e-9), lim_z = 1.0 / std::max(gains_.ki_z, 1e-9);
    integral_ += e * dt;
    integral_.x() = std::clamp(integral_.x(), -lim, lim);
    integral_.y() = std::clamp(integral_.y(), -lim, lim);
    integral_.z() = std::clamp(integral_.z(), -lim_z, lim_z);

    const Eigen::Vector3d dv = v_ref - v;
    const double ax = gains_.kp_xy * e.x() + gains_.ki_xy * integral_.x() + gains_.kd_xy * dv.x();
    const double ay = gains_.kp_xy * e.y() + gains_.ki_xy * integral_.y() + gains_.kd_xy * dv.y();
    const double az = gains_.kp_z * e.z() + gains_.ki_z * integral_.z() + gains_.kd_z * dv.z();

    const double fwd = std::cos(yaw) * ax + std::sin(yaw) * ay;
    const double left = -std::sin(yaw) * ax + std::cos(yaw) * ay;
    return {
      std::clamp(-std::atan2(left, g), -gains_.max_tilt, gains_.max_tilt),  // +roll → −y
      std::clamp(std::atan2(fwd, g), -gains_.max_tilt, gains_.max_tilt),    // +pitch → +x
      std::clamp(gains_.mass * (g + az) / std::max(cos_tilt, 0.5), 0.0, gains_.max_thrust)};
  }

  PositionGains & gains() {return gains_;}

private:
  PositionGains gains_;
  Eigen::Vector3d integral_ = Eigen::Vector3d::Zero();
};

}  // namespace ft_tilt
