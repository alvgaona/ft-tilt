#include "ft_tilt_control/attitude_pid.hpp"

#include <cmath>

namespace ft_tilt
{

Eigen::Vector3d AttitudePid::update(
  const Eigen::Vector3d & rpy_ref, const Eigen::Vector3d & rpy,
  const Eigen::Vector3d & body_rates, double dt)
{
  Eigen::Vector3d e = rpy_ref - rpy;
  e.z() = std::remainder(e.z(), 2.0 * M_PI);  // shortest way around in yaw
  integral_ += gains_.ki.cwiseProduct(e) * dt;
  return gains_.kp.cwiseProduct(e) + integral_ - gains_.kd.cwiseProduct(body_rates);
}

}  // namespace ft_tilt
