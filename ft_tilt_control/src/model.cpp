#include "ft_tilt_control/model.hpp"

#include <cmath>

namespace ft_tilt
{

Rotors hexagon(double arm_length, double gamma)
{
  Rotors rotors;
  for (int i = 0; i < kNumRotors; ++i) {
    const double theta = i * M_PI / 3.0;
    const Eigen::Vector3d d(std::cos(theta), std::sin(theta), 0.0);
    rotors[i].position = arm_length * d;
    rotors[i].axis = std::cos(gamma) * d + std::sin(gamma) * Eigen::Vector3d::UnitZ();
    rotors[i].spin = (i % 2 == 0) ? 1.0 : -1.0;  // motor 1 (i = 0) is +1
  }
  return rotors;
}

Matrix46d effectiveness(const Rotors & rotors, double kt_ratio)
{
  Matrix46d a;
  for (int i = 0; i < kNumRotors; ++i) {
    const Rotor & r = rotors[i];
    a.block<3, 1>(0, i) = r.position.cross(r.axis) + r.spin * kt_ratio * r.axis;
    a(3, i) = r.axis.z();
  }
  return a;
}

}  // namespace ft_tilt
