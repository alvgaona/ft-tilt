#pragma once

#include <array>

#include <Eigen/Dense>

namespace ft_tilt
{

constexpr int kNumRotors = 6;
using Vector6d = Eigen::Matrix<double, kNumRotors, 1>;
using Matrix46d = Eigen::Matrix<double, 4, kNumRotors>;

// One rotor in the body frame (FLU, origin at the centre of mass).
struct Rotor
{
  Eigen::Vector3d position;  // m
  Eigen::Vector3d axis;      // unit thrust direction
  double spin;               // +1: reaction torque is +k̃t·f·axis, -1: the opposite
};

using Rotors = std::array<Rotor, kNumRotors>;

// Paper geometry (RPIC17/ICRIS18 Fig. 1-2). Motor i sits on the arm at (i-1)·60° from +x
// toward +y, its axis is cos(γ)·d̂ + sin(γ)·ẑ (γ > 90° leans it toward the centre, as on
// the real vehicle) and its spin is (-1)^(i+1).
Rotors hexagon(double arm_length, double gamma);

// Force-torque matrix: [Mx, My, Mz, Fz]ᵀ = A·f, column i = [r_i × e_i + spin_i·k̃t·e_i ; ẑ·e_i].
// k̃t = k_t / k_f (m). The printed A of RPIC17 eq. (3) is this matrix with γ_paper = 180° − γ.
Matrix46d effectiveness(const Rotors & rotors, double kt_ratio);

}  // namespace ft_tilt
