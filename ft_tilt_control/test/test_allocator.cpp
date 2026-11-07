#include <gtest/gtest.h>

#include <cmath>

#include "ft_tilt_control/allocator.hpp"

namespace
{

using ft_tilt::Matrix46d;
using ft_tilt::Vector6d;

// RPIC17 Table I vehicle at γ = 107° (axes leaning inward).
constexpr double kKf = 0.0776;               // N per % PWM
constexpr double kMaxForce = 0.644 * 9.81;   // F_M
constexpr double kLinearFloor = 14.0 * kKf;  // ICRIS18 14 % PWM floor, linear thrust model
constexpr double kFloor = 0.14 * 0.14 * kMaxForce;  // the same floor with thrust ~ PWM²
constexpr double kWeight = 2.1 * 9.81;

Matrix46d a_of(double gamma_deg)
{
  return ft_tilt::effectiveness(ft_tilt::hexagon(0.26, gamma_deg * M_PI / 180.0), 0.1);
}

struct Bounds
{
  Vector6d lower = Vector6d::Zero();
  Vector6d upper = Vector6d::Constant(kMaxForce);
  Bounds & fail(int i) {lower(i) = upper(i) = 0.0; return *this;}
};

const Eigen::Vector4d kHover(0.0, 0.0, 0.0, kWeight);

}  // namespace

TEST(Allocator, NominalHoverSharesThrustEqually)
{
  const Bounds b;
  const auto out = ft_tilt::allocate(a_of(107), kHover, b.lower, b.upper, false);
  const double each = kWeight / (6 * std::sin(107 * M_PI / 180.0));
  EXPECT_TRUE(out.force.isApprox(Vector6d::Constant(each), 1e-9));
  EXPECT_GE(out.margin, -1e-9);
}

TEST(Allocator, Motor2FailureHoversWithinLimits)
{
  const auto b = Bounds().fail(1);
  const Matrix46d a = a_of(107);
  const auto out = ft_tilt::allocate(a, kHover, b.lower, b.upper, true);
  EXPECT_EQ(out.force(1), 0.0);
  EXPECT_GE(out.margin, -1e-9);
  EXPECT_LT((a * out.force - kHover).norm(), 1e-9);
  // Motor 5, opposite the failure, runs far below the others (ICRIS18 Fig. 11).
  EXPECT_LT(out.force(4), 0.05 * out.force.maxCoeff());
}

TEST(Allocator, LinearFloorLeavesNoHoverAfterFailure)
{
  // w5 = α ≈ 0.07: lifting motor 5 to k_f·14 % pushes the others past F_M.
  auto b = Bounds().fail(1);
  for (int i : {0, 2, 3, 4, 5}) {
    b.lower(i) = kLinearFloor;
  }
  EXPECT_LT(ft_tilt::allocate(a_of(107), kHover, b.lower, b.upper, true).margin, 0.0);
}

TEST(Allocator, QuadraticFloorHoversAfterFailure)
{
  auto b = Bounds().fail(1);
  for (int i : {0, 2, 3, 4, 5}) {
    b.lower(i) = kFloor;
  }
  const Matrix46d a = a_of(107);
  const auto out = ft_tilt::allocate(a, kHover, b.lower, b.upper, true);
  EXPECT_GE(out.margin, -1e-9);
  EXPECT_EQ(out.force(1), 0.0);
  EXPECT_GE(out.force(4), kFloor - 1e-9);
  EXPECT_LT((a * out.force - kHover).norm(), 1e-9);
}

TEST(Allocator, KeepsMinimumNormWhenWithinBounds)
{
  const auto b = Bounds().fail(1);
  Matrix46d a = a_of(107);
  const auto out = ft_tilt::allocate(a, kHover, b.lower, b.upper, true);
  a.col(1).setZero();
  const Vector6d f0 = a.completeOrthogonalDecomposition().pseudoInverse() * kHover;
  EXPECT_TRUE(out.force.isApprox(f0, 1e-9));  // RPIC17: "most often w = 0"
}

TEST(Allocator, ReachesTorqueInEveryDirectionAfterFailure)
{
  const auto b = Bounds().fail(1);
  const Matrix46d a = a_of(107);
  for (int axis = 0; axis < 6; ++axis) {
    const Eigen::Vector3d dir = (axis % 2 ? -1.0 : 1.0) * Eigen::Vector3d::Unit(axis / 2);
    const Eigen::Vector4d v(0.05 * dir.x(), 0.05 * dir.y(), 0.05 * dir.z(), kWeight);
    const auto out = ft_tilt::allocate(a, v, b.lower, b.upper, true);
    EXPECT_GE(out.margin, -1e-9) << dir.transpose();
    EXPECT_LT((a * out.force - v).norm(), 1e-9) << dir.transpose();
  }
}

TEST(Allocator, UntiltedVehicleLosesOneYawDirection)
{
  // γ = 90°: f5 depends on the torque alone (Q5 = w5 = α = 0), so −Mz would need f5 < 0.
  const auto b = Bounds().fail(1);
  const Matrix46d a = a_of(90);
  const Eigen::Vector4d minus_yaw(0, 0, -0.05, kWeight), plus_yaw(0, 0, 0.05, kWeight);
  const auto bad = ft_tilt::allocate(a, minus_yaw, b.lower, b.upper, true);
  EXPECT_LT(bad.margin, 0.0);
  EXPECT_GE(bad.force.minCoeff(), 0.0);  // clamped
  EXPECT_GE(ft_tilt::allocate(a, plus_yaw, b.lower, b.upper, true).margin, -1e-9);
  // The tilted vehicle handles both.
  EXPECT_GE(ft_tilt::allocate(a_of(107), minus_yaw, b.lower, b.upper, true).margin, -1e-9);
}

TEST(Allocator, PartialFaultRespectsCap)
{
  Bounds b;
  b.upper(1) = 0.4 * kMaxForce;  // RPIC17: motor 2 limited to 40 % of its maximum thrust
  const Matrix46d a = a_of(107);
  const auto out = ft_tilt::allocate(a, kHover, b.lower, b.upper, true);
  EXPECT_GE(out.margin, -1e-9);
  EXPECT_LE(out.force(1), b.upper(1));
  EXPECT_LT((a * out.force - kHover).norm(), 1e-9);
}
