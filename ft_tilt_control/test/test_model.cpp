// Checks the geometric model against the closed forms printed in RPIC17 (eqs. 3, 6) and
// ICRIS18 (eq. 4). The printed A uses γ_paper = 180° − γ (see AGENTS.md, "Tilt direction").

#include <gtest/gtest.h>

#include <cmath>

#include "ft_tilt_control/model.hpp"

namespace
{

using ft_tilt::Matrix46d;

constexpr double kKt = 0.1;     // k̃t (m), RPIC17 Table I
constexpr double kArm = 0.26;   // l (m)
const double kSqrt3 = std::sqrt(3.0);

double deg(double d) {return d * M_PI / 180.0;}
double alpha_of(double gp) {return kKt / (kSqrt3 * kArm * std::tan(gp));}

// RPIC17 eq. (3) with ICRIS18's +sin(γ) thrust row.
Matrix46d printed_a(double gp)
{
  const double a = alpha_of(gp), c = std::cos(gp), s = std::sin(gp);
  Matrix46d m;
  m.row(0) << -1, (a + 1) / (2 * a), (a + 1) / (2 * a), -1, (a - 1) / (2 * a), (a - 1) / (2 * a);
  m.row(0) *= kKt * c;
  m.row(1) << -1 / (kSqrt3 * a), (3 * a - 1) / (2 * kSqrt3 * a), -(3 * a - 1) / (2 * kSqrt3 * a),
    1 / (kSqrt3 * a), (3 * a + 1) / (2 * kSqrt3 * a), -(3 * a + 1) / (2 * kSqrt3 * a);
  m.row(1) *= kKt * c;
  m.row(2) << 1, -1, 1, -1, 1, -1;
  m.row(2) *= kKt * s;
  m.row(3).setConstant(s);
  return m;
}

Matrix46d model_a(double gp)
{
  return ft_tilt::effectiveness(ft_tilt::hexagon(kArm, M_PI - gp), kKt);
}

Matrix46d without_motor(Matrix46d m, int i)
{
  m.col(i).setZero();
  return m;
}

// RPIC17 eq. (6), Ã_2†. k = k̃t, D = 3α²+1.
Eigen::Matrix<double, 6, 4> printed_pinv_a2(double gp)
{
  const double a = alpha_of(gp), k = kKt, D = 3 * a * a + 1;
  const double sec = 1 / std::cos(gp), csc = 1 / std::sin(gp);
  Eigen::Matrix<double, 6, 4> p;
  p.row(0) << -a * (15 * a * a * a + 9 * a * a + 9 * a - 1) * sec / (4 * k * D * D),
    -a * (9 * a * a * a + 27 * a * a - 9 * a + 5) * sec / (4 * kSqrt3 * k * D * D),
    std::pow(3 * a + 1, 2) * csc / (12 * k * D), std::pow(a - 1, 2) * csc / (4 * D);
  p.row(1).setZero();
  p.row(2) << 3 * a * std::pow(a + 1, 3) * sec / (4 * k * D * D),
    -a * std::pow(3 * a - 1, 3) * sec / (4 * kSqrt3 * k * D * D),
    std::pow(3 * a - 1, 2) * csc / (12 * k * D), std::pow(a + 1, 2) * csc / (4 * D);
  p.row(3) << -a * (3 * a - 1) * sec / (4 * k * D), kSqrt3 * a * (a + 1) * sec / (4 * k * D),
    -csc / (4 * k), csc / 4;
  p.row(4) << a * (3 * a * a * a - 1) * sec / (k * D * D),
    a * (9 * a * a * a + 1) * sec / (kSqrt3 * k * D * D), csc / (3 * k * D), a * a * csc / D;
  p.row(5) << a * (3 * a - 1) * sec / (4 * k * D), -kSqrt3 * a * (a + 1) * sec / (4 * k * D),
    -csc / (4 * k), csc / 4;
  return p;
}

}  // namespace

TEST(Model, MatchesPrintedForceTorqueMatrix)
{
  for (double gp : {deg(107), deg(73), deg(60), deg(120)}) {
    EXPECT_TRUE(model_a(gp).isApprox(printed_a(gp), 1e-12)) << "gamma_paper = " << gp;
  }
}

TEST(Model, NullVectorOfMotor2Failure)
{
  const double a = alpha_of(deg(107));
  ft_tilt::Vector6d w;
  w << 0.75 * (a - 1) * (a + 1.0 / 3), 0, -0.75 * (a + 1) * (a - 1.0 / 3),
    -0.75 * (a * a + 1.0 / 3), a, 0.75 * (a * a + 1.0 / 3);
  EXPECT_LT((without_motor(model_a(deg(107)), 1) * w).norm(), 1e-12);
}

TEST(Model, PseudoInverseMatchesRpic17Eq6)
{
  for (double gp : {deg(107), deg(73), deg(100)}) {
    const Eigen::MatrixXd pinv =
      without_motor(model_a(gp), 1).completeOrthogonalDecomposition().pseudoInverse();
    EXPECT_TRUE(pinv.isApprox(printed_pinv_a2(gp), 1e-9)) << "gamma_paper = " << gp;
  }
}

TEST(Model, ThrustColumnPositiveOnlyWhenTilted)
{
  auto q = [](double gp) -> Eigen::VectorXd {
      return without_motor(model_a(gp), 1).completeOrthogonalDecomposition().pseudoInverse().col(3);
    };
  // Tilted (0 < |α| < 1): every working motor has Q_i > 0.
  const Eigen::VectorXd q107 = q(deg(107));
  for (int i : {0, 2, 3, 4, 5}) {
    EXPECT_GT(q107(i), 0.0) << "motor " << i + 1;
  }
  // γ = 90° (α = 0): motor 5 (index 4) gets Q_5 = 0. |α| = 1: Q_1 = 0.
  EXPECT_NEAR(q(deg(90))(4), 0.0, 1e-12);
  EXPECT_NEAR(q(std::atan(kKt / (kSqrt3 * kArm)))(0), 0.0, 1e-9);
}
