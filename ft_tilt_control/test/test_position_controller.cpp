#include <cmath>

#include <gtest/gtest.h>

#include "ft_tilt_control/position_controller.hpp"

using ft_tilt::PositionController;
using Eigen::Vector3d;

TEST(PositionController, HoldsHoverAtReference)
{
  PositionController c;
  const auto out = c.update({1, 2, 1.5}, Vector3d::Zero(), {1, 2, 1.5}, Vector3d::Zero(), 0.3, 1.0, 0.01);
  EXPECT_NEAR(out.roll, 0.0, 1e-12);
  EXPECT_NEAR(out.pitch, 0.0, 1e-12);
  EXPECT_NEAR(out.thrust, 2.1 * 9.80665, 1e-9);
}

TEST(PositionController, TiltsTowardTheTargetInFlu)
{
  PositionController c;
  // Target ahead (+x) and to the left (+y), yaw 0: pitch forward (+), roll left (−).
  const auto out = c.update({0.5, 0.5, 1}, Vector3d::Zero(), {0, 0, 1}, Vector3d::Zero(), 0.0, 1.0, 0.01);
  EXPECT_GT(out.pitch, 0.0);
  EXPECT_LT(out.roll, 0.0);
  // Yawed 90°: the +y target is now straight ahead.
  const auto turned = PositionController().update(
    {0, 0.5, 1}, Vector3d::Zero(), {0, 0, 1}, Vector3d::Zero(), M_PI / 2, 1.0, 0.01);
  EXPECT_GT(turned.pitch, 0.0);
  EXPECT_NEAR(turned.roll, 0.0, 1e-12);
}

TEST(PositionController, ClampsTiltAndThrust)
{
  PositionController c;
  // The 2 m error cap limits the position term to ~2° of tilt; flying away at 10 m/s saturates it.
  const auto out = c.update({50, 0, 50}, Vector3d::Zero(), {0, 0, 0}, {-10, 0, 0}, 0.0, 1.0, 0.01);
  EXPECT_NEAR(out.pitch, c.gains().max_tilt, 1e-12);
  EXPECT_NEAR(out.thrust, c.gains().max_thrust, 1e-12);
  const auto down = c.update({0, 0, -50}, Vector3d::Zero(), {0, 0, 0}, Vector3d::Zero(), 0.0, 1.0, 0.01);
  EXPECT_EQ(down.thrust, 0.0);
}

TEST(PositionController, VelocityFeedforwardDescends)
{
  // Speed mode: on the reference but asked to sink at 0.5 m/s → less than hover thrust.
  PositionController c;
  const auto out = c.update({0, 0, 1}, {0, 0, -0.5}, {0, 0, 1}, Vector3d::Zero(), 0.0, 1.0, 0.01);
  EXPECT_NEAR(out.thrust, 2.1 * (9.80665 - 3.0 * 0.5), 1e-9);
}

TEST(PositionController, VerticalIntegralIsCapped)
{
  // Sitting on the ground while the reference is 1 m below (landing) for a minute: once back on
  // the reference, the integral costs at most 1 m/s² of thrust.
  PositionController c;
  for (int i = 0; i < 6000; ++i) {
    c.update({0, 0, -1}, Vector3d::Zero(), {0, 0, 0}, Vector3d::Zero(), 0.0, 1.0, 0.01);
  }
  const auto out = c.update({0, 0, 0}, Vector3d::Zero(), {0, 0, 0}, Vector3d::Zero(), 0.0, 1.0, 0.01);
  EXPECT_NEAR(out.thrust, 2.1 * (9.80665 - 1.0), 1e-6);
}
