#include <gtest/gtest.h>

#include <cmath>

#include "ft_tilt_control/attitude_pid.hpp"

TEST(AttitudePid, YawErrorTakesShortestWay)
{
  ft_tilt::AttitudePid pid;
  const double d = M_PI / 180.0;
  const Eigen::Vector3d m = pid.update({0, 0, 179 * d}, {0, 0, -179 * d}, Eigen::Vector3d::Zero(), 0.0);
  EXPECT_NEAR(m.z(), ft_tilt::aggressive_gains().kp.z() * (-2 * d), 1e-12);
}

TEST(AttitudePid, GainSwitchIsBumpless)
{
  ft_tilt::AttitudePid pid;
  const Eigen::Vector3d ref(0.1, 0, 0), zero = Eigen::Vector3d::Zero();
  for (int i = 0; i < 200; ++i) {
    pid.update(ref, zero, zero, 0.005);
  }
  const double before = pid.update(zero, zero, zero, 0.005).x();  // integral term only
  pid.set_gains(ft_tilt::mild_gains());
  const double after = pid.update(zero, zero, zero, 0.005).x();
  EXPECT_DOUBLE_EQ(before, after);
  EXPECT_GT(before, 0.0);
}
