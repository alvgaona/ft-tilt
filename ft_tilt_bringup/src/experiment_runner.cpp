// Scripted experiment: stands in for the papers' RC pilot. Holds altitude and position (the
// papers flew those by hand) by setting thrust and roll/pitch references for the paper's
// attitude loop, and schedules the fault, the disturbance and yaw steps on sim time.
// Exits when the experiment is over.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include <actuator_msgs/msg/actuators.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class ExperimentRunner : public rclcpp::Node
{
public:
  ExperimentRunner()
  : Node("experiment_runner")
  {
    duration_ = declare_parameter("duration", 40.0);
    takeoff_time_ = declare_parameter("takeoff_time", 1.0);
    altitude_ = declare_parameter("altitude", 1.5);
    mass_ = declare_parameter("mass", 2.1);
    kp_z_ = declare_parameter("altitude_kp", 3.0);
    ki_z_ = declare_parameter("altitude_ki", 0.5);
    kd_z_ = declare_parameter("altitude_kd", 3.0);
    max_thrust_ = declare_parameter("max_thrust", 30.0);
    // Well below the mild attitude loop (≈ 1 rad/s): linearised damping ≥ 0.4 with both gain
    // sets (derived). Pitching back needs −My, which is weak with motor 2 off. The integral
    // cancels the sideways force tilted rotors make after a fault (≈ 0.2 m/s², derived).
    kp_xy_ = declare_parameter("position_kp", 0.2);
    ki_xy_ = declare_parameter("position_ki", 0.02);
    kd_xy_ = declare_parameter("position_kd", 0.6);
    max_tilt_ = declare_parameter("max_tilt_deg", 10.0) * M_PI / 180.0;
    fault_time_ = declare_parameter("fault_time", -1.0);
    fault_clear_time_ = declare_parameter("fault_clear_time", -1.0);
    fault_motor_ = declare_parameter("fault_motor", 2);
    fault_health_ = declare_parameter("fault_health", 0.0);
    dist_start_ = declare_parameter("disturbance_start", -1.0);
    dist_end_ = declare_parameter("disturbance_end", -1.0);
    dist_torque_ = declare_parameter("disturbance_torque", std::vector<double>{0.0, 0.0, 0.0});
    yaw_times_ = declare_parameter("yaw_ref_times", std::vector<double>{});
    yaw_deg_ = declare_parameter("yaw_ref_deg", std::vector<double>{});

    if (fault_motor_ < 1 || fault_motor_ > 6 || dist_torque_.size() != 3 ||
      yaw_times_.size() != yaw_deg_.size())
    {
      throw std::invalid_argument(
              "fault_motor must be 1..6, disturbance_torque needs 3 values, "
              "yaw_ref_times and yaw_ref_deg must have the same length");
    }

    attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ft_tilt/attitude_ref", 10);
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>("/ft_tilt/thrust_ref", 10);
    health_pub_ = create_publisher<actuator_msgs::msg::Actuators>("/ft_tilt/motor_health", 10);
    dist_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ft_tilt/disturbance_torque", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ft_tilt/odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = *m;});
    timer_ = rclcpp::create_timer(this, get_clock(), 10ms, [this] {Step();});
  }

private:
  void Step()
  {
    const rclcpp::Time now = get_clock()->now();
    if (now.nanoseconds() == 0 || !odom_) {
      return;  // waiting for /clock and the vehicle
    }
    if (!t0_) {
      t0_ = now;
      RCLCPP_INFO(get_logger(), "experiment started");
    }
    const double t = (now - *t0_).seconds();
    const double dt = last_t_ ? t - *last_t_ : 0.0;
    last_t_ = t;
    if (t > duration_) {
      RCLCPP_INFO(get_logger(), "experiment finished at t = %.1f s", t);
      rclcpp::shutdown();
      return;
    }

    // Fault: health 1 = healthy, 0 = off, in between = PWM cap.
    actuator_msgs::msg::Actuators health;
    health.header.stamp = now;
    health.normalized.assign(6, 1.0);
    if (Active(t, fault_time_, fault_clear_time_)) {
      health.normalized[fault_motor_ - 1] = fault_health_;
    }
    health_pub_->publish(health);

    geometry_msgs::msg::Vector3 dist;
    if (Active(t, dist_start_, dist_end_)) {
      dist.x = dist_torque_[0];
      dist.y = dist_torque_[1];
      dist.z = dist_torque_[2];
    }
    dist_pub_->publish(dist);

    geometry_msgs::msg::Vector3 ref;
    for (size_t i = 0; i < yaw_times_.size(); ++i) {
      if (t >= yaw_times_[i]) {
        ref.z = yaw_deg_[i] * M_PI / 180.0;
      }
    }
    if (t < takeoff_time_) {
      attitude_pub_->publish(ref);
      return;  // motors stay off until the first thrust command
    }
    const auto & p = odom_->pose.pose;
    const auto & q = p.orientation;
    const auto & v = odom_->twist.twist.linear;  // body frame
    // World velocity = R·v, with R from the quaternion.
    const double r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);  // cos(roll)·cos(pitch)
    const double vx = (1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * v.x +
      2.0 * (q.x * q.y - q.w * q.z) * v.y + 2.0 * (q.x * q.z + q.w * q.y) * v.z;
    const double vy = 2.0 * (q.x * q.y + q.w * q.z) * v.x +
      (1.0 - 2.0 * (q.x * q.x + q.z * q.z)) * v.y + 2.0 * (q.y * q.z - q.w * q.x) * v.z;
    const double vz = 2.0 * (q.x * q.z - q.w * q.y) * v.x + 2.0 * (q.y * q.z + q.w * q.x) * v.y +
      r22 * v.z;

    // Position PID → horizontal acceleration → roll/pitch (small angles, FLU, yaw frame).
    if (!home_) {
      home_ = {p.position.x, p.position.y};
    }
    // Error capped at 2 m so a far-off vehicle flies back at ≈ kp·2/kd instead of overshooting.
    const double ex = std::clamp(home_->first - p.position.x, -2.0, 2.0);
    const double ey = std::clamp(home_->second - p.position.y, -2.0, 2.0);
    const double lim = 0.5 / std::max(ki_xy_, 1e-9);  // integral term ≤ 0.5 m/s² (anti-windup)
    x_integral_ = std::clamp(x_integral_ + ex * dt, -lim, lim);
    y_integral_ = std::clamp(y_integral_ + ey * dt, -lim, lim);
    const double ax = kp_xy_ * ex + ki_xy_ * x_integral_ - kd_xy_ * vx;
    const double ay = kp_xy_ * ey + ki_xy_ * y_integral_ - kd_xy_ * vy;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double fwd = std::cos(yaw) * ax + std::sin(yaw) * ay;
    const double left = -std::sin(yaw) * ax + std::cos(yaw) * ay;
    ref.y = std::clamp(std::atan2(fwd, 9.80665), -max_tilt_, max_tilt_);    // +pitch → +x
    ref.x = std::clamp(-std::atan2(left, 9.80665), -max_tilt_, max_tilt_);  // +roll → −y
    attitude_pub_->publish(ref);

    // Altitude PID → vertical acceleration → thrust along body z.
    const double e = altitude_ - p.position.z;
    z_integral_ += e * dt;
    const double accel = kp_z_ * e + ki_z_ * z_integral_ - kd_z_ * vz;
    std_msgs::msg::Float64 thrust;
    thrust.data = std::clamp(mass_ * (9.80665 + accel) / std::max(r22, 0.5), 0.0, max_thrust_);
    thrust_pub_->publish(thrust);
  }

  static bool Active(double t, double start, double end)
  {
    return start >= 0.0 && t >= start && (end < 0.0 || t < end);
  }

  double duration_, takeoff_time_, altitude_, mass_, kp_z_, ki_z_, kd_z_, max_thrust_;
  double kp_xy_, ki_xy_, kd_xy_, max_tilt_;
  double fault_time_, fault_clear_time_, fault_health_, dist_start_, dist_end_;
  int64_t fault_motor_;
  std::vector<double> dist_torque_, yaw_times_, yaw_deg_;

  std::optional<rclcpp::Time> t0_;
  std::optional<double> last_t_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  double z_integral_ = 0.0, x_integral_ = 0.0, y_integral_ = 0.0;
  std::optional<std::pair<double, double>> home_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_, dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr health_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentRunner>());
  rclcpp::shutdown();
  return 0;
}
