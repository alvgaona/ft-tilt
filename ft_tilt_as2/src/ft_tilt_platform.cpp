// Aerostack2 platform for the ft_tilt Gazebo vehicle. A thin adapter to the "flight controller"
// (the ft_tilt::FtTiltController gz system: attitude PID → fault-aware allocation, 200 Hz):
//   ATTITUDE (roll, pitch, yaw angle + collective thrust) → /ft_tilt/attitude_ref, thrust_ref
//   /ft_tilt/odometry → ground_truth/pose (earth) and ground_truth/twist (base frame)
// On hardware the same commands would go to the flight board instead.

#include <algorithm>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "as2_core/aerial_platform.hpp"
#include "as2_core/core_functions.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/utils/control_mode_utils.hpp"

class FtTiltPlatform : public as2::AerialPlatform
{
public:
  explicit FtTiltPlatform(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : as2::AerialPlatform(options)
  {
    attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3>("/ft_tilt/attitude_ref", 10);
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>("/ft_tilt/thrust_ref", 10);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      as2_names::topics::ground_truth::pose, as2_names::topics::ground_truth::qos);
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::ground_truth::twist, as2_names::topics::ground_truth::qos);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ft_tilt/odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = m->header.stamp;
        pose.header.frame_id = "earth";  // gz world = earth (ground_truth wants this frame)
        pose.pose = m->pose.pose;
        pose_pub_->publish(pose);
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = m->header.stamp;
        twist.header.frame_id = getBaseFrameId();  // gz odometry twist is in the body frame
        twist.twist = m->twist.twist;
        twist_pub_->publish(twist);
      });
  }

  void configureSensors() override {}

  bool ownSetArmingState(bool state) override
  {
    if (!state) {
      motorsOff();
    }
    return true;  // motors start with the first thrust command
  }

  bool ownSetOffboardControl(bool) override {return true;}

  bool ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & mode) override
  {
    RCLCPP_INFO(get_logger(), "Control mode: [%s]", as2::control_mode::controlModeToString(mode).c_str());
    // shortcut: odom yaw = gz world yaw only while the vehicle spawns at yaw 0; use the
    // earth → odom transform when it doesn't.
    setCommandPoseFrameId(getOdomFrameId());
    return mode.control_mode == as2_msgs::msg::ControlMode::ATTITUDE;
  }

  bool ownSendCommand() override
  {
    const auto & q = command_pose_msg_.pose.orientation;  // ZYX Euler, as the gz plugin reads it
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    rpy.y = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    rpy.z = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    attitude_pub_->publish(rpy);
    yaw_ = rpy.z;
    std_msgs::msg::Float64 thrust;
    thrust.data = std::max(0.0, static_cast<double>(command_thrust_msg_.thrust));
    thrust_pub_->publish(thrust);
    return true;
  }

  void ownKillSwitch() override {motorsOff();}

  void ownStopPlatform() override
  {
    // Emergency: level out and keep the last thrust; the motion controller is not trusted here.
    geometry_msgs::msg::Vector3 level;
    level.z = yaw_;
    attitude_pub_->publish(level);
  }

private:
  void motorsOff()
  {
    std_msgs::msg::Float64 off;
    off.data = -1.0;  // the gz plugin switches the motors off on a negative thrust
    thrust_pub_->publish(off);
  }

  double yaw_ = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FtTiltPlatform>();
  as2::spinLoop(node);
  rclcpp::shutdown();
  return 0;
}
