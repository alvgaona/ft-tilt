// Aerostack2 motion controller plugin: the ft_tilt position loop (ft_tilt_control,
// PositionController). POSITION (take-off, hover, go-to) and SPEED (land) in, ATTITUDE out,
// for the ft_tilt platform and its fault-tolerant attitude loop + allocation.
// Works in the odom frame (FLU, z up) for both pose and velocity.

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <string>

#include <pluginlib/class_list_macros.hpp>

#include "as2_core/utils/frame_utils.hpp"
#include "as2_motion_controller/controller_base.hpp"
#include "ft_tilt_control/position_controller.hpp"

namespace ft_tilt_motion_controller
{

using as2_msgs::msg::ControlMode;
using Eigen::Vector3d;

class Plugin : public as2_motion_controller_plugin_base::ControllerBase
{
public:
  void ownInitialize() override
  {
    setDesiredTwistFrameId(getDesiredPoseFrameId());  // the position law needs world velocity
  }

  void updateParameter(const std::string & name, const rclcpp::Parameter & param) override
  {
    auto & g = position_.gains();
    const std::map<std::string, double *> gains{
      {"mass", &g.mass}, {"max_thrust", &g.max_thrust},
      {"position_kp", &g.kp_xy}, {"position_ki", &g.ki_xy}, {"position_kd", &g.kd_xy},
      {"altitude_kp", &g.kp_z}, {"altitude_ki", &g.ki_z}, {"altitude_kd", &g.kd_z}};
    if (name == "max_speed") {
      max_speed_ = param.as_double();
    } else if (name == "max_tilt_deg") {
      g.max_tilt = param.as_double() * M_PI / 180.0;
    } else if (const auto it = gains.find(name); it != gains.end()) {
      *it->second = param.as_double();
    } else {
      RCLCPP_ERROR(getNodePtr()->get_logger(), "Unknown parameter '%s'", name.c_str());
    }
  }

  ControlMode hoverMode() const override
  {
    ControlMode mode;
    mode.control_mode = ControlMode::POSITION;
    mode.yaw_mode = ControlMode::YAW_ANGLE;
    return mode;
  }

  bool onSetMode(const ControlMode & mode_in, const ControlMode &) override
  {
    carrot_.reset();  // the moving reference restarts from where the vehicle is
    v_ref_.setZero();
    if (mode_in.yaw_mode == ControlMode::YAW_SPEED) {
      yaw_ref_ = yaw_;
    }
    return true;  // the integral is kept, so switching modes (take-off → hover) doesn't jump
  }

  void onUpdateState(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::TwistStamped & twist) override
  {
    p_ = {pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
    v_ = {twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z};
    const auto & q = pose.pose.orientation;
    yaw_ = as2::frame::getYawFromQuaternion(q);
    cos_tilt_ = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);  // cos(roll)·cos(pitch)
  }

  void onUpdateReference(const geometry_msgs::msg::PoseStamped & ref) override
  {
    p_ref_ = {ref.pose.position.x, ref.pose.position.y, ref.pose.position.z};
    yaw_ref_ = as2::frame::getYawFromQuaternion(ref.pose.orientation);
  }

  void onUpdateReference(const geometry_msgs::msg::TwistStamped & ref) override
  {
    // SPEED: the velocity to fly. POSITION: per-axis speed limits (0 = max_speed).
    v_ref_ = {ref.twist.linear.x, ref.twist.linear.y, ref.twist.linear.z};
    yaw_rate_ref_ = ref.twist.angular.z;
  }

  bool computeOutput(
    double dt, geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::TwistStamped &,
    as2_msgs::msg::Thrust & thrust) override
  {
    if (!isStateReceived() || !isReferenceReceived()) {
      return false;
    }
    // Track a reference point (carrot) that moves at the commanded speed (SPEED), or toward
    // the target no faster than the speed limits (POSITION), with that speed as feedforward.
    const auto & mode = getControlModeIn();
    Vector3d carrot = carrot_.value_or(p_), v_ff;
    if (mode.control_mode == ControlMode::SPEED) {
      v_ff = v_ref_;
      // Kept within 1 m of the vehicle so it doesn't run away (e.g. into the ground on landing).
      carrot = (carrot + v_ff * dt).cwiseMax(p_ - Vector3d::Ones()).cwiseMin(p_ + Vector3d::Ones());
    } else {
      // Never jump: the limit can arrive a tick after the pose (separate topics).
      const Vector3d step = p_ref_ - carrot;
      for (int i = 0; i < 3; ++i) {
        const double lim = (v_ref_(i) != 0.0 ? std::abs(v_ref_(i)) : max_speed_) * dt;
        v_ff(i) = std::clamp(step(i), -lim, lim) / dt;
      }
      carrot += v_ff * dt;
    }
    carrot_ = carrot;
    const Vector3d p_ref = carrot;
    if (mode.yaw_mode == ControlMode::YAW_SPEED) {
      yaw_ref_ += yaw_rate_ref_ * dt;
    }
    const auto out = position_.update(p_ref, v_ff, p_, v_, yaw_, cos_tilt_, dt);

    // ZYX: q = Rz(yaw)·Ry(pitch)·Rx(roll)
    const double cr = std::cos(out.roll / 2), sr = std::sin(out.roll / 2);
    const double cp = std::cos(out.pitch / 2), sp = std::sin(out.pitch / 2);
    const double cy = std::cos(yaw_ref_ / 2), sy = std::sin(yaw_ref_ / 2);
    pose.header.frame_id = getDesiredPoseFrameId();
    pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    thrust.thrust = out.thrust;
    return true;
  }

  void reset() override
  {
    ControllerBase::reset();
    position_.reset();
    carrot_.reset();
  }

private:
  ft_tilt::PositionController position_;
  Vector3d p_ = Vector3d::Zero(), v_ = Vector3d::Zero();
  Vector3d p_ref_ = Vector3d::Zero(), v_ref_ = Vector3d::Zero();
  std::optional<Vector3d> carrot_;
  double yaw_ = 0.0, yaw_ref_ = 0.0, yaw_rate_ref_ = 0.0, cos_tilt_ = 1.0;
  double max_speed_ = 1.0;  // m/s, POSITION references that carry no speed limit
};

}  // namespace ft_tilt_motion_controller

PLUGINLIB_EXPORT_CLASS(ft_tilt_motion_controller::Plugin, as2_motion_controller_plugin_base::ControllerBase)
