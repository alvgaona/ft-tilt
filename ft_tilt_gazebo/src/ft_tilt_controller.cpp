// The papers' flight software at 200 Hz: attitude PID → fault-aware allocation → rotor speeds.
// Topics, all under /model/<model name>/:
//   in   attitude_ref        gz.msgs.Vector3d   roll, pitch, yaw (rad)
//   in   thrust_ref          gz.msgs.Double     Fz along body z (N); motors stay off until the first one,
//                                                  and a negative value switches them off (disarm)
//   in   motor_health        gz.msgs.Actuators  normalized[i]: 1 healthy, 0 failed, else thrust cap h·F_M (RPIC17 §V)
//   in   disturbance_torque  gz.msgs.Vector3d   body-frame torque applied to the base link (N·m)
//   out  pwm                 gz.msgs.Actuators  normalized[i] = PWM (%), thrust ~ PWM² (thrust.hpp)
//   out  attitude            gz.msgs.Vector3d   measured roll, pitch, yaw (rad), ground truth
//   out  torque_cmd          gz.msgs.Vector3d   PID output Mx, My, Mz (N·m)
//   out  allocation_margin   gz.msgs.Double     smallest distance to 0 / F_M (N), < 0 means clamped

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Actuators.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "ft_tilt_control/allocator.hpp"
#include "ft_tilt_control/attitude_pid.hpp"
#include "ft_tilt_control/model.hpp"
#include "ft_tilt_control/thrust.hpp"

namespace ft_tilt
{

using gz::sim::EntityComponentManager;

class FtTiltController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm)) {
      gzerr << "FtTiltController must be attached to a model.\n";
      return;
    }
    auto cfg = sdf->Clone();

    base_ = gz::sim::Link(model_.LinkByName(ecm, cfg->Get<std::string>("base_link", "base_link").first));
    if (!base_.Valid(ecm)) {
      gzerr << "FtTiltController: base link not found.\n";
      return;
    }
    base_.EnableVelocityChecks(ecm, true);

    // Rotor geometry comes from the model itself, so A always matches what Gazebo simulates.
    const auto base_pose = ecm.Component<gz::sim::components::Pose>(base_.Entity())->Data();
    Rotors rotors;
    int n = 0;
    for (auto e = cfg->HasElement("rotor") ? cfg->GetElement("rotor") : nullptr; e;
      e = e->GetNextElement("rotor"))
    {
      const auto link = model_.LinkByName(ecm, e->Get<std::string>("link"));
      if (n >= kNumRotors || link == gz::sim::kNullEntity) {
        gzerr << "FtTiltController: expected 6 <rotor> with valid <link>.\n";
        return;
      }
      const auto rel = base_pose.Inverse() * ecm.Component<gz::sim::components::Pose>(link)->Data();
      const auto axis = rel.Rot().RotateVector(gz::math::Vector3d::UnitZ);
      rotors[n++] = {
        {rel.Pos().X(), rel.Pos().Y(), rel.Pos().Z()}, {axis.X(), axis.Y(), axis.Z()},
        e->Get<double>("spin")};
    }
    if (n != kNumRotors) {
      gzerr << "FtTiltController: expected 6 <rotor>, got " << n << ".\n";
      return;
    }

    a_ = effectiveness(rotors, cfg->Get<double>("kt_ratio"));
    kf_ = cfg->Get<double>("kf");
    max_force_ = cfg->Get<double>("max_force");
    // 14 % PWM through thrust ~ PWM²: the linear k_f·14 % (1.1 N) leaves no hover with motor 2 dead.
    floor_ = pwm_to_force(cfg->Get<double>("pwm_floor", 14.0).first, kf_, max_force_);
    motor_constant_ = cfg->Get<double>("motor_constant");
    period_ = 1.0 / cfg->Get<double>("rate", 200.0).first;
    const auto gains = cfg->Get<std::string>("failure_gains", "mild").first;
    failure_gains_ = gains == "aggressive" ? aggressive_gains() :
      gains == "rpic17" ? rpic17_gains() : mild_gains();
    nominal_gains_ = gains == "rpic17" ? rpic17_gains() : aggressive_gains();
    // pinv_clamp ignores a partial fault (assumed for RPIC17's standard vehicle, the paper doesn't say).
    const auto allocation = cfg->Get<std::string>("allocation", "null_space").first;
    null_space_on_fault_ = allocation != "pinv_clamp";
    gzmsg << "FtTiltController: A =\n" << a_ << "\nfailure gains: " << gains
          << ", allocation: " << allocation << "\n";

    const std::string ns = "/model/" + model_.Name(ecm) + "/";
    node_.Subscribe(ns + "attitude_ref", &FtTiltController::on_attitude_ref, this);
    node_.Subscribe(ns + "thrust_ref", &FtTiltController::on_thrust_ref, this);
    node_.Subscribe(ns + "motor_health", &FtTiltController::on_motor_health, this);
    node_.Subscribe(ns + "disturbance_torque", &FtTiltController::on_disturbance, this);
    pwm_pub_ = node_.Advertise<gz::msgs::Actuators>(ns + "pwm");
    attitude_pub_ = node_.Advertise<gz::msgs::Vector3d>(ns + "attitude");
    torque_pub_ = node_.Advertise<gz::msgs::Vector3d>(ns + "torque_cmd");
    margin_pub_ = node_.Advertise<gz::msgs::Double>(ns + "allocation_margin");

    rotor_cmd_.mutable_velocity()->Resize(kNumRotors, 0.0);
    configured_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {
      return;
    }
    const auto pose = base_.WorldPose(ecm);
    const auto omega = base_.WorldAngularVelocity(ecm);
    if (!pose || !omega) {
      return;
    }

    Eigen::Vector3d rpy_ref, disturbance;
    std::optional<double> thrust;
    Vector6d health;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      rpy_ref = rpy_ref_;
      thrust = thrust_ref_;
      health = health_;
      disturbance = disturbance_;
    }

    // Disturbance on every physics step; the controller only at its own rate.
    if (!disturbance.isZero()) {
      const auto t = pose->Rot().RotateVector({disturbance.x(), disturbance.y(), disturbance.z()});
      base_.AddWorldWrench(ecm, gz::math::Vector3d::Zero, t);
    }

    const double now = std::chrono::duration<double>(info.simTime).count();
    if (last_update_ && now - *last_update_ < period_ - 1e-9) {
      return;
    }
    const double dt = last_update_ ? now - *last_update_ : period_;
    last_update_ = now;

    const Eigen::Vector3d rpy(pose->Rot().Roll(), pose->Rot().Pitch(), pose->Rot().Yaw());
    const auto w = pose->Rot().RotateVectorReverse(*omega);
    const Eigen::Vector3d rates(w.X(), w.Y(), w.Z());

    Vector6d force = Vector6d::Zero();
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    double margin = 0.0;
    if (thrust) {
      const bool fault = (health.array() < 1.0).any();
      pid_.set_gains(fault ? failure_gains_ : nominal_gains_);
      torque = pid_.update(rpy_ref, rpy, rates, dt);

      Vector6d lower, upper;
      for (int i = 0; i < kNumRotors; ++i) {
        upper(i) = health(i) * max_force_;
        lower(i) = std::min(floor_, upper(i));  // never stop a working motor (ICRIS18 §V)
      }
      const Eigen::Vector4d wrench(torque.x(), torque.y(), torque.z(), *thrust);
      const auto alloc = allocate(a_, wrench, lower, upper, fault && null_space_on_fault_);
      margin = alloc.margin;
      force = alloc.force;
    } else {
      pid_.reset();
    }

    for (int i = 0; i < kNumRotors; ++i) {
      rotor_cmd_.set_velocity(i, std::sqrt(force(i) / motor_constant_));
    }
    set_rotor_command(ecm);
    publish(info, rpy, torque, force, margin);
  }

private:
  // A NaN would reach the rotor speeds and the physics, so non-finite messages are dropped.
  static bool all_finite(const std::vector<double> & values, const char * topic)
  {
    for (double v : values) {
      if (!std::isfinite(v)) {
        gzerr << topic << ": ignoring a non-finite value.\n";
        return false;
      }
    }
    return true;
  }

  void on_attitude_ref(const gz::msgs::Vector3d & msg)
  {
    if (!all_finite({msg.x(), msg.y(), msg.z()}, "attitude_ref")) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    rpy_ref_ = {msg.x(), msg.y(), msg.z()};
  }

  void on_thrust_ref(const gz::msgs::Double & msg)
  {
    if (!all_finite({msg.data()}, "thrust_ref")) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    thrust_ref_ = msg.data() < 0.0 ? std::nullopt : std::optional<double>(msg.data());
  }

  void on_motor_health(const gz::msgs::Actuators & msg)
  {
    if (msg.normalized_size() != kNumRotors) {
      gzerr << "motor_health needs 6 normalized values, got " << msg.normalized_size() << ".\n";
      return;
    }
    if (!all_finite(std::vector<double>(msg.normalized().begin(), msg.normalized().end()), "motor_health")) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < kNumRotors; ++i) {
      health_(i) = std::clamp(msg.normalized(i), 0.0, 1.0);
    }
  }

  void on_disturbance(const gz::msgs::Vector3d & msg)
  {
    if (!all_finite({msg.x(), msg.y(), msg.z()}, "disturbance_torque")) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    disturbance_ = {msg.x(), msg.y(), msg.z()};
  }

  // Same mechanism as Gazebo's multicopter controllers: the motor model reads this component.
  void set_rotor_command(EntityComponentManager & ecm)
  {
    auto comp = ecm.Component<gz::sim::components::Actuators>(model_.Entity());
    if (!comp) {
      ecm.CreateComponent(model_.Entity(), gz::sim::components::Actuators(rotor_cmd_));
      return;
    }
    comp->Data() = rotor_cmd_;
    ecm.SetChanged(
      model_.Entity(), gz::sim::components::Actuators::typeId,
      gz::sim::ComponentState::PeriodicChange);
  }

  void publish(
    const gz::sim::UpdateInfo & info, const Eigen::Vector3d & rpy, const Eigen::Vector3d & torque,
    const Vector6d & force, double margin)
  {
    const auto stamp = gz::msgs::Convert(info.simTime);
    gz::msgs::Actuators pwm;
    *pwm.mutable_header()->mutable_stamp() = stamp;
    for (int i = 0; i < kNumRotors; ++i) {
      pwm.add_normalized(force_to_pwm(force(i), kf_, max_force_));
    }
    pwm_pub_.Publish(pwm);

    gz::msgs::Vector3d v;
    *v.mutable_header()->mutable_stamp() = stamp;
    v.set_x(rpy.x()); v.set_y(rpy.y()); v.set_z(rpy.z());
    attitude_pub_.Publish(v);
    v.set_x(torque.x()); v.set_y(torque.y()); v.set_z(torque.z());
    torque_pub_.Publish(v);

    gz::msgs::Double m;
    *m.mutable_header()->mutable_stamp() = stamp;
    m.set_data(margin);
    margin_pub_.Publish(m);
  }

  bool configured_ = false;
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Link base_;
  Matrix46d a_;
  double kf_ = 0, max_force_ = 0, floor_ = 0, motor_constant_ = 1, period_ = 0.005;
  PidGains failure_gains_ = mild_gains();
  PidGains nominal_gains_ = aggressive_gains();
  bool null_space_on_fault_ = true;
  AttitudePid pid_;
  std::optional<double> last_update_;
  gz::msgs::Actuators rotor_cmd_;

  gz::transport::Node node_;
  gz::transport::Node::Publisher pwm_pub_, attitude_pub_, torque_pub_, margin_pub_;

  std::mutex mutex_;
  Eigen::Vector3d rpy_ref_ = Eigen::Vector3d::Zero();
  std::optional<double> thrust_ref_;
  Vector6d health_ = Vector6d::Ones();
  Eigen::Vector3d disturbance_ = Eigen::Vector3d::Zero();
};

}  // namespace ft_tilt

GZ_ADD_PLUGIN(
  ft_tilt::FtTiltController, gz::sim::System,
  ft_tilt::FtTiltController::ISystemConfigure,
  ft_tilt::FtTiltController::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(ft_tilt::FtTiltController, "ft_tilt::FtTiltController")
