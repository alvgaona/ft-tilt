"""ft_tilt sim + Aerostack2: platform, ground-truth state estimator, motion controller with the
ft_tilt plugin, take-off and land behaviours. Drone namespace: ft_tilt.

    pixi run as2 [gamma_deg:=107] [battery:=4s] [failure_gains:=mild] [headless:=false] [yaw:=0.0]
    pixi run mission            # in a second terminal
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = "ft_tilt"


def generate_launch_description():
    share = get_package_share_directory("ft_tilt_as2")
    config = os.path.join(share, "config", "as2.yaml")
    sim_time = {"use_sim_time": True}
    sim_args = ["gamma_deg", "battery", "failure_gains", "allocation", "headless", "yaw"]
    se = get_package_share_directory("as2_state_estimator")
    estimator_defaults = [  # every plugin parameter must be set; ours (last) override these
        os.path.join(se, "config", "state_estimator_default.yaml"),
        os.path.join(se, "plugins", "ground_truth", "config", "plugin_default.yaml"),
        os.path.join(share, "config", "state_estimator.yaml")]

    def behavior(name, plugin):
        return IncludeLaunchDescription(
            os.path.join(get_package_share_directory("as2_behaviors_motion"), "launch",
                         f"{name}_behavior_launch.py"),
            launch_arguments={"namespace": NS, "use_sim_time": "true", "plugin_name": plugin}.items())

    return LaunchDescription([
        DeclareLaunchArgument("gamma_deg", default_value="107"),
        DeclareLaunchArgument("battery", default_value="4s"),
        DeclareLaunchArgument("failure_gains", default_value="mild"),
        DeclareLaunchArgument("allocation", default_value="null_space"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory("ft_tilt_bringup"), "launch", "sim.launch.py"),
            launch_arguments={a: LaunchConfiguration(a) for a in sim_args}.items()),
        Node(package="ft_tilt_as2", executable="ft_tilt_platform", name="platform", namespace=NS,
             output="screen", emulate_tty=True,
             parameters=[sim_time, config, {
                 "control_modes_file": os.path.join(share, "config", "platform_control_modes.yaml")}]),
        Node(package="as2_state_estimator", executable="as2_state_estimator_node",
             name="state_estimator", namespace=NS, output="screen", emulate_tty=True,
             parameters=[sim_time, *estimator_defaults]),
        Node(package="as2_motion_controller", executable="as2_motion_controller_node",
             name="controller_manager", namespace=NS, output="screen", emulate_tty=True,
             parameters=[sim_time, config, {
                 "plugin_available_modes_config_file":
                     os.path.join(share, "config", "controller_available_modes.yaml")}]),
        behavior("takeoff", "takeoff_plugin_position"),
        behavior("land", "land_plugin_speed"),
    ])
