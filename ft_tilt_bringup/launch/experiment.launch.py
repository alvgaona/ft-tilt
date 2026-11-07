"""Runs one scripted experiment, records it to bags/, and shuts down when it ends.

    ros2 launch ft_tilt_bringup experiment.launch.py config:=icris18_pulse battery:=4s
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory("ft_tilt_bringup")
    config = LaunchConfiguration("config")
    gamma = LaunchConfiguration("gamma_deg")
    gains = LaunchConfiguration("failure_gains")
    battery = LaunchConfiguration("battery")

    runner = Node(
        package="ft_tilt_bringup",
        executable="experiment_runner",
        parameters=[
            PathJoinSubstitution([bringup, "config", [config, ".yaml"]]),
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="icris18_pulse",
                description="icris18_pulse | icris18_yaw | rpic17_inflight_fault",
            ),
            DeclareLaunchArgument("gamma_deg", default_value="107"),
            DeclareLaunchArgument("failure_gains", default_value="mild"),
            DeclareLaunchArgument("battery", default_value="3s"),
            DeclareLaunchArgument("headless", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "sim.launch.py")),
                launch_arguments={
                    "gamma_deg": gamma,
                    "failure_gains": gains,
                    "battery": battery,
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
            runner,
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record", "--use-sim-time", "-e", "/ft_tilt/.*",
                    "-o", ["bags/", config, "_gamma", gamma, "_", gains, "_", battery],
                ],
                output="screen",
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=runner, on_exit=[EmitEvent(event=Shutdown())])
            ),
        ]
    )
