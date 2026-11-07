"""Gazebo Harmonic with the tilted hexacopter and the ROS bridge.

Server and GUI run as separate processes, which works on Linux and macOS.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    desc = get_package_share_directory("ft_tilt_description")
    bringup = get_package_share_directory("ft_tilt_bringup")
    model = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(desc, "models", "ft_tilt.sdf.xacro"),
                " gamma_deg:=",
                LaunchConfiguration("gamma_deg"),
                " failure_gains:=",
                LaunchConfiguration("failure_gains"),
                " battery:=",
                LaunchConfiguration("battery"),
                " allocation:=",
                LaunchConfiguration("allocation"),
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gamma_deg", default_value="107", description="Rotor tilt (90 = standard)"
            ),
            DeclareLaunchArgument(
                "failure_gains",
                default_value="mild",
                description="Roll/pitch PID gains after a fault: mild | aggressive | rpic17 (RPIC17 set, also before the fault)",
            ),
            DeclareLaunchArgument(
                "battery",
                default_value="3s",
                description="3s: RPIC17 Table I | 4s: ICRIS18 total-failure vehicle (estimated)",
            ),
            DeclareLaunchArgument(
                "allocation",
                default_value="null_space",
                description="After a fault: null_space (fault-aware) | pinv_clamp (pseudo-inverse + clamp)",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn yaw (rad)"),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-s", "-v", "3",
                     os.path.join(bringup, "worlds", "ft_tilt_world.sdf")],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-g", "-v", "2",
                     "--gui-config", os.path.join(bringup, "config", "gui.config")],
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
            # Keep the GUI camera on the drone: it drops the follow target if the drone isn't
            # in its scene yet or the view is dragged. Re-send it quickly right after spawn, then
            # every 2 s (offset and gain come from config/gui.config).
            ExecuteProcess(
                cmd=["bash", "-c",
                     "follow() { gz service -s /gui/follow --reqtype gz.msgs.StringMsg "
                     "--reptype gz.msgs.Boolean --timeout 1000 --req 'data: \"ft_tilt\"'; }; "
                     "gz topic -e -t /model/ft_tilt/odometry -n 1 > /dev/null && "
                     "for i in $(seq 15); do follow > /dev/null; sleep 0.3; done; "
                     "while true; do sleep 2; follow > /dev/null; done"],
                condition=UnlessCondition(LaunchConfiguration("headless")),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                parameters=[
                    {"world": "ft_tilt_world", "name": "ft_tilt", "string": model, "z": 0.22,
                     "Y": ParameterValue(LaunchConfiguration("yaw"), value_type=float)}
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": os.path.join(bringup, "config", "bridge.yaml"),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
