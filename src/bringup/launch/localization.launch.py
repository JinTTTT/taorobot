"""Localization bringup: particle filter on a saved map, with RViz.

Composes:

1. rviz2 with the shared localization view, started first so it is up before
   the map arrives (disable with use_rviz:=false).
2. After a short delay:
   - localization/particle_filter_localization.launch.py — serves the saved
     map on /map (latched) and runs Monte-Carlo localization (map -> odom).

Run the simulation (or a real robot providing /scan, /odom, base_link TF)
separately first, plus a teleop to move the robot:

    ros2 launch simulation bringup_simulation.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 launch bringup localization.launch.py

Then give the particle filter an initial guess with RViz "2D Pose Estimate"
and drive around to let the particles converge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("bringup"), "rviz", "localization.rviz"
    )

    default_map = os.path.join(
        get_package_share_directory("graph_pose_slam"), "maps", "slam_map.yaml"
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Absolute path to the map YAML served on /map.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz preconfigured with the localization view.",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("localization"),
                "launch",
                "particle_filter_localization.launch.py",
            )
        ),
        launch_arguments={"map": LaunchConfiguration("map")}.items(),
    )

    # RViz starts immediately; the stack follows once RViz has had time to
    # come up, so its displays are subscribed before the latched map arrives.
    stack = TimerAction(period=3.0, actions=[localization])

    return LaunchDescription([
        map_arg,
        use_rviz_arg,
        rviz,
        stack,
    ])
