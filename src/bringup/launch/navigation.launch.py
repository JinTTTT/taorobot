"""Full navigation bringup: localize on a saved map, plan, and follow paths.

Composes the per-package launch files into one goal-to-goal navigation stack:

1. rviz2 with the shared navigation view, started first so it is up before
   the map arrives (disable with use_rviz:=false).
2. After a short delay, in sequence:
   - localization/particle_filter_localization.launch.py — serves the saved
     map on /map (latched) and runs Monte-Carlo localization (map -> odom).
   - motion_planning/motion_planning.launch.py — A* global planner with
     inflation and path smoothing.
   - path_follow_control/path_follow_control.launch.py — pure-pursuit
     follower publishing /cmd_vel.

Run the simulation (or a real robot providing /scan, /odom, base_link TF)
separately first:

    ros2 launch simulation bringup_simulation.launch.py
    ros2 launch bringup navigation.launch.py

Then give the particle filter an initial guess with RViz "2D Pose Estimate"
and send a goal with "2D Goal Pose".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_launch(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), "launch", launch_file
            )
        ),
        launch_arguments=launch_arguments,
    )


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("bringup"), "rviz", "navigation.rviz"
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
        description="Start RViz preconfigured with the navigation view.",
    )

    localization = package_launch(
        "localization",
        "particle_filter_localization.launch.py",
        launch_arguments={"map": LaunchConfiguration("map")}.items(),
    )

    motion_planning = package_launch("motion_planning", "motion_planning.launch.py")

    path_follow_control = package_launch(
        "path_follow_control", "path_follow_control.launch.py"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # RViz starts immediately; the stack follows once RViz has had time to
    # come up, so its displays are subscribed before the latched map arrives.
    stack = TimerAction(
        period=3.0,
        actions=[
            localization,
            motion_planning,
            path_follow_control,
        ],
    )

    return LaunchDescription([
        map_arg,
        use_rviz_arg,
        rviz,
        stack,
    ])
