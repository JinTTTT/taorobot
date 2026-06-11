"""SLAM bringup: graph-pose SLAM with RViz.

Composes:

1. rviz2 with the shared SLAM view, started first so it is up before the
   first map is published (disable with use_rviz:=false).
2. After a short delay:
   - graph_pose_slam/graph_pose_slam.launch.py — keyframe pose-graph SLAM
     publishing /map, /poses_graph, /estimated_pose, and map -> odom.

Run the simulation separately first, plus a teleop to move the robot:

    ros2 launch simulation bringup_simulation.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ros2 launch bringup slam.launch.py

Do NOT run localization or the mapping node at the same time — they also
publish /map and map -> odom.
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
        get_package_share_directory("bringup"), "rviz", "slam.rviz"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz preconfigured with the SLAM view.",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("graph_pose_slam"),
                "launch",
                "graph_pose_slam.launch.py",
            )
        )
    )

    # RViz starts immediately; SLAM follows once RViz has had time to come
    # up, so its displays are subscribed before the first latched map.
    stack = TimerAction(period=3.0, actions=[slam])

    return LaunchDescription([
        use_rviz_arg,
        rviz,
        stack,
    ])
