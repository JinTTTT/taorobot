from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("graph_pose_slam"),
        "config",
        "graph_pose_slam.yaml",
    )

    return LaunchDescription([
        Node(
            package="graph_pose_slam",
            executable="graph_pose_slam_node",
            name="graph_pose_slam_node",
            output="screen",
            parameters=[config],
        ),
    ])
