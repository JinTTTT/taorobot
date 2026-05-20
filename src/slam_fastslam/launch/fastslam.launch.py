from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    fastslam_config = os.path.join(
        get_package_share_directory("slam_fastslam"), "config", "fastslam.yaml")
    mapping_config = os.path.join(
        get_package_share_directory("mapping"), "config", "mapping.yaml")

    return LaunchDescription([
        Node(
            package="slam_fastslam",
            executable="fastslam_node",
            name="fastslam_node",
            output="screen",
            parameters=[mapping_config, fastslam_config],
        ),
    ])
