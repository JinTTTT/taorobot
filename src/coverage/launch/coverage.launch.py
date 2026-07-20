import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("coverage")
    config = os.path.join(pkg, "config", "coverage.yaml")

    coverage_node = Node(
        package="coverage",
        executable="coverage_node",
        name="coverage_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([coverage_node])
