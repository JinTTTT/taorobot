import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("exploration")
    config = os.path.join(pkg, "config", "exploration.yaml")

    exploration_node = Node(
        package="exploration",
        executable="exploration_node",
        name="exploration_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([exploration_node])
