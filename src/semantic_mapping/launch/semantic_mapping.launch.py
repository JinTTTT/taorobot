import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("semantic_mapping")
    perception_config = os.path.join(pkg, "config", "perception.yaml")
    map_config = os.path.join(pkg, "config", "semantic_map.yaml")

    perception_node = Node(
        package="semantic_mapping",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[perception_config],
    )

    semantic_map_node = Node(
        package="semantic_mapping",
        executable="semantic_map_node",
        name="semantic_map_node",
        output="screen",
        parameters=[map_config],
    )

    return LaunchDescription([perception_node, semantic_map_node])
