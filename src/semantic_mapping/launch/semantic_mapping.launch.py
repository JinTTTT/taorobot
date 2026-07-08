import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("semantic_mapping"),
        "config",
        "detection.yaml",
    )

    object_detection_node = Node(
        package="semantic_mapping",
        executable="object_detection_node",
        name="object_detection_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([object_detection_node])
