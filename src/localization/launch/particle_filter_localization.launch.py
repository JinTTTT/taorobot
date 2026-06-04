"""Particle-filter localization against a saved scanned map.

Brings up two things:

1. `map_server` (from graph_pose_slam/map_server.launch.py) — serves a saved
   map on /map, latched. By default this is the SLAM-produced
   graph_pose_slam/maps/slam_map.yaml; override with `map:=/abs/path.yaml`.
2. `particle_filter_localization_node` — global Monte-Carlo localization. It
   waits for /map, spreads particles over the free cells, fuses /odom + /scan,
   and broadcasts the map -> odom correction.

Run the simulation separately first (provides /scan, /odom, base_link TF):

    ros2 launch simulation bringup_simulation.launch.py
    ros2 launch localization particle_filter_localization.launch.py

Do NOT run the SLAM or mapping node at the same time — they also publish /map
and map -> odom, which would collide with map_server + this node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    localization_share = get_package_share_directory("localization")
    config_file = os.path.join(localization_share, "config", "localization.yaml")

    default_map = os.path.join(
        get_package_share_directory("graph_pose_slam"),
        "maps",
        "slam_map.yaml",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Absolute path to the map YAML served on /map for localization.",
    )

    # Serve the saved scanned map on /map (latched). Reuses the self-activating
    # map_server launch from graph_pose_slam so no nav2_lifecycle_manager is needed.
    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("graph_pose_slam"),
                "launch",
                "map_server.launch.py",
            )
        ),
        launch_arguments={"map": LaunchConfiguration("map")}.items(),
    )

    particle_filter = Node(
        package="localization",
        executable="particle_filter_localization_node",
        name="particle_filter_localization_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([
        map_arg,
        map_server,
        particle_filter,
    ])
