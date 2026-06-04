"""Serve a saved SLAM map on /map for RViz / localization.

Loads a stored map (PGM + YAML produced by `map_saver_cli`) with
nav2_map_server's `map_server` and publishes it on /map (latched).

`map_server` is a lifecycle node, so it does nothing until it is
configured and activated. nav2_lifecycle_manager is NOT installed in this
workspace, so instead of relying on it we drive the transitions straight
from the launch file:

  1. emit CONFIGURE as soon as the node starts
  2. when it reports it reached the `inactive` state, emit ACTIVATE

After activation /map carries the saved grid (transient_local / latched),
so a late-starting RViz still receives it. View with Fixed Frame = "map"
and a Map display on /map.

Usage:
    ros2 launch graph_pose_slam map_server.launch.py
    ros2 launch graph_pose_slam map_server.launch.py map:=/abs/path/to/other.yaml
"""

import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    default_map = os.path.join(
        get_package_share_directory("graph_pose_slam"),
        "maps",
        "slam_map.yaml",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Absolute path to the map YAML file to serve.",
    )

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map"),
            # No nav2_lifecycle_manager runs here, so disable the bond the node
            # would otherwise try (and fail) to create to one. 0.0 = bond off.
            "bond_heartbeat_period": 0.0,
        }],
    )

    # Step 1: configure the node as soon as it launches.
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Step 2: once it is 'inactive' (configured), activate it.
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        map_arg,
        map_server,
        activate,
        configure,
    ])
