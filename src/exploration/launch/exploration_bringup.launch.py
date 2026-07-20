"""Exploration bringup: live mapping + planner + frontier goal selection.

Runs on top of the simulation (start that separately):

    ros2 launch simulation bringup_simulation.launch.py
    ros2 launch exploration exploration_bringup.launch.py

Brings up:
  - mapping            builds the live /map from /scan + pose
  - motion_planning    plans a path to /goal_pose, using ground truth as the
                       start pose (/estimated_pose remapped to /ground_truth_pose,
                       since we have no localization running)
  - exploration        picks frontier goals and publishes them on /goal_pose

Path comes back on /smoothed_planned_path.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapping_launch = os.path.join(
        get_package_share_directory("mapping"), "launch", "mapping.launch.py")
    planner_config = os.path.join(
        get_package_share_directory("motion_planning"), "config", "motion_planning.yaml")
    exploration_config = os.path.join(
        get_package_share_directory("exploration"), "config", "exploration.yaml")

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch))

    # Feed ground-truth pose in as the planner's start pose (no localization here).
    motion_planning = Node(
        package="motion_planning",
        executable="motion_planning_node",
        name="motion_planning_node",
        output="screen",
        parameters=[planner_config],
        remappings=[("/estimated_pose", "/ground_truth_pose")],
    )

    exploration = Node(
        package="exploration",
        executable="exploration_node",
        name="exploration_node",
        output="screen",
        parameters=[exploration_config],
    )

    return LaunchDescription([mapping, motion_planning, exploration])
