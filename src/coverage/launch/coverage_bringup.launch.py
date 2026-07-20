"""Coverage bringup: live mapping + semantic mapping + planner + controller + goal selection.

Runs on top of the simulation (start that separately):

    ros2 launch simulation bringup_simulation.launch.py
    ros2 launch coverage coverage_bringup.launch.py

Same base stack as exploration_bringup, with two differences: the semantic
mapping pipeline runs (so the camera builds the object map), and the goal
picker is `coverage` instead of `exploration` -- it drives the robot to where
the camera has not looked yet, building the occupancy map and the semantic map
in one pass.

Everything runs on the sim's ground-truth pose, so the whole stack is
consistent and a stray bump can't smear the map:
  - ground_truth_map_to_odom  exact map -> odom (the sim's "perfect localizer")
  - mapping                   builds the live /map, using ground-truth pose
  - semantic_mapping          camera -> persistent object map
  - motion_planning           plans a path to /goal_pose
  - path_follow_control       drives the path with pure pursuit -> /cmd_vel
  - coverage                  picks nearest-unseen goals and publishes /goal_pose

The planner and controller take their start pose on /estimated_pose, remapped
to the ground-truth pose since no localization is running.

Full loop: /goal_pose -> /smoothed_planned_path -> /cmd_vel -> robot moves.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mapping_config = os.path.join(
        get_package_share_directory("mapping"), "config", "mapping.yaml")
    planner_config = os.path.join(
        get_package_share_directory("motion_planning"), "config", "motion_planning.yaml")
    controller_config = os.path.join(
        get_package_share_directory("path_follow_control"), "config", "path_follow_control.yaml")
    coverage_config = os.path.join(
        get_package_share_directory("coverage"), "config", "coverage.yaml")

    # Sim-only perfect localizer: exact map -> odom from ground truth, so the
    # `map` frame is drift-free. Replaces mapping.launch's static-identity one.
    ground_truth_map_to_odom = Node(
        package="simulation",
        executable="ground_truth_map_to_odom",
        name="ground_truth_map_to_odom",
        output="screen",
    )

    # Build the map from ground-truth pose (override the config's odom default),
    # so a collision can't corrupt the map through drifting wheel odometry.
    mapping = Node(
        package="mapping",
        executable="occupancy_mapper_node",
        name="mapping_node",
        output="screen",
        parameters=[mapping_config, {"use_ground_truth_pose": True}],
    )

    # Camera -> persistent semantic object map (perception + semantic_map nodes).
    semantic_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("semantic_mapping"),
            "launch", "semantic_mapping.launch.py")))

    # Planner and controller take their start pose on /estimated_pose; with no
    # localization running, feed in the sim's ground-truth pose.
    ground_truth_start = [("/estimated_pose", "/ground_truth_pose")]

    motion_planning = Node(
        package="motion_planning",
        executable="motion_planning_node",
        name="motion_planning_node",
        output="screen",
        parameters=[planner_config],
        remappings=ground_truth_start,
    )

    path_follow_control = Node(
        package="path_follow_control",
        executable="path_follow_control_node",
        name="path_follow_control_node",
        output="screen",
        parameters=[controller_config],
        remappings=ground_truth_start,
    )

    coverage = Node(
        package="coverage",
        executable="coverage_node",
        name="coverage_node",
        output="screen",
        parameters=[coverage_config],
    )

    return LaunchDescription([
        ground_truth_map_to_odom,
        mapping,
        semantic_mapping,
        motion_planning,
        path_follow_control,
        coverage,
    ])
