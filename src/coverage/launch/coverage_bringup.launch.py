"""Coverage bringup: occupancy mapping + planner + controller + goal selection.

Runs on top of the *semantic* simulation (start that separately):

    ros2 launch simulation semantic_bringup_simulation.launch.py
    ros2 launch coverage coverage_bringup.launch.py

Coverage needs the camera, so it layers on semantic_bringup_simulation (office
world with objects, OAK-D camera bridge, ground-truth localizer, and the
semantic_mapping perception + map nodes) -- NOT the plain bringup_simulation,
which has no camera. That sim already provides map -> odom and semantic mapping,
so this launch only adds the pieces it is missing:

  - mapping             builds the live /map from ground-truth pose
  - motion_planning     plans a path to /goal_pose
  - path_follow_control drives the path with pure pursuit -> /cmd_vel
  - coverage            picks nearest-unseen goals and publishes /goal_pose

The goal picker is `coverage` instead of `exploration`: it drives the robot to
where the camera has not looked yet, building the occupancy map and (via the
sim's semantic_mapping nodes) the semantic map in one pass.

The planner and controller take their start pose on /estimated_pose, remapped
to the sim's ground-truth pose since no localization is running.

Full loop: /goal_pose -> /smoothed_planned_path -> /cmd_vel -> robot moves.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    # Build the map from ground-truth pose (override the config's odom default),
    # so a collision can't corrupt the map through drifting wheel odometry. The
    # semantic sim already publishes map -> odom, so we don't start that here.
    mapping = Node(
        package="mapping",
        executable="occupancy_mapper_node",
        name="mapping_node",
        output="screen",
        parameters=[mapping_config, {"use_ground_truth_pose": True}],
    )

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
        mapping,
        motion_planning,
        path_follow_control,
        coverage,
    ])
