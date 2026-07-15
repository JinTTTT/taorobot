import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    """
    Bringup launch file for the simulation system.
    
    Includes spawn_robot.launch.py and adds the ROS-Gazebo bridge.
    This is the main launch file for running the simulation.
    """

    # 1. Include the spawn_robot launch file
    # This starts Gazebo, spawns the robot, and publishes robot state
    pkg_path = get_package_share_directory('simulation')
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'spawn_robot.launch.py')
        )
    )

    # 2. Add the ROS 2 <-> Gazebo bridge
    # This bridges communication between ROS 2 topics and Gazebo topics
    # Without this bridge, ROS 2 commands won't reach Gazebo!
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Command velocity: ROS 2 → Gazebo (control the robot)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # Odometry: Gazebo → ROS 2 (perfect physics odometry, before noise injection)
            '/odom_raw@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # Perfect odom→base_link TF from Gazebo (debug only; noisy TF is published by odometry_noise_node)
            '/tf_gazebo_exact@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Lidar scan: Gazebo → ROS 2 (sensor data)
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # Ground truth world poses: Gazebo -> ROS 2
            '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        output='screen'
    )

    ground_truth_pose_publisher = Node(
        package='simulation',
        executable='ground_truth_pose_publisher',
        output='screen'
    )

    odometry_noise_node = Node(
        package='simulation',
        executable='odometry_noise_node',
        output='screen',
        parameters=[
            ParameterFile(
                os.path.join(pkg_path, 'config', 'odometry_noise.yaml'),
                allow_substs=True
            )
        ]
    )

    # 3. Return all launch actions
    return LaunchDescription([
        spawn_robot_launch,        # Start Gazebo and spawn the robot
        bridge,                    # ROS 2 <-> Gazebo topic bridge
        ground_truth_pose_publisher,
        odometry_noise_node,       # Injects realistic drift into /odom
    ])
