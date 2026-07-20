import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """
    Self-contained bringup for the semantic-mapping demo.

    Like bringup_simulation.launch.py but with the OAK-D camera bridge, the
    depth/localization helper nodes, and the semantic_mapping perception + map
    nodes. The world is selectable (default my_world.sdf, the plain simple_room);
    pass world:=office_world.sdf for the object-filled office needed to actually
    see detections. Start this when you want the camera (coverage or semantic
    mapping); use bringup_simulation.launch.py for the plain camera-less sim.
    """

    sim_pkg = get_package_share_directory('simulation')
    urdf_file = os.path.join(sim_pkg, 'urdf', 'my_robot.urdf')

    # `models/objs` holds downloaded Fuel objects (chairs, tables, ...);
    # it must be on the path so `model://<name>` resolves by directory name.
    models_path = os.path.join(sim_pkg, 'models')
    objs_path = os.path.join(models_path, 'objs')
    gazebo_model_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f'{models_path}:{objs_path}'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # World is selectable: default my_world.sdf (the plain simple_room);
    # pass world:=office_world.sdf for the object-filled office (semantic mapping).
    # Both worlds are named "empty", so the ground-truth pose bridge is unchanged.
    world_arg = DeclareLaunchArgument(
        'world', default_value='my_world.sdf',
        description='world file in simulation/worlds (my_world.sdf | office_world.sdf)')
    world_path = PathJoinSubstitution([sim_pkg, 'worlds', LaunchConfiguration('world')])
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_first_robot',
            '-x', '0.0',                   # world origin = spawn spot (office
            '-y', '0.0',                   # world is shifted to match)
            '-z', '0.5'                    # Spawn it 0.5 meters high (so it drops)
        ],
        output='screen'
    )

    # ROS 2 <-> Gazebo bridge, including the OAK-D camera topics needed by
    # the semantic perception node.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Command velocity: ROS 2 -> Gazebo (control the robot)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # Odometry: Gazebo -> ROS 2 (perfect physics odometry, before noise injection)
            '/odom_raw@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # Perfect odom->base_link TF from Gazebo (debug only; noisy TF is published by odometry_noise_node)
            '/tf_gazebo_exact@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Lidar scan: Gazebo -> ROS 2 (sensor data)
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            # Ground truth world poses: Gazebo -> ROS 2
            '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # OAK-D RGB: Gazebo -> ROS 2 (matches real TurtleBot 4 oakd topics)
            '/oakd/rgb/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/oakd/rgb/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # OAK-D depth: gz publishes 32FC1 metres on image_float; depth_to_mm
            # republishes 16UC1 mm on /oakd/stereo/image_raw (real driver format)
            '/oakd/stereo/image_float@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/oakd/stereo/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
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
                os.path.join(sim_pkg, 'config', 'odometry_noise.yaml'),
                allow_substs=True
            )
        ]
    )

    # Convert the sim depth (32FC1 m) to the real OAK-D format (16UC1 mm)
    depth_to_mm = Node(
        package='simulation',
        executable='depth_to_mm',
        output='screen'
    )

    # Sim-only "perfect localizer": publishes map -> odom from ground truth so a
    # stable `map` frame exists without building/loading an occupancy map.
    # Replace with real particle-filter localization later (same map -> odom).
    ground_truth_map_to_odom = Node(
        package='simulation',
        executable='ground_truth_map_to_odom',
        output='screen'
    )

    # Semantic perception + map nodes (from the semantic_mapping package).
    semantic_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('semantic_mapping'),
                'launch', 'semantic_mapping.launch.py'
            )
        )
    )

    return LaunchDescription([
        world_arg,
        gazebo_model_path_env,
        start_gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,                    # ROS 2 <-> Gazebo topic bridge (incl. OAK-D)
        ground_truth_pose_publisher,
        odometry_noise_node,       # Injects realistic drift into /odom
        depth_to_mm,               # sim depth (32FC1 m) -> OAK-D format (16UC1 mm)
        ground_truth_map_to_odom,  # perfect map -> odom so `map` frame exists
        semantic_mapping_launch,   # perception_node + semantic_map_node
    ])
