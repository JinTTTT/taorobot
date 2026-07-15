import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. SETUP: Get the path to our package and the URDF file
    # We need to know *exactly* where your 'simulation' package is installed
    pkg_path = get_package_share_directory('simulation')
    urdf_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf')

    models_path = os.path.join(pkg_path, 'models')
    gazebo_model_path_env = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # 2. CONFIG: Read the URDF file specifically
    # Gazebo needs the actual XML text content, not just the file path
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. ACTION: Start the Gazebo Simulator
    # We use the standard "ros_gz_sim" launch file.
    # '-r' means "run immediately" (don't start paused).
    world_file = os.path.join(pkg_path, 'worlds', 'my_world.sdf')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 4. ACTION: Spawn the Robot
    # This node pushes your URDF text into the running Gazebo instance.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # The topic where the URDF is published
            '-name', 'my_first_robot',     # The name it will have in Gazebo
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'                    # Spawn it 0.5 meters high (so it drops)
        ],
        output='screen'
    )

    # 5. ACTION: Publish Robot State
    # This node publishes the URDF to the topic /robot_description
    # It allows the 'spawn_entity' node (above) to find the robot definition.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 6. RETURN: Send all these actions to ROS to execute
    return LaunchDescription([
        gazebo_model_path_env, #
        start_gazebo,
        robot_state_publisher,
        spawn_entity,
    ])