cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(ground_truth_pose_publisher
  src/ground_truth_pose_publisher.cpp
)

ament_target_dependencies(ground_truth_pose_publisher
  rclcpp
  tf2_msgs
  geometry_msgs
  nav_msgs
)

add_executable(odometry_noise_node
  src/odometry_noise_node.cpp
)

ament_target_dependencies(odometry_noise_node
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
)

add_executable(depth_to_mm
  src/depth_to_mm.cpp
)

ament_target_dependencies(depth_to_mm
  rclcpp
  sensor_msgs
)

install(TARGETS
  ground_truth_pose_publisher
  odometry_noise_node
  depth_to_mm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
