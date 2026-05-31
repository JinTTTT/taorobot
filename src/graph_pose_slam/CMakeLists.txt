cmake_minimum_required(VERSION 3.8)
project(graph_pose_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(g2o REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# Core SLAM library (no ROS node, easy to unit test)
add_library(graph_pose_slam_core
  src/graph_pose_slam.cpp
  src/correlative_scan_matcher.cpp
  src/pose_graph.cpp
  src/pose_graph_optimizer.cpp
)

target_include_directories(graph_pose_slam_core PUBLIC
  include
)

ament_target_dependencies(graph_pose_slam_core
  builtin_interfaces
  rclcpp
  sensor_msgs
)

target_link_libraries(graph_pose_slam_core
  g2o::core
  g2o::types_slam2d
  g2o::solver_eigen
)

# ROS node (subscribes to /odom and /scan, runs the SLAM core)
add_executable(graph_pose_slam_node
  src/graph_pose_slam_node.cpp
  src/utils.cpp
)

target_include_directories(graph_pose_slam_node PUBLIC
  include
)

ament_target_dependencies(graph_pose_slam_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(graph_pose_slam_node
  graph_pose_slam_core
)

# Standalone CSM test script — run with: ros2 run graph_pose_slam test_correlative_scan_matcher
add_executable(test_correlative_scan_matcher
  test/test_correlative_scan_matcher.cpp
)

target_include_directories(test_correlative_scan_matcher PUBLIC
  include
)

ament_target_dependencies(test_correlative_scan_matcher
  ament_index_cpp
  sensor_msgs
)

target_link_libraries(test_correlative_scan_matcher
  graph_pose_slam_core
  yaml-cpp
)

# ICP vs CSM comparison — compiles icp_scan_matcher.cpp directly (not in core lib)
add_executable(test_matcher_comparison
  test/test_matcher_comparison.cpp
  src/icp_scan_matcher.cpp
)

target_include_directories(test_matcher_comparison PUBLIC include)

ament_target_dependencies(test_matcher_comparison
  ament_index_cpp
  sensor_msgs
)

target_link_libraries(test_matcher_comparison
  graph_pose_slam_core
  yaml-cpp
)

install(TARGETS
  graph_pose_slam_core
  graph_pose_slam_node
  test_correlative_scan_matcher
  test_matcher_comparison
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
