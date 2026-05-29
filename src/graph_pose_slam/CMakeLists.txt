cmake_minimum_required(VERSION 3.8)
project(graph_pose_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(graph_pose_slam_core
  src/graph_pose_slam.cpp
)

target_include_directories(graph_pose_slam_core PUBLIC
  include
)

ament_target_dependencies(graph_pose_slam_core
  builtin_interfaces
  nav_msgs
  sensor_msgs
)

add_executable(graph_pose_slam_node
  src/graph_pose_slam_node.cpp
)

target_include_directories(graph_pose_slam_node PUBLIC
  include
)

ament_target_dependencies(graph_pose_slam_node
  nav_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(graph_pose_slam_node
  graph_pose_slam_core
)

install(TARGETS
  graph_pose_slam_core
  graph_pose_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
