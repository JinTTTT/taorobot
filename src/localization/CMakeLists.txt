cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

add_library(localization_core
  src/geometry_utils.cpp
  src/likelihood_field.cpp
  src/particle_filter.cpp
)

target_include_directories(localization_core PUBLIC
  include
)

ament_target_dependencies(localization_core
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

add_executable(particle_filter_localization_node
  src/particle_filter_localization_node.cpp
)

target_include_directories(particle_filter_localization_node PUBLIC
  include
)

target_link_libraries(particle_filter_localization_node
  localization_core
)

ament_target_dependencies(particle_filter_localization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  localization_core
  particle_filter_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
