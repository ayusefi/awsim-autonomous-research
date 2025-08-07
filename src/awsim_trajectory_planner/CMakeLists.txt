cmake_minimum_required(VERSION 3.8)
project(awsim_trajectory_planner)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# Include directories
include_directories(include)
# Create trajectory planner library
add_library(${PROJECT_NAME} SHARED
  src/trajectory_planner_node.cpp
  src/trajectory_generator.cpp
  src/obstacle_detector.cpp
  src/cost_evaluator.cpp
)
# Let ament_target_dependencies handle linking
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  rclcpp_components
  tf2_sensor_msgs
)
# Create executable
add_executable(trajectory_planner_node src/main.cpp)
target_link_libraries(trajectory_planner_node 
  ${PROJECT_NAME}
)
# Let ament_target_dependencies handle linking for the executable too
ament_target_dependencies(trajectory_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  rclcpp_components
  tf2_sensor_msgs
)
# Export dependencies
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  rclcpp_components
  tf2_sensor_msgs
)
# Install targets with export
install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include/
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
# Register component
rclcpp_components_register_nodes(${PROJECT_NAME} "awsim_trajectory_planner::TrajectoryPlannerNode")
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
ament_package()