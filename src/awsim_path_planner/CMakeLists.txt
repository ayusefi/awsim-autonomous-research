cmake_minimum_required(VERSION 3.8)
project(awsim_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Define dependencies list
set(dependencies
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

# Path Planner Node Library
add_library(${PROJECT_NAME}_lib SHARED
  src/path_planner_node.cpp
  src/astar_planner.cpp
  src/rrt_star_planner.cpp
  src/hd_map_manager.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE ${PCL_DEFINITIONS})

ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

# Register components
rclcpp_components_register_nodes(${PROJECT_NAME}_lib "awsim_path_planner::PathPlannerNode")

# Main executable
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install directories
install(DIRECTORY
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${dependencies})

ament_package()
