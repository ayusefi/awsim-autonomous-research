cmake_minimum_required(VERSION 3.8)
project(awsim_sensor_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Add executable
add_executable(sensor_logger_node src/sensor_logger_node.cpp)
ament_target_dependencies(sensor_logger_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  nav_msgs
  message_filters
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
  autoware_vehicle_msgs)

# Link PCL libraries
target_link_libraries(sensor_logger_node ${PCL_LIBRARIES})
target_include_directories(sensor_logger_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(sensor_logger_node PRIVATE ${PCL_DEFINITIONS})

# Install targets
install(TARGETS
  sensor_logger_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install scripts
install(PROGRAMS scripts/analyze_sensor_data.py
  DESTINATION lib/${PROJECT_NAME})

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
