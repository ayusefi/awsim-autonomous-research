cmake_minimum_required(VERSION 3.8)
project(awsim_object_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(multi_object_tracker_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

# Add library for Kalman filter
add_library(kalman_filter SHARED src/kalman_filter.cpp)
ament_target_dependencies(kalman_filter "Eigen3")
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Add library for perception pipeline
add_library(perception_pipeline SHARED src/perception_pipeline.cpp)
ament_target_dependencies(perception_pipeline "PCL")
target_include_directories(perception_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(perception_pipeline)

# Add library for object tracker
add_library(object_tracker SHARED src/object_tracker.cpp)
ament_target_dependencies(object_tracker "Eigen3")
target_include_directories(object_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(object_tracker kalman_filter)

# Add library for tracker_node component
add_library(tracker_node SHARED src/tracker_node.cpp)
ament_target_dependencies(tracker_node rclcpp rclcpp_components sensor_msgs geometry_msgs std_msgs visualization_msgs pcl_conversions PCL Eigen3 tf2_ros tf2_sensor_msgs tf2_geometry_msgs multi_object_tracker_msgs)
target_include_directories(tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tracker_node object_tracker perception_pipeline)

# Register the component
rclcpp_components_register_nodes(tracker_node "awsim_object_tracker::TrackerNode")

# Install
install(TARGETS kalman_filter perception_pipeline object_tracker tracker_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  PCL
  Eigen3
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  multi_object_tracker_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()