cmake_minimum_required(VERSION 3.5)
project(ground_filter)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Qhull REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(ground_filter_node src/ground_filter_node.cpp)
ament_target_dependencies(ground_filter_node rclcpp sensor_msgs PCL pcl_conversions pcl_ros)
target_link_libraries(ground_filter_node ${PCL_LIBRARIES} Qhull::qhull_r)

install(TARGETS ground_filter_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
