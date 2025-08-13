cmake_minimum_required(VERSION 3.14)
project(awsim_controller)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(pluginlib REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

include_directories(
  include
)

# Pure Pursuit controller library
add_library(pure_pursuit_controller SHARED
  src/pure_pursuit_controller.cpp
)

ament_target_dependencies(pure_pursuit_controller
  rclcpp geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs autoware_control_msgs autoware_vehicle_msgs pluginlib
)

add_executable(pure_pursuit_node src/pp_node.cpp)
target_link_libraries(pure_pursuit_node pure_pursuit_controller)
ament_target_dependencies(pure_pursuit_node rclcpp)

# Plugin export (optional future use)
pluginlib_export_plugin_description_file(awsim_controller controller_plugins.xml)

install(TARGETS pure_pursuit_controller pure_pursuit_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
