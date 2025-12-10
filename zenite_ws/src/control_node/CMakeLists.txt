cmake_minimum_required(VERSION 3.8)
project(control_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(control_node src/control_node.cpp)

ament_target_dependencies(
  control_node
  rclcpp
  geometry_msgs
)

install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
