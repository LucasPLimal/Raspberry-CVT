cmake_minimum_required(VERSION 3.8)
project(acquisition_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_adjust_msgs REQUIRED)

add_executable(acquisition_node src/acquisition_node.cpp)

ament_target_dependencies(
  acquisition_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
  image_adjust_msgs
)

target_include_directories(acquisition_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  acquisition_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()