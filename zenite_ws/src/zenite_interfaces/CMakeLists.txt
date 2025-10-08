cmake_minimum_required(VERSION 3.5)
project(zenite_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Aqui só chamamos rosidl_generate_interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PixelToMeter.srv"
)

ament_package()
