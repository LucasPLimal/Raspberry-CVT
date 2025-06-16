cmake_minimum_required(VERSION 3.8)
project(acquisition_node)

# Busca por dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_adjust_msgs REQUIRED) 

# Configura o executável
add_executable(acquisition_node src/acquisition_node.cpp)
target_include_directories(acquisition_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# Linkagem
ament_target_dependencies(acquisition_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_adjust_msgs
)

# Instalação
install(TARGETS acquisition_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
