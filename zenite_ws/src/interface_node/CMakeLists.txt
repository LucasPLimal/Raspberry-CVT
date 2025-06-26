cmake_minimum_required(VERSION 3.8)
project(interface_node)

# Busca por pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_adjust_msgs REQUIRED)

# Configura o executável
add_executable(interface_node src/interface_node.cpp)

# Inclui diretórios necessários
target_include_directories(interface_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Linkagem de dependências
ament_target_dependencies(
  interface_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_adjust_msgs
)

# Instalação
install(TARGETS interface_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
