cmake_minimum_required(VERSION 3.8)
project(localization_node)

# Busca por pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zenite_interfaces REQUIRED)  # nosso pacote de srv


# Configura o executável
add_executable(localization_node src/localization_node.cpp)

# Inclui diretórios necessários
target_include_directories(localization_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Linkagem de dependências
ament_target_dependencies(
  localization_node
  rclcpp
  OpenCV
  std_msgs
  sensor_msgs
  cv_bridge
  zenite_interfaces
)

# Instalação
install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
