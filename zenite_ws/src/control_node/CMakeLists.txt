cmake_minimum_required(VERSION 3.8)
project(control_node)

# Busca por pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs)

# Configura o executável
add_executable(control_node src/control_node.cpp)

# Inclui diretórios necessários
target_include_directories(control_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Linkagem de dependências
ament_target_dependencies(
  control_node
  rclcpp
  geometry_msgs
)

# Instalação
install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
