cmake_minimum_required(VERSION 3.8)
project(control_node)

# Encontrar pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gerar mensagens customizadas
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/CustomData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES std_msgs
)

# Adicionar executável (renomeado para evitar conflito)
add_executable(control_node_exec src/control_node_exec.cpp)
ament_target_dependencies(control_node_exec rclcpp std_msgs)

# Incluir diretórios
target_include_directories(control_node_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Vincular mensagens ao executável
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_node_exec "${cpp_typesupport_target}")

# Instalar o executável
install(TARGETS control_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

