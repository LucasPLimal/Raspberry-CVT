cmake_minimum_required(VERSION 3.8)
project(transmission_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Encontrar a biblioteca BlueZ
find_package(PkgConfig REQUIRED)
pkg_check_modules(BLUETOOTH REQUIRED bluez)

# Criar o executável bluetooth_node
add_executable(bluetooth_node
  src/bluetooth_node.cpp
  src/bluetoothAction.cpp  # BluetoothAction está dentro do bluetooth_node
)

# Definir dependências
ament_target_dependencies(bluetooth_node rclcpp std_msgs sensor_msgs)

# Incluir diretórios
target_include_directories(bluetooth_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Vincular bibliotecas externas
target_link_libraries(bluetooth_node ${BLUETOOTH_LIBRARIES})

# Gerar mensagens customizadas
find_package(rosidl_default_generators REQUIRED)
set(MSG_FILES "msg/CustomData.msg")
rosidl_generate_interfaces(${PROJECT_NAME} ${MSG_FILES} DEPENDENCIES std_msgs)

# Vincular mensagens ao executável
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bluetooth_node "${cpp_typesupport_target}")

# Instalar executável
install(TARGETS bluetooth_node DESTINATION lib/${PROJECT_NAME})

ament_package()

