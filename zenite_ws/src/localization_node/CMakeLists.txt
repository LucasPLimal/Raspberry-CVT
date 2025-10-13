cmake_minimum_required(VERSION 3.22)
project(localization_node)

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(zenite_utils REQUIRED)

# Inclui diretórios
include_directories(
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${zenite_utils_INCLUDE_DIRS}  # pega include da biblioteca
)

# Executável
add_executable(localization_node src/localization_node.cpp)

# Link das bibliotecas
ament_target_dependencies(localization_node
  rclcpp
  sensor_msgs
  cv_bridge
  yaml-cpp
  zenite_utils  # Linka a biblioteca utilitária
  )

target_link_libraries(localization_node
  ${OpenCV_LIBS}      # OpenCV
)

# Instalação
install(TARGETS localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testes (opcional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
