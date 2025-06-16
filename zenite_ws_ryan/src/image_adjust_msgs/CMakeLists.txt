cmake_minimum_required(VERSION 3.8)
project(image_adjust_msgs)

# Busca pacotes essenciais
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Gera as mensagens
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ImageParams.msg"
  DEPENDENCIES std_msgs
)

# Exporta as dependências corretamente
ament_export_dependencies(rosidl_default_runtime)
ament_package()
