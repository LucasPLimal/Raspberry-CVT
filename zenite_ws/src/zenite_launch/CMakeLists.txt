# Versão mínima do CMake
cmake_minimum_required(VERSION 3.8)

# Nome do projeto (deve bater com o nome no package.xml)
project(zenite_launch)

# Busca por ament_cmake (obrigatório para pacotes ROS 2)
find_package(ament_cmake REQUIRED)

# Instala a pasta 'launch' para que ros2 launch encontre os arquivos
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

# Declara que este é um pacote ament_cmake
ament_package()
