cmake_minimum_required(VERSION 3.16)
project(rokae_bringup)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(environment/rokae_rmw.sh)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
