cmake_minimum_required(VERSION 3.8)
project(rokae_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlHand.srv"
  "srv/GetCartesianState.srv"
  "srv/GetVisionTarget.srv"
  "srv/MoveL.srv"
  "srv/MoveLRelative.srv"
  "srv/MoveLTarget.srv"
  "srv/SetVisionOffset.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
