cmake_minimum_required(VERSION 3.16)
project(rokae_driver LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)

# Default layout:
#   /home/niic/cxl/xCoreSDK-v0.7.1.ar_6
#   /home/niic/rokae_ws
# Override it with either:
#   export ROKAE_SDK_ROOT=/path/to/xCoreSDK-v0.7.1.ar_6
# or:
#   colcon build --cmake-args -DROKAE_SDK_ROOT=/path/to/sdk
get_filename_component(
  DEFAULT_ROKAE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../cxl/xCoreSDK-v0.7.1.ar_6"
  ABSOLUTE
)

if(DEFINED ENV{ROKAE_SDK_ROOT} AND
   NOT "$ENV{ROKAE_SDK_ROOT}" STREQUAL "")
  set(DEFAULT_ROKAE_SDK_ROOT "$ENV{ROKAE_SDK_ROOT}")
endif()

set(
  ROKAE_SDK_ROOT
  "${DEFAULT_ROKAE_SDK_ROOT}"
  CACHE PATH
  "Absolute path to the xCoreSDK root"
)

if(NOT EXISTS "${ROKAE_SDK_ROOT}/include/rokae/robot.h")
  message(FATAL_ERROR
    "Cannot find ${ROKAE_SDK_ROOT}/include/rokae/robot.h. "
    "Set ROKAE_SDK_ROOT to the xCoreSDK root directory.")
endif()

if(NOT EXISTS "${ROKAE_SDK_ROOT}/external")
  message(FATAL_ERROR
    "Cannot find ${ROKAE_SDK_ROOT}/external. "
    "The SDK Eigen headers are required.")
endif()

# A caller can select an exact library when the SDK contains several
# operating-system or architecture variants:
#   -DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.so
set(
  XCORESDK_LIBRARY
  ""
  CACHE FILEPATH
  "Absolute path to the Linux xCoreSDK library"
)

if(NOT XCORESDK_LIBRARY)
  file(GLOB_RECURSE XCORESDK_LIBRARY_CANDIDATES
    LIST_DIRECTORIES false
    "${ROKAE_SDK_ROOT}/lib/libxCoreSDK.so"
    "${ROKAE_SDK_ROOT}/lib/libxCoreSDK.a"
  )

  list(LENGTH XCORESDK_LIBRARY_CANDIDATES XCORESDK_LIBRARY_COUNT)
  if(XCORESDK_LIBRARY_COUNT EQUAL 0)
    message(FATAL_ERROR
      "No libxCoreSDK.so or libxCoreSDK.a was found below "
      "${ROKAE_SDK_ROOT}/lib.")
  elseif(XCORESDK_LIBRARY_COUNT GREATER 1)
    message(FATAL_ERROR
      "Multiple xCoreSDK libraries were found: "
      "${XCORESDK_LIBRARY_CANDIDATES}. Select the Linux library with "
      "-DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.so")
  endif()

  list(GET XCORESDK_LIBRARY_CANDIDATES 0 XCORESDK_LIBRARY)
endif()

if(NOT EXISTS "${XCORESDK_LIBRARY}")
  message(FATAL_ERROR
    "XCORESDK_LIBRARY does not exist: ${XCORESDK_LIBRARY}")
endif()

get_filename_component(
  XCORESDK_LIBRARY_DIR
  "${XCORESDK_LIBRARY}"
  DIRECTORY
)

message(STATUS "ROKAE_SDK_ROOT=${ROKAE_SDK_ROOT}")
message(STATUS "XCORESDK_LIBRARY=${XCORESDK_LIBRARY}")

add_library(Rokae::xCoreSDK UNKNOWN IMPORTED)
set_target_properties(Rokae::xCoreSDK PROPERTIES
  IMPORTED_LOCATION "${XCORESDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ROKAE_SDK_ROOT}/include;${ROKAE_SDK_ROOT}/external"
)

function(configure_rokae_target target_name)
  target_link_libraries(${target_name} PRIVATE
    Rokae::xCoreSDK
    Threads::Threads
  )

  # Keep the vendor library outside the ROS workspace, but make the installed
  # ROS executables able to locate it at runtime.
  set_target_properties(${target_name} PROPERTIES
    BUILD_RPATH "${XCORESDK_LIBRARY_DIR}"
    INSTALL_RPATH "${XCORESDK_LIBRARY_DIR}"
  )
endfunction()

add_executable(
  ros_moveabsj_action_server
  src/ros_moveabsj_action_server.cpp
)
configure_rokae_target(ros_moveabsj_action_server)
ament_target_dependencies(ros_moveabsj_action_server
  control_msgs
  rclcpp
  rclcpp_action
)

add_executable(
  ros_pos_publisher
  src/ros_pos_publisher.cpp
)
configure_rokae_target(ros_pos_publisher)
ament_target_dependencies(ros_pos_publisher
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
)

# ROS 2 searches <install-prefix>/lib/<package-name> for node executables.
install(TARGETS
  ros_moveabsj_action_server
  ros_pos_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
