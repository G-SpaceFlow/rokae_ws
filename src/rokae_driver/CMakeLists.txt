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
find_package(rokae_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)

# Default layout:
#   /home/niic/aide/hardware/arm/xCoreSDK-v0.7.1.ar_6
#   /home/niic/rokae_ws
# Override it with either:
#   export ROKAE_SDK_ROOT=/path/to/xCoreSDK-v0.7.1.ar_6
# or:
#   colcon build --cmake-args -DROKAE_SDK_ROOT=/path/to/sdk
get_filename_component(
  DEFAULT_ROKAE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../aide/hardware/arm/xCoreSDK-v0.7.1.ar_6"
  ABSOLUTE
)
get_filename_component(
  LEGACY_ROKAE_SDK_ROOT
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

# Existing workspaces can retain the old default in CMakeCache.txt. Migrate
# only that known legacy value; explicit custom SDK paths remain untouched.
if(ROKAE_SDK_ROOT STREQUAL LEGACY_ROKAE_SDK_ROOT AND
   NOT EXISTS "${ROKAE_SDK_ROOT}/include/rokae/robot.h" AND
   EXISTS "${DEFAULT_ROKAE_SDK_ROOT}/include/rokae/robot.h")
  message(STATUS
    "Migrating cached ROKAE_SDK_ROOT from ${LEGACY_ROKAE_SDK_ROOT} "
    "to ${DEFAULT_ROKAE_SDK_ROOT}")
  set(
    ROKAE_SDK_ROOT
    "${DEFAULT_ROKAE_SDK_ROOT}"
    CACHE PATH
    "Absolute path to the xCoreSDK root"
    FORCE
  )
endif()

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

# A caller can override architecture detection and select an exact library:
#   -DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.a
set(
  XCORESDK_LIBRARY
  ""
  CACHE FILEPATH
  "Absolute path to the Linux xCoreSDK library"
)

if(NOT XCORESDK_LIBRARY)
  string(TOLOWER
    "${CMAKE_SYSTEM_PROCESSOR}"
    XCORESDK_SYSTEM_PROCESSOR
  )

  if(XCORESDK_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(XCORESDK_ARCHITECTURE "aarch64")
  elseif(XCORESDK_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(XCORESDK_ARCHITECTURE "x86_64")
  else()
    message(FATAL_ERROR
      "Unsupported target processor '${CMAKE_SYSTEM_PROCESSOR}'. "
      "Select the matching SDK library explicitly with "
      "-DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.a")
  endif()

  set(
    XCORESDK_ARCH_LIBRARY_DIR
    "${ROKAE_SDK_ROOT}/lib/Linux/${XCORESDK_ARCHITECTURE}"
  )

  # Prefer a shared library when the SDK provides both forms. The current
  # xCoreSDK-v0.7.1 package normally provides only the static archive.
  if(EXISTS "${XCORESDK_ARCH_LIBRARY_DIR}/libxCoreSDK.so")
    set(
      XCORESDK_LIBRARY
      "${XCORESDK_ARCH_LIBRARY_DIR}/libxCoreSDK.so"
    )
  elseif(EXISTS "${XCORESDK_ARCH_LIBRARY_DIR}/libxCoreSDK.a")
    set(
      XCORESDK_LIBRARY
      "${XCORESDK_ARCH_LIBRARY_DIR}/libxCoreSDK.a"
    )
  else()
    message(FATAL_ERROR
      "No xCoreSDK library was found for target processor "
      "'${CMAKE_SYSTEM_PROCESSOR}'. Expected libxCoreSDK.so or "
      "libxCoreSDK.a in ${XCORESDK_ARCH_LIBRARY_DIR}. "
      "Override detection with "
      "-DXCORESDK_LIBRARY=/absolute/path/to/libxCoreSDK.a")
  endif()
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

set(
  XMATE_MODEL_LIBRARY
  "${XCORESDK_LIBRARY_DIR}/libxMateModel.a"
)

if(NOT EXISTS "${XMATE_MODEL_LIBRARY}")
  message(FATAL_ERROR
    "The xCoreSDK model library does not exist: ${XMATE_MODEL_LIBRARY}")
endif()

message(STATUS "ROKAE_SDK_ROOT=${ROKAE_SDK_ROOT}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR=${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "XCORESDK_LIBRARY=${XCORESDK_LIBRARY}")

add_library(Rokae::xMateModel STATIC IMPORTED)
set_target_properties(Rokae::xMateModel PROPERTIES
  IMPORTED_LOCATION "${XMATE_MODEL_LIBRARY}"
)

add_library(Rokae::xCoreSDK UNKNOWN IMPORTED)
set_target_properties(Rokae::xCoreSDK PROPERTIES
  IMPORTED_LOCATION "${XCORESDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES
    "${ROKAE_SDK_ROOT}/include;${ROKAE_SDK_ROOT}/external"
  INTERFACE_LINK_LIBRARIES Rokae::xMateModel
)

function(configure_rokae_target target_name)
  target_include_directories(${target_name} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  # ament_target_dependencies() uses the plain target_link_libraries
  # signature on ROS 2 Humble, so this target must use the same signature.
  target_link_libraries(${target_name}
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
  ros_dual_arm_driver
  src/ros_dual_arm_driver.cpp
  src/ros_go_home_service.cpp
  src/ros_hand_service.cpp
  src/ros_moveabsj_action_server.cpp
  src/ros_movel_service.cpp
  src/ros_pos_publisher.cpp
  src/ros_robot_initializer_service.cpp
  src/ros_servoj_subscriber.cpp
)
configure_rokae_target(ros_dual_arm_driver)
target_compile_definitions(ros_dual_arm_driver PRIVATE
  ROKAE_UNIFIED_DRIVER
  XMATEMODEL_LIB_SUPPORTED
)
ament_target_dependencies(ros_dual_arm_driver
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  rokae_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2
)

# ROS 2 searches <install-prefix>/lib/<package-name> for node executables.
install(TARGETS
  ros_dual_arm_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
