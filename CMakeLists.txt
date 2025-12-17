# Copyright (c) 2025 Rafael Carbonell Lázaro (racarla96)
# Licensed under the Creative Commons Attribution 4.0 International License (CC BY 4.0)
# See: https://creativecommons.org/licenses/by/4.0/

cmake_minimum_required(VERSION 3.16)
project(caddy_ai2_ros2_control_system_steering_driver LANGUAGES CXX)

# Establecer el estándar C++20 para todo el proyecto
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  caddy_ai2_ros2_common
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/system_steering_hardware.cpp
  src/canopen_driver.cpp
  src/motor_driver.cpp
  src/encoder_driver.cpp
  src/steering_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Add caddy_ai2_ros2_common headers
target_include_directories(${PROJECT_NAME} PUBLIC
  ${caddy_ai2_ros2_common_INCLUDE_DIRS}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Cause the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "CADDY_AI2_ROS2_CONTROL_SYSTEM_STEERING_DRIVER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface caddy_ai2_ros2_control_system_steering_driver.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY description bringup scripts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install scripts with executable permissions
install(PROGRAMS
  scripts/setup_can_steer_drv.sh 
  scripts/setup_vcan_steer_drv.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()