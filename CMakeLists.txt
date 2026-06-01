cmake_minimum_required(VERSION 3.16)
project(caddy_ai2_ros2_control_system_steering_driver)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(caddy_ai2_ros2_common REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate parameters from YAML
generate_parameter_library(
  steering_driver_controller_parameters
  src/steering_driver_controller_parameters.yaml
)

# Library definition
add_library(${PROJECT_NAME} SHARED
  src/steering_driver_controller.cpp
  src/canopen_driver.cpp
  src/motor_driver.cpp
  src/encoder_driver.cpp
  src/steering_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    steering_driver_controller_parameters
    caddy_ai2_ros2_common::socketcan_interface
    controller_interface::controller_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${std_msgs_TARGETS}
)

# Pre-existing CAN driver files were written before ros2_control_cmake's strict
# conversion-warning policy; suppress only for those sources.
set_source_files_properties(
  src/canopen_driver.cpp
  src/motor_driver.cpp
  src/encoder_driver.cpp
  src/steering_controller.cpp
  PROPERTIES COMPILE_OPTIONS "-Wno-conversion"
)

# Export the plugin description
pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  steering_driver_controller_parameters
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

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
