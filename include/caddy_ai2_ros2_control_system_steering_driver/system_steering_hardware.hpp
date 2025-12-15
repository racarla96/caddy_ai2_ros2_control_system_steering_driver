#ifndef CADDY_AI2_ROS2_CONTROL_SYSTEM_STEERING_DRIVER__SYSTEM_STEERING_HARDWARE_HPP_
#define CADDY_AI2_ROS2_CONTROL_SYSTEM_STEERING_DRIVER__SYSTEM_STEERING_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"

namespace caddy_ai2_ros2_control_system_steering_driver
{
class SystemSteeringHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SystemSteeringHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parámetros de configuración
  std::string interface_name_;
  double controller_manager_frequency_hz_;
  double hardware_sample_frequency_hz_;
  int read_multiplicity_;
  int write_multiplicity_;
  int read_offset_;
  int write_offset_;

  // Frecuencias calculadas
  int frequency_ratio_;
  int effective_read_multiplicity_;
  int effective_write_multiplicity_;

  // Contadores de ciclos
  int read_counter_;
  int write_counter_;

  // Hardware state
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Steering controller (integración del código CANopen)
  std::unique_ptr<SteeringController> steering_controller_;
  
  // Node IDs para motor y encoder
  uint8_t motor_node_id_;
  uint8_t encoder_node_id_;
  
  // Conversión de unidades
  double counts_per_radian_;  // Factor de conversión encoder counts -> radianes
};

}  // namespace caddy_ai2_ros2_control_system_steering_driver

#endif  // CADDY_AI2_ROS2_CONTROL_SYSTEM_STEERING_DRIVER__SYSTEM_STEERING_HARDWARE_HPP_