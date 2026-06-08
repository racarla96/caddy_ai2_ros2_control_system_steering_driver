#pragma once

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

// ============================================================================
// SystemSteeringHardware — hardware interface ROS2 control para la dirección
//
// Lee todos los parámetros de calibración del URDF y expone una única
// interfaz position en radianes para el steering_joint.
//
// Calibración motor (command):
//   counts = actuator_zero_ + angle_rad * actuator_counts_per_rad_
//
// Calibración feedback (state):
//   angle_rad = (raw - feedback_zero_) * feedback_rad_per_unit_
//
//   Feedback EPC: raw = encoder counts, zero = reading_encoder_zero_position
//   Feedback POT: raw = mV del potenciómetro,  zero = potentiometer_zero_mv
// ============================================================================
class SystemSteeringHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SystemSteeringHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── Conversión feedback → ángulo (rad) ───────────────────────────────────
  double feedback_to_rad(int32_t raw) const
  {
    return static_cast<double>(raw - feedback_zero_) * feedback_rad_per_unit_;
  }

  // ── Conversión ángulo (rad) → counts motor ───────────────────────────────
  int32_t rad_to_motor_counts(double rad) const
  {
    return static_cast<int32_t>(std::round(actuator_zero_ + rad * actuator_counts_per_rad_));
  }

  // ── Parámetros CAN y modo de feedback ────────────────────────────────────
  std::string can_interface_name_;
  uint8_t     motor_node_id_;
  int         feedback_mode_;         // 0 = potenciómetro, 1 = encoder EPC
  uint8_t     encoder_node_id_;       // solo si feedback_mode = 1
  uint8_t     analog_input_pin_;      // 0-based (0=AI1,1=AI2,2=AI3), solo si mode = 0

  // ── Parámetros de calibración del actuador (command: rad → counts motor) ─
  // actuator_encoder_left:  counts del encoder del motor cuando la dirección
  //                         está en su límite izquierdo (ángulo = +steering_angle_range)
  // actuator_encoder_right: counts del encoder del motor en límite derecho
  //                         (ángulo = −steering_angle_range)
  double actuator_zero_;              // (left + right) / 2
  double actuator_counts_per_rad_;    // (left − right) / (2 · range)

  // ── Parámetros de calibración del feedback (state: raw → rad) ────────────
  // EPC: reading_encoder_left  = counts en límite izquierdo → +range
  //      reading_encoder_right = counts en límite derecho   → −range
  //      reading_encoder_zero_position = counts en posición cero
  // POT: potentiometer_left_mv  = mV en límite izquierdo → +range
  //      potentiometer_right_mv = mV en límite derecho   → −range
  //      potentiometer_zero_mv  = mV en posición cero
  double feedback_zero_;              // raw (counts o mV) que corresponde a 0 rad
  double feedback_rad_per_unit_;      // rad por count (EPC) o rad por mV (POT)

  double steering_angle_range_;       // rad, rango máximo de ±giro

  // ── Parámetros de temporización ──────────────────────────────────────────
  double controller_manager_frequency_hz_;
  double hardware_sample_frequency_hz_;
  int    read_multiplicity_;
  int    write_multiplicity_;
  int    read_offset_;
  int    write_offset_;

  // ── Derivados ─────────────────────────────────────────────────────────────
  int effective_read_multiplicity_;
  int effective_write_multiplicity_;

  // ── Contadores de ciclo ───────────────────────────────────────────────────
  int read_counter_;
  int write_counter_;

  // ── Buffers del hardware interface ───────────────────────────────────────
  std::vector<double> hw_states_;    // [0] = posición (rad)
  std::vector<double> hw_commands_;  // [0] = posición objetivo (rad)

  // ── Driver ────────────────────────────────────────────────────────────────
  std::unique_ptr<SteeringController> steering_controller_;
};

}  // namespace caddy_ai2_ros2_control_system_steering_driver
