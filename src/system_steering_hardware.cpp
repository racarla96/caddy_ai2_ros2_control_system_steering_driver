#include "caddy_ai2_ros2_control_system_steering_driver/system_steering_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace caddy_ai2_ros2_control_system_steering_driver
{

// ── Helpers de lectura de parámetros URDF ────────────────────────────────────

static std::string param_str(const hardware_interface::HardwareInfo & info,
                             const std::string & key,
                             const std::string & default_val = "")
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? it->second : default_val;
}

static int param_int(const hardware_interface::HardwareInfo & info,
                     const std::string & key, int default_val = 0)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? std::stoi(it->second) : default_val;
}

static double param_double(const hardware_interface::HardwareInfo & info,
                           const std::string & key, double default_val = 0.0)
{
  auto it = info.hardware_parameters.find(key);
  return (it != info.hardware_parameters.end()) ? std::stod(it->second) : default_val;
}

// ── on_init ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn SystemSteeringHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & info = info_;
  auto logger = rclcpp::get_logger("SystemSteeringHardware");

  // ── CAN y modo de feedback ────────────────────────────────────────────────
  can_interface_name_ = param_str(info, "can_interface_name");
  motor_node_id_      = static_cast<uint8_t>(param_int(info, "motor_node_id", 1));
  feedback_mode_      = param_int(info, "feedback_mode", 1);  // 1 = EPC, 0 = POT
  encoder_node_id_    = static_cast<uint8_t>(param_int(info, "encoder_node_id", 127));
  analog_input_pin_   = static_cast<uint8_t>(param_int(info, "analog_input_pin", 0));

  if (can_interface_name_.empty()) {
    RCLCPP_FATAL(logger, "Parámetro 'can_interface_name' no puede estar vacío");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Rango de dirección ────────────────────────────────────────────────────
  steering_angle_range_ = param_double(info, "steering_angle_range", 0.5747);
  if (steering_angle_range_ <= 0.0) {
    RCLCPP_FATAL(logger, "'steering_angle_range' debe ser > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Calibración del actuador (angle → motor counts) ──────────────────────
  // actuator_encoder_left:  counts del motor en límite izquierdo → +range
  // actuator_encoder_right: counts del motor en límite derecho   → −range
  const double act_left  = param_double(info, "actuator_encoder_left",   99100.0);
  const double act_right = param_double(info, "actuator_encoder_right", -97500.0);

  actuator_zero_           = (act_left + act_right) / 2.0;
  actuator_counts_per_rad_ = (act_left - act_right) / (2.0 * steering_angle_range_);

  // ── Calibración del feedback (raw → rad) ─────────────────────────────────
  if (feedback_mode_ == 0) {
    // POTENCIÓMETRO: left_mv → +range,  right_mv → −range,  zero_mv → 0 rad
    const double pot_left_mv  = param_double(info, "potentiometer_left_mv",  0.0);
    const double pot_right_mv = param_double(info, "potentiometer_right_mv", 5000.0);
    const double pot_zero_mv  = param_double(info, "potentiometer_zero_mv",
                                             (pot_left_mv + pot_right_mv) / 2.0);

    if (std::abs(pot_right_mv - pot_left_mv) < 1.0) {
      RCLCPP_FATAL(logger,
                   "potentiometer_left_mv (%.1f) y potentiometer_right_mv (%.1f) "
                   "deben ser distintos", pot_left_mv, pot_right_mv);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // mV aumenta → ángulo decrece (left_mv < right_mv y left → +range)
    feedback_zero_          = pot_zero_mv;
    feedback_rad_per_unit_  = (-2.0 * steering_angle_range_) /
                              (pot_right_mv - pot_left_mv);

    RCLCPP_INFO(logger, "Feedback: POTENCIÓMETRO  AI%d  "
                        "left=%.0fmV  right=%.0fmV  zero=%.0fmV  "
                        "scale=%.6f rad/mV",
                static_cast<int>(analog_input_pin_ + 1),
                pot_left_mv, pot_right_mv, feedback_zero_,
                feedback_rad_per_unit_);
  } else {
    // ENCODER EPC: left_count → +range,  right_count → −range
    const double epc_left  = param_double(info, "reading_encoder_left",          880.0);
    const double epc_right = param_double(info, "reading_encoder_right",        4520.0);
    const double epc_zero  = param_double(info, "reading_encoder_zero_position", 2650.0);

    if (std::abs(epc_right - epc_left) < 1.0) {
      RCLCPP_FATAL(logger,
                   "reading_encoder_left (%.0f) y reading_encoder_right (%.0f) "
                   "deben ser distintos", epc_left, epc_right);
      return hardware_interface::CallbackReturn::ERROR;
    }

    feedback_zero_         = epc_zero;
    feedback_rad_per_unit_ = (-2.0 * steering_angle_range_) /
                             (epc_right - epc_left);

    const int epc_res = param_int(info, "reading_encoder_resolution", 4096);
    RCLCPP_INFO(logger, "Feedback: ENCODER EPC  node_id=%d  res=%d  "
                        "left=%.0f  right=%.0f  zero=%.0f  "
                        "scale=%.6f rad/count",
                static_cast<int>(encoder_node_id_), epc_res,
                epc_left, epc_right, feedback_zero_,
                feedback_rad_per_unit_);
  }

  RCLCPP_INFO(logger, "Actuador:  left=%.0f  right=%.0f  zero=%.1f  "
                      "scale=%.1f counts/rad",
              act_left, act_right, actuator_zero_, actuator_counts_per_rad_);

  // ── Temporización ─────────────────────────────────────────────────────────
  controller_manager_frequency_hz_ = param_double(info, "controller_manager_frequency_hz", 500.0);
  hardware_sample_frequency_hz_    = param_double(info, "hardware_sample_frequency_hz", 500.0);
  read_multiplicity_  = param_int(info, "read_multiplicity",  1);
  write_multiplicity_ = param_int(info, "write_multiplicity", 1);
  read_offset_        = param_int(info, "read_offset",        0);
  write_offset_       = param_int(info, "write_offset",       1);

  if (controller_manager_frequency_hz_ <= 0.0 || hardware_sample_frequency_hz_ <= 0.0) {
    RCLCPP_FATAL(logger, "Las frecuencias deben ser > 0 Hz");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (read_multiplicity_ <= 0 || write_multiplicity_ <= 0) {
    RCLCPP_FATAL(logger, "Las multiplicidades deben ser > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const int frequency_ratio = std::max(
    1, static_cast<int>(controller_manager_frequency_hz_ / hardware_sample_frequency_hz_));

  effective_read_multiplicity_  = read_multiplicity_  * frequency_ratio;
  effective_write_multiplicity_ = write_multiplicity_ * frequency_ratio;

  read_counter_       = 0;
  write_counter_      = 0;
  motor_count_offset_ = 0;

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(logger,
              "on_init OK — CAN:%s  motor:%d  "
              "CM:%.0fHz  HW:%.0fHz  "
              "read:%d  write:%d  offsets:%d/%d",
              can_interface_name_.c_str(), static_cast<int>(motor_node_id_),
              controller_manager_frequency_hz_, hardware_sample_frequency_hz_,
              effective_read_multiplicity_, effective_write_multiplicity_,
              read_offset_, write_offset_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn SystemSteeringHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("SystemSteeringHardware");
  RCLCPP_INFO(logger, "Configurando hardware (abriendo CAN y arrancando drive)...");

  const uint8_t eid = (feedback_mode_ == 1) ? encoder_node_id_ : 0;
  const uint8_t pin = (feedback_mode_ == 0) ? analog_input_pin_ : 0;

  steering_controller_ = std::make_unique<SteeringController>(
    can_interface_name_, motor_node_id_, eid, pin);

  if (!steering_controller_->init()) {
    RCLCPP_ERROR(logger,
                 "init() falló — verifica la interfaz CAN '%s' y el hardware",
                 can_interface_name_.c_str());
    steering_controller_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger, "on_configure OK — drive en OPERATIONAL");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export interfaces ────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
SystemSteeringHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemSteeringHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

// ── on_activate ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn SystemSteeringHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("SystemSteeringHardware");
  RCLCPP_INFO(logger, "Activando — esperando OPERATION_ENABLED (máx 5 s)...");

  constexpr auto kTimeout     = std::chrono::seconds(5);
  constexpr auto kPollPeriod  = std::chrono::milliseconds(20);
  constexpr double kDt        = 0.02;

  const auto deadline = std::chrono::steady_clock::now() + kTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    steering_controller_->cycle_read(kDt);
    if (steering_controller_->is_motor_enabled()) {
      break;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }

  if (!steering_controller_->is_motor_enabled()) {
    RCLCPP_ERROR(logger,
                 "Timeout: el drive no alcanzó OPERATION_ENABLED en 5 s. "
                 "NMT=%d  DS402=%d",
                 static_cast<int>(steering_controller_->get_motor_nmt_state()),
                 static_cast<int>(steering_controller_->get_motor_drive_state()));
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Inicializar contadores con sus offsets para escalonar read/write
  read_counter_  = effective_read_multiplicity_  - read_offset_;
  write_counter_ = effective_write_multiplicity_ - write_offset_;

  // Inicializar estado con la posición actual
  const double  init_rad            = feedback_to_rad(steering_controller_->get_position());
  const int32_t actual_motor_counts = steering_controller_->get_motor_position();
  motor_count_offset_ = actual_motor_counts - rad_to_motor_counts(init_rad);
  hw_states_[0] = init_rad;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = hw_states_[i];
    }
  }

  RCLCPP_INFO(logger, "on_activate OK — posición inicial: %.4f rad  "
                      "(raw=%d  motor_counts=%d  offset=%d)",
              init_rad,
              steering_controller_->get_position(),
              actual_motor_counts,
              motor_count_offset_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn SystemSteeringHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"),
              "Desactivando — enviando QUICK_STOP...");
  if (steering_controller_) {
    steering_controller_->shutdown();
  }
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "on_deactivate OK");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ─────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn SystemSteeringHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("SystemSteeringHardware"),
               "on_error — apagando drive de forma segura");
  if (steering_controller_) {
    steering_controller_->shutdown();
    steering_controller_.reset();
  }
  read_counter_  = 0;
  write_counter_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type SystemSteeringHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  read_counter_++;
  if (read_counter_ < effective_read_multiplicity_) {
    return hardware_interface::return_type::OK;
  }
  read_counter_ = 0;

  steering_controller_->cycle_read(period.seconds());

  if (steering_controller_->has_fault()) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("SystemSteeringHardware"),
                          *rclcpp::Clock::make_shared(), 1000,
                          "FAULT detectado — NMT=%d  DS402=%d",
                          static_cast<int>(steering_controller_->get_motor_nmt_state()),
                          static_cast<int>(steering_controller_->get_motor_drive_state()));
    return hardware_interface::return_type::ERROR;
  }

  hw_states_[0] = feedback_to_rad(steering_controller_->get_position());
  return hardware_interface::return_type::OK;
}

// ── write ────────────────────────────────────────────────────────────────────

hardware_interface::return_type SystemSteeringHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  write_counter_++;
  if (write_counter_ < effective_write_multiplicity_) {
    return hardware_interface::return_type::OK;
  }
  write_counter_ = 0;

  if (!std::isnan(hw_commands_[0])) {
    const double cmd = std::clamp(hw_commands_[0], -steering_angle_range_, steering_angle_range_);
    steering_controller_->set_target_position(rad_to_motor_counts(cmd) + motor_count_offset_);
    steering_controller_->cycle_write();
  }

  return hardware_interface::return_type::OK;
}

}  // namespace caddy_ai2_ros2_control_system_steering_driver

PLUGINLIB_EXPORT_CLASS(
  caddy_ai2_ros2_control_system_steering_driver::SystemSteeringHardware,
  hardware_interface::SystemInterface)
