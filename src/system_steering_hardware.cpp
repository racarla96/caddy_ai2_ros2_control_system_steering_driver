#include "caddy_ai2_ros2_control_system_steering_driver/system_steering_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/canopen_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/motor_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/encoder_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"

namespace caddy_ai2_ros2_control_system_steering_driver
{
hardware_interface::CallbackReturn SystemSteeringHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Llamar al on_init del padre para inicializar info_
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Leer parámetros de configuración
  interface_name_ = info_.hardware_parameters["interface_name"];
  controller_manager_frequency_hz_ = std::stod(info_.hardware_parameters["controller_manager_frequency_hz"]);
  hardware_sample_frequency_hz_ = std::stod(info_.hardware_parameters["hardware_sample_frequency_hz"]);
  read_multiplicity_ = std::stoi(info_.hardware_parameters["read_multiplicity"]);
  write_multiplicity_ = std::stoi(info_.hardware_parameters["write_multiplicity"]);
  read_offset_ = std::stoi(info_.hardware_parameters["read_offset"]);
  write_offset_ = std::stoi(info_.hardware_parameters["write_offset"]);

  // Parámetros CANopen
  motor_node_id_ = std::stoi(info_.hardware_parameters.count("motor_node_id") ? 
                              info_.hardware_parameters["motor_node_id"] : "1");
  encoder_node_id_ = std::stoi(info_.hardware_parameters.count("encoder_node_id") ? 
                                info_.hardware_parameters["encoder_node_id"] : "127");
  counts_per_radian_ = std::stod(info_.hardware_parameters.count("counts_per_radian") ? 
                                  info_.hardware_parameters["counts_per_radian"] : "100000.0");

  // Validar parámetros
  if (hardware_sample_frequency_hz_ <= 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("SystemSteeringHardware"),
      "La frecuencia del hardware debe ser mayor que 0 Hz");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (controller_manager_frequency_hz_ <= 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("SystemSteeringHardware"),
      "La frecuencia del controller manager debe ser mayor que 0 Hz");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (read_multiplicity_ <= 0 || write_multiplicity_ <= 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("SystemSteeringHardware"),
      "Las multiplicidades deben ser mayores que 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (read_offset_ < 0 || write_offset_ < 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("SystemSteeringHardware"),
      "Los offsets no pueden ser negativos");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Calcular ratio de frecuencias
  frequency_ratio_ = static_cast<int>(controller_manager_frequency_hz_ / hardware_sample_frequency_hz_);
  if (frequency_ratio_ < 1)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SystemSteeringHardware"),
      "La frecuencia del controller manager (%.2f Hz) es menor que la frecuencia del hardware (%.2f Hz). "
      "Usando ratio = 1",
      controller_manager_frequency_hz_, hardware_sample_frequency_hz_);
    frequency_ratio_ = 1;
  }

  // Calcular multiplicidades efectivas
  effective_read_multiplicity_ = read_multiplicity_ * frequency_ratio_;
  effective_write_multiplicity_ = write_multiplicity_ * frequency_ratio_;

  // Inicializar contadores
  read_counter_ = 0;
  write_counter_ = 0;

  // Inicializar vectores de estado
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemSteeringHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "=== Configuración del Hardware de Dirección ===");
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Interfaz CAN: %s", interface_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Motor Node ID: %d", motor_node_id_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Encoder Node ID: %d", encoder_node_id_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Counts per radian: %.2f", counts_per_radian_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Frecuencia controller_manager: %.2f Hz", controller_manager_frequency_hz_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Frecuencia del hardware: %.2f Hz", hardware_sample_frequency_hz_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Ratio de frecuencias: %d", frequency_ratio_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Multiplicidad base - Lectura: %d, Escritura: %d", 
              read_multiplicity_, write_multiplicity_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Multiplicidad efectiva - Lectura: %d, Escritura: %d", 
              effective_read_multiplicity_, effective_write_multiplicity_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Offsets - Lectura: %d ciclos, Escritura: %d ciclos", 
              read_offset_, write_offset_);
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Frecuencia real - Lectura: %.2f Hz, Escritura: %.2f Hz",
              controller_manager_frequency_hz_ / effective_read_multiplicity_,
              controller_manager_frequency_hz_ / effective_write_multiplicity_);

  // Crear el controlador de dirección
  steering_controller_ = std::make_unique<SteeringController>(
    interface_name_, motor_node_id_, encoder_node_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SystemSteeringHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SystemSteeringHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SystemSteeringHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Activando hardware...");

  // Inicializar el controlador de dirección
  if (!steering_controller_->init())
  {
    RCLCPP_FATAL(rclcpp::get_logger("SystemSteeringHardware"), 
                 "Error al inicializar el controlador de dirección");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Inicializar contadores con offsets
  read_counter_ = effective_read_multiplicity_ - read_offset_;
  write_counter_ = effective_write_multiplicity_ - write_offset_;

  // Inicializar comandos con posición actual
  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0.0;
    }
    hw_states_[i] = hw_commands_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Hardware activado correctamente");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SystemSteeringHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemSteeringHardware"), "Desactivando hardware...");
  
  if (steering_controller_)
  {
    steering_controller_->shutdown();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SystemSteeringHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Incrementar contador de lectura
  read_counter_++;

  // Verificar si es momento de leer
  if (read_counter_ >= effective_read_multiplicity_)
  {
    read_counter_ = 0;

    // Procesar mensajes CAN entrantes y actualizar estado
    steering_controller_->step();

    // Leer posición del encoder absoluto (en counts)
    int32_t position_counts = steering_controller_->getAbsoluteEncoderPosition();
    
    // Convertir a radianes
    hw_states_[0] = static_cast<double>(position_counts) / counts_per_radian_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SystemSteeringHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Incrementar contador de escritura
  write_counter_++;

  // Verificar si es momento de escribir
  if (write_counter_ >= effective_write_multiplicity_)
  {
    write_counter_ = 0;

    // Convertir comando de radianes a counts
    int32_t target_counts = static_cast<int32_t>(hw_commands_[0] * counts_per_radian_);
    
    // Enviar comando al motor
    steering_controller_->setTargetSteeringPosition(target_counts);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace caddy_ai2_ros2_control_system_steering_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  caddy_ai2_ros2_control_system_steering_driver::SystemSteeringHardware, hardware_interface::SystemInterface)