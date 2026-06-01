#include "caddy_ai2_ros2_control_system_steering_driver/steering_driver_controller.hpp"

#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace caddy_ai2_ros2_control_system_steering_driver
{

SteeringDriverController::SteeringDriverController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn SteeringDriverController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<steering_driver_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringDriverController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    params_ = param_listener_->get_params();

    // Instantiate the CAN/CANopen steering controller
    steering_controller_ = std::make_unique<SteeringController>(
      params_.interface_name,
      static_cast<uint8_t>(params_.motor_node_id),
      static_cast<uint8_t>(params_.encoder_node_id));

    // Compute effective multiplicity (mirrors the hardware interface logic)
    frequency_ratio_ = static_cast<int>(
      params_.controller_manager_frequency_hz / params_.hardware_sample_frequency_hz);
    if (frequency_ratio_ < 1)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "controller_manager_frequency_hz (%.2f) < hardware_sample_frequency_hz (%.2f), using ratio=1",
        params_.controller_manager_frequency_hz, params_.hardware_sample_frequency_hz);
      frequency_ratio_ = 1;
    }
    effective_read_multiplicity_  = static_cast<int>(params_.read_multiplicity)  * frequency_ratio_;
    effective_write_multiplicity_ = static_cast<int>(params_.write_multiplicity) * frequency_ratio_;

    RCLCPP_INFO(get_node()->get_logger(), "CAN interface    : %s",  params_.interface_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor node ID    : %ld", params_.motor_node_id);
    RCLCPP_INFO(get_node()->get_logger(), "Encoder node ID  : %ld", params_.encoder_node_id);
    RCLCPP_INFO(get_node()->get_logger(), "Counts/rad       : %.4f", params_.counts_per_radian);
    RCLCPP_INFO(get_node()->get_logger(), "Freq ratio       : %d",  frequency_ratio_);
    RCLCPP_INFO(get_node()->get_logger(), "Eff read mult    : %d",  effective_read_multiplicity_);
    RCLCPP_INFO(get_node()->get_logger(), "Eff write mult   : %d",  effective_write_multiplicity_);

    auto node = get_node();

    // Subscriber: receive target steering position [rad] from an upstream controller or operator
    target_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      "~/reference", rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        target_position_rad_.store(msg->data, std::memory_order_relaxed);
      });

    // Publisher: broadcast measured encoder position [rad] to observers
    encoder_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "~/state", rclcpp::SensorDataQoS());

    // Temporary debug publishers: raw encoder counts
    abs_encoder_counts_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "~/debug/abs_encoder_counts", rclcpp::SensorDataQoS());
    inc_encoder_counts_pub_ = node->create_publisher<std_msgs::msg::Float64>(
      "~/debug/inc_encoder_counts", rclcpp::SensorDataQoS());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SteeringDriverController::command_interface_configuration() const
{
  // This controller communicates directly with the CAN hardware; no ros2_control
  // command interfaces are required.
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
SteeringDriverController::state_interface_configuration() const
{
  // Encoder position is read directly via SteeringController over CAN.
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn SteeringDriverController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating SteeringDriverController...");

  if (!steering_controller_->init())
  {
    RCLCPP_FATAL(get_node()->get_logger(),
      "Failed to initialise SteeringController (CAN/CANopen init error)");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialise phase counters with configured offsets
  read_counter_  = effective_read_multiplicity_  - static_cast<int>(params_.read_offset);
  write_counter_ = effective_write_multiplicity_ - static_cast<int>(params_.write_offset);

  // Start from zero command
  target_position_rad_.store(0.0, std::memory_order_relaxed);

  RCLCPP_INFO(get_node()->get_logger(), "SteeringDriverController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringDriverController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating SteeringDriverController...");

  if (steering_controller_)
  {
    steering_controller_->shutdown();
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SteeringDriverController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  steering_controller_.reset();
  target_sub_.reset();
  encoder_pub_.reset();
  abs_encoder_counts_pub_.reset();
  inc_encoder_counts_pub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SteeringDriverController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // --- Read path: process CAN frames and sample encoder ---
  read_counter_++;
  if (read_counter_ >= effective_read_multiplicity_)
  {
    read_counter_ = 0;

    // Drive the CANopen state machines and read incoming frames
    steering_controller_->step();

    // Read absolute encoder and publish [rad]
    const int32_t abs_counts = steering_controller_->getAbsoluteEncoderPosition();
    const int32_t inc_counts = steering_controller_->getMotorEncoderPosition();

    auto msg = std_msgs::msg::Float64();
    msg.data = static_cast<double>(abs_counts) / params_.counts_per_radian;
    encoder_pub_->publish(msg);

    msg.data = static_cast<double>(abs_counts);
    abs_encoder_counts_pub_->publish(msg);

    msg.data = static_cast<double>(inc_counts);
    inc_encoder_counts_pub_->publish(msg);
  }

  // --- Write path: send target position command ---
  write_counter_++;
  if (write_counter_ >= effective_write_multiplicity_)
  {
    write_counter_ = 0;

    const double target_rad = target_position_rad_.load(std::memory_order_relaxed);
    const int32_t target_counts =
      static_cast<int32_t>(target_rad * params_.counts_per_radian);

    steering_controller_->setTargetSteeringPosition(target_counts);
  }

  return controller_interface::return_type::OK;
}

}  // namespace caddy_ai2_ros2_control_system_steering_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  caddy_ai2_ros2_control_system_steering_driver::SteeringDriverController,
  controller_interface::ControllerInterface)
