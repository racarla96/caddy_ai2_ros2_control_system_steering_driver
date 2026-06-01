#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64.hpp"

#include "caddy_ai2_ros2_control_system_steering_driver/steering_driver_controller_parameters.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"

namespace caddy_ai2_ros2_control_system_steering_driver
{

class SteeringDriverController : public controller_interface::ControllerInterface
{
public:
  SteeringDriverController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<steering_driver_controller::ParamListener> param_listener_;
  steering_driver_controller::Params params_;

  std::unique_ptr<SteeringController> steering_controller_;

  // ROS2 I/O: target position commands in, encoder position out
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_pub_;

  // RT-safe target storage (written by subscription callback, read in update)
  std::atomic<double> target_position_rad_{0.0};

  // Multiplicity / phase counters (mirrors hardware interface logic)
  int frequency_ratio_{1};
  int effective_read_multiplicity_{1};
  int effective_write_multiplicity_{1};
  int read_counter_{0};
  int write_counter_{0};
};

}  // namespace caddy_ai2_ros2_control_system_steering_driver
