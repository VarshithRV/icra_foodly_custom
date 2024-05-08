// Copyright 2024 RT Corporation

#ifndef FOODLY_RD_CONTROL__FOODLY_RD_HARDWARE_HPP_
#define FOODLY_RD_CONTROL__FOODLY_RD_HARDWARE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "foodly_rd_control/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rt_manipulators_cpp/hardware.hpp"
#include "rclcpp_lifecycle/state.hpp"

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;

namespace foodly_rd_control
{
class FoodlyRDHardware : public
  hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FoodlyRDHardware)

  FOODLY_RD_CONTROL_PUBLIC
  ~FoodlyRDHardware();

  FOODLY_RD_CONTROL_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  FOODLY_RD_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FOODLY_RD_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FOODLY_RD_CONTROL_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FOODLY_RD_CONTROL_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  FOODLY_RD_CONTROL_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  FOODLY_RD_CONTROL_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool communication_timeout();

  std::shared_ptr<rt_manipulators_cpp::Hardware> hardware_;
  double timeout_seconds_;

  std::map<std::string, double> hw_position_commands_;
  std::map<std::string, double> hw_position_states_;
  std::map<std::string, double> hw_velocity_states_;
  std::map<std::string, double> hw_effort_states_;

  std::map<std::string, double> current_to_effort_;

  rclcpp::Clock steady_clock_;
  rclcpp::Time prev_comm_timestamp_;
  bool timeout_has_printed_;
};
}  // namespace foodly_rd_control

#endif  // FOODLY_RD_CONTROL__FOODLY_RD_HARDWARE_HPP_
