// Copyright (c) 2024, ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Dr. Denis
//

#ifndef WORKSHOP_CONTROLLERS__SLEEPY_CONTROLLER_HPP_
#define WORKSHOP_CONTROLLERS__SLEEPY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "sleepy_controller_parameters.hpp"

namespace workshop_controllers
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t
{
  FAST = 0,
  SLOW = 1,
};

class SleepyController : public controller_interface::ChainableControllerInterface
{
public:
  SleepyController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerModeSrvType = example_interfaces::srv::SetBool;

protected:
  std::shared_ptr<sleepy_controller::ParamListener> param_listener_;
  sleepy_controller::Params params_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;
};

}  // namespace workshop_controllers

#endif  // WORKSHOP_CONTROLLERS__SLEEPY_CONTROLLER_HPP_
