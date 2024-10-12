// Copyright (c) 2024 ros2_control Development Team
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

#ifndef WORKSHOP_CONTROLLERS__FAULTY_JTC_HPP_
#define WORKSHOP_CONTROLLERS__FAULTY_JTC_HPP_

#include <functional> // for std::reference_wrapper
#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "control_msgs/srv/query_trajectory_state.hpp"
#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/service.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals; // NOLINT

namespace workshop_controllers {

class FaultyJTC
    : public joint_trajectory_controller::JointTrajectoryController {
public:
  FaultyJTC() = default;
  ~FaultyJTC() override = default;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  CallbackReturn on_init() override;

private:
  bool should_fail_ = false;

  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

} // namespace workshop_controllers

#endif // WORKSHOP_CONTROLLERS__FAULTY_JTC_HPP_
