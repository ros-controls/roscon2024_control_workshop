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

#include "workshop_controllers/faulty_jtc.hpp"

namespace workshop_controllers {

controller_interface::CallbackReturn FaultyJTC::on_init() {
  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration
FaultyJTC::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();
  return conf;
}

controller_interface::CallbackReturn
FaultyJTC::on_activate(const rclcpp_lifecycle::State &state) {
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type
FaultyJTC::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
  if (should_fail_) {
    return controller_interface::return_type::ERROR;
  }

  return JointTrajectoryController::update(time, period);
}

} // namespace workshop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(workshop_controllers::FaultyJTC,
                       controller_interface::ControllerInterface)