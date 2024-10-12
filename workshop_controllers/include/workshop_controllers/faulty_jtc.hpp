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

#include "example_interfaces/srv/set_bool.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/service.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace workshop_controllers
{

class FaultyJTC : public joint_trajectory_controller::JointTrajectoryController
{
public:
  FaultyJTC() = default;
  ~FaultyJTC() override = default;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool should_fail_ = false;

  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

}  // namespace workshop_controllers

#endif  // WORKSHOP_CONTROLLERS__FAULTY_JTC_HPP_
