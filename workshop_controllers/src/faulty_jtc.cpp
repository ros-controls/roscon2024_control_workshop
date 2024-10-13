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

// Changed services history QoS to keep all so we don't lose any client service
// calls \note The versions conditioning is added here to support the
// source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 17
rclcpp::QoS qos_services =
  rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
    .reliable()
    .durability_volatile();
#else
static const rmw_qos_profile_t qos_services = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};
#endif

namespace workshop_controllers
{

controller_interface::CallbackReturn FaultyJTC::on_configure(const rclcpp_lifecycle::State & state)
{
  auto set_fault_callback =
    [&](
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Setting this JTC fail in the update(): %s",
      request->data ? "true" : "false");

    should_fail_ = request->data;
    response->success = true;
  };

  service_ = get_node()->create_service<example_interfaces::srv::SetBool>(
    "~/set_fault", set_fault_callback, qos_services);
  return JointTrajectoryController::on_configure(state);
}

controller_interface::return_type FaultyJTC::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (should_fail_)
  {
    should_fail_ = true;
    return controller_interface::return_type::ERROR;
  }
  return JointTrajectoryController::update(time, period);
}

}  // namespace workshop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(workshop_controllers::FaultyJTC, controller_interface::ControllerInterface)
