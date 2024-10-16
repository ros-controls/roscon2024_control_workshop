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
// Authors: Bence Magyar
//

#include "workshop_controllers/chained_filter.hpp"

#include "rclcpp/version.h"

namespace
{  // utility

// Changed services history QoS to keep all so we don't lose any client service calls
// \note The versions conditioning is added here to support the source-compatibility with Humble
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

}  // namespace

namespace workshop_controllers
{
controller_interface::CallbackReturn ChainedFilter::on_init()
{
  try
  {
    param_listener_ = std::make_shared<chained_filter::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChainedFilter::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration interfaces_config;
  interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  interfaces_config.names = input_state_interfaces_;

  return interfaces_config;
}

controller_interface::CallbackReturn ChainedFilter::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();
  input_state_interfaces_ = {params_.input_interface};

  joints_cmd_sub_ = this->get_node()->create_subscription<DataType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const DataType::SharedPtr msg) { rt_buffer_ptr_.writeFromNonRT(msg); });

  command_interfaces_.reserve(input_state_interfaces_.size());

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  output_state_interface_names_ = {params_.output_interface};
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(
    output_state_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // //  check if we have all resources defined in the "points" parameter
  // //  also verify that we *only* have the resources defined in the "points" parameter
  // // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   ordered_interfaces;
  // if (
  //   !controller_interface::get_ordered_interfaces(
  //     command_interfaces_, input_state_interfaces_, std::string(""), ordered_interfaces) ||
  //   input_state_interfaces_.size() != ordered_interfaces.size())
  // {
  //   RCLCPP_ERROR(
  //     this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
  //     input_state_interfaces_.size(), ordered_interfaces.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(
    reference_interfaces_.begin(), reference_interfaces_.end(),
    std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ChainedFilter::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto sensor_value = state_interfaces_[0].get_value();
  RCLCPP_INFO(get_node()->get_logger(), "Pre-filter value is %f", sensor_value);
  RCLCPP_INFO(
    get_node()->get_logger(), "Chained Filter is %s.",
    (is_in_chained_mode() ? "chained" : "not chained"));

  if (!std::isnan(sensor_value))
  {
    reference_interfaces_[0] = sensor_value;
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> exported_interfaces;

  for (size_t i = 0; i < output_state_interface_names_.size(); ++i)
  {
    exported_interfaces.push_back(hardware_interface::StateInterface(
      get_node()->get_name(), output_state_interface_names_[i], &reference_interfaces_[i]));
  }

  return exported_interfaces;
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if (!(!joint_commands || !(*joint_commands)))
  {
    reference_interfaces_ = {(*joint_commands)->data};
  }

  return controller_interface::return_type::OK;
}

// rclcpp::NodeOptions ChainedFilter::define_custom_node_options() const
// {
//   return rclcpp::NodeOptions()
//     .allow_undeclared_parameters(true)
//     .automatically_declare_parameters_from_overrides(false);
// }

}  // namespace workshop_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  workshop_controllers::ChainedFilter, controller_interface::ChainableControllerInterface)
