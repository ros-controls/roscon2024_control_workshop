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
    filter_ = std::make_unique<filters::FilterChain<double>>("double");
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
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, {params_.input_interface}};
}

controller_interface::CallbackReturn ChainedFilter::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  if (!filter_->configure(
        "filter_chain", get_node()->get_node_logging_interface(),
        get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to configure filter chain."
      "Check the parameters for filters setup.");
    return CallbackReturn::FAILURE;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  output_state_value_ = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto sensor_value = state_interfaces_[0].get_value();

  if (!std::isnan(sensor_value))
  {
    filter_->update(sensor_value, output_state_value_);
  }

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  return {hardware_interface::StateInterface(
    get_node()->get_name(), params_.output_interface, &output_state_value_)};
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

rclcpp::NodeOptions ChainedFilter::define_custom_node_options() const
{
  return rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(false);
}

}  // namespace workshop_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  workshop_controllers::ChainedFilter, controller_interface::ChainableControllerInterface)
