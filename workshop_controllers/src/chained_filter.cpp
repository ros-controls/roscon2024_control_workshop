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

// FOR REFERENCE
// enum class interface_configuration_type : std::uint8_t
// {
//   ALL = 0,
//   INDIVIDUAL = 1,
//   NONE = 2,
// };
// struct InterfaceConfiguration
// {
//   interface_configuration_type type;
//   std::vector<std::string> names = {};
// };

controller_interface::InterfaceConfiguration ChainedFilter::command_interface_configuration() const
{
  // TODO fill this
  return {};
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  // TODO fill this
  return {};
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
  // TODO anything to do here?
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO change this to get some proper input values
  const auto sensor_value = 13.37;
  double output = 0.0;

  if (!std::isnan(sensor_value))
  {
    filter_->update(sensor_value, output);
  }

  // TODO get "output" out of this controller somehow

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  // TODO
  return {};
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO anything to do here?
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
