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
ChainedFilter::ChainedFilter() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn ChainedFilter::on_init()
{
  try
  {
    param_listener_ = std::make_shared<chained_filter::ParamListener>(get_node());
    filter_ = std::make_unique<filters::FilterChain<double>>("double");
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedFilter::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = configure_parameters();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

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

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChainedFilter::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;

  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = {params_.output_interface};

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ChainedFilter::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> ChainedFilter::on_export_reference_interfaces()
{
  reference_interfaces_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), params_.input_interface, &reference_interfaces_[0]));

  return reference_interfaces;
}

std::vector<hardware_interface::StateInterface> ChainedFilter::on_export_state_interfaces()
{
  state_interfaces_values_.resize(dof_, std::numeric_limits<double>::quiet_NaN());

  return {hardware_interface::StateInterface(
    get_node()->get_name(), params_.output_interface, &state_interfaces_values_[0])};
}

controller_interface::CallbackReturn ChainedFilter::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedFilter::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();

  double filtered_value = reference_interfaces_[0];

  if (!std::isnan(reference_interfaces_[0]))
  {
    filter_->update(reference_interfaces_[0], filtered_value);
  }

  RCLCPP_WARN(get_node()->get_logger(), "Filtered value is %f", filtered_value);

  command_interfaces_[0].set_value(filtered_value);

  return controller_interface::return_type::OK;
}

bool ChainedFilter::on_set_chained_mode(bool /*chained_mode*/)
{
  // Always accept switch to/from chained mode
  return true;
}

controller_interface::return_type ChainedFilter::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // there's no non-chained mode here
  return controller_interface::return_type::OK;
}

void ChainedFilter::update_parameters()
{
  if (!param_listener_->is_old(params_))
  {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn ChainedFilter::configure_parameters()
{
  update_parameters();

  if (params_.input_interface.empty() || params_.output_interface.empty())
  {
    RCLCPP_FATAL(
      get_node()->get_logger(), "Both input_interface and output_interface should be defined!");
    return CallbackReturn::FAILURE;
  }

  dof_ = 1;

  return CallbackReturn::SUCCESS;
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
