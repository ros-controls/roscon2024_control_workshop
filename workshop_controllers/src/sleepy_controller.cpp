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

#include "workshop_controllers/sleepy_controller.hpp"

#include <sys/time.h>
#include <chrono>
#include <ctime>

namespace
{  // utility

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

}  // namespace

namespace workshop_controllers
{
SleepyController::SleepyController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn SleepyController::on_init()
{
  control_mode_.initRT(control_mode_type::FAST);

  try
  {
    param_listener_ = std::make_shared<sleepy_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SleepyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::SLOW);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::FAST);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback, qos_services);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SleepyController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SleepyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> SleepyController::on_export_reference_interfaces()
{
  return std::vector<hardware_interface::CommandInterface>();
}

std::vector<hardware_interface::StateInterface> SleepyController::on_export_state_interfaces()
{
  state_interfaces_values_.resize(1, std::numeric_limits<double>::quiet_NaN());

  return {hardware_interface::StateInterface(
    get_node()->get_name(), "max_sleep_time", &state_interfaces_values_[0])};
}

controller_interface::CallbackReturn SleepyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SleepyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SleepyController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type SleepyController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }

  double fast_factor = 10;
  if (*(control_mode_.readFromRT()) == control_mode_type::SLOW)
  {
    fast_factor = 1.0;
  }

  state_interfaces_values_[0] = std::round(params_.max_sleep_time * 1000 / fast_factor);

  auto generate_seed = []() -> unsigned int
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);  // Get current time with microsecond precision
    unsigned int seed =
      static_cast<unsigned int>(tv.tv_usec ^ tv.tv_sec);  // XOR to combine time values
    return seed;
  };

  unsigned int seed = generate_seed();

  const int sleepTime = rand_r(&seed) % static_cast<int>(state_interfaces_values_[0]);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));

  return controller_interface::return_type::OK;
}

}  // namespace workshop_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  workshop_controllers::SleepyController, controller_interface::ChainableControllerInterface)
