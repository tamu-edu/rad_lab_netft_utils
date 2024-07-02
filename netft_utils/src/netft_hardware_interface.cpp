// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Subhas Das, Denis Stogl
//

#include "netft_utils/netft_hardware_interface.hpp"

#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace netft_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn NetFTHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  ip_address_ = info_.hardware_parameters["address"];

  hw_sensor_states_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NetFTHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

CallbackReturn NetFTHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("NetFTHardwareInterface"), "Opening sensor at: " << ip_address_);

  try {
    ft_driver_ = std::make_unique<netft_rdt_driver::NetFTRDTDriver>(ip_address_);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("NetFTHardwareInterface"), "Failed to reach sensor at: " << ip_address_);
    throw e;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type NetFTHardwareInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  geometry_msgs::msg::WrenchStamped wrench;
  ft_driver_->getData(wrench);

  hw_sensor_states_.at(0) = wrench.wrench.force.x;
  hw_sensor_states_.at(1) = wrench.wrench.force.y;
  hw_sensor_states_.at(2) = wrench.wrench.force.z;
  hw_sensor_states_.at(3) = wrench.wrench.torque.x;
  hw_sensor_states_.at(4) = wrench.wrench.torque.y;
  hw_sensor_states_.at(5) = wrench.wrench.torque.z;

  return hardware_interface::return_type::OK;
}

}  // namespace netft_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  netft_hardware_interface::NetFTHardwareInterface, hardware_interface::SensorInterface)
