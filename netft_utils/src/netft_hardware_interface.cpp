#include "netft_utils/netft_hardware_interface.hpp"

#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::return_type;

static auto LOGGER = rclcpp::get_logger("NetFTHardwareInterface");

namespace netft_hardware_interface
{
NetFTHardwareInterface::~NetFTHardwareInterface()
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
}

CallbackReturn NetFTHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  std::cout << "In on_init\n";
  // Call parent's init
  auto init_result = hardware_interface::SensorInterface::on_init(hardware_info);
  if (init_result != CallbackReturn::SUCCESS) {
    return init_result;
  }

  // Store the input data
  ip_address_ = info_.hardware_parameters["ip_address"];
  sensor_frame_ = info_.hardware_parameters["sensor_frame"];

  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  std::cout << "In on_configure\n";
  // Connect to the sensor
  netft_ = std::make_unique<netft_rdt_driver::NetFTRDTDriver>(ip_address_, sensor_frame_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NetFTHardwareInterface::export_state_interfaces()
{
  std::cout << "In export_state_interfaces\n";
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/force.x", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.force.x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/force.y", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.force.y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/force.z", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.force.z));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/torque.x", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.torque.x));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/torque.y", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.torque.y));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "netft_sensor/torque.z", hardware_interface::HW_IF_EFFORT, &wrench_data_.wrench.torque.z));

  return state_interfaces;
}

CallbackReturn NetFTHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  std::cout << "In on_activate\n";
  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn NetFTHardwareInterface::on_error(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

return_type NetFTHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::cout << "In on_read\n";
  if (netft_->waitForNewData()) {
    netft_->getData(wrench_data_);
  }

  return return_type::OK;
}

}  // namespace netft_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  netft_hardware_interface::NetFTHardwareInterface, hardware_interface::SensorInterface)
