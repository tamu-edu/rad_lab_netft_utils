#pragma once

#include "hardware_interface/sensor_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "netft_utils/netft_rdt_driver.h"

namespace netft_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NetFTHardwareInterface : public hardware_interface::SensorInterface
{
public:
  NetFTHardwareInterface();
  ~NetFTHardwareInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info);
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

protected:
  // This is the actual thing that talks with the force torque sensor
  std::unique_ptr<netft_rdt_driver::NetFTRDTDriver> netft_;
  geometry_msgs::msg::WrenchStamped wrench_data_;

  // Data for connecting to sensor
  std::string ip_address_, sensor_frame_;
};
}  // namespace netft_hardware_interface
