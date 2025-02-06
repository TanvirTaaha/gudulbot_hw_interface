#pragma once
#ifndef __GUDUL_HW_INTERFACE__
#define __GUDUL_HW_INTERFACE__

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
// #include "serial_comm.hpp"
#include "std_msgs/msg/string.hpp"

namespace gudul {

struct JointCSValues {
  double left;
  double right;
};

class GudulHWInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GudulHWInterface)
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  // doing the old way, cause not got any example of the new way
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  //   std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
  //   std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  //   std::unique_ptr<SerialComm> serial_comm_;
  JointCSValues hw_command_vels_;
  JointCSValues hw_state_vels_;
  JointCSValues hw_state_poss_;
};
}  // namespace gudul

#endif