#include "gudulbot_hw_interface/hw_interface.hpp"

namespace gudul {

hardware_interface::CallbackReturn GudulHWInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("GudulHWInterface"), "Bismillah tei GOLOD....Failed at on_init");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GudulHWInterface"), "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GudulHWInterface"), "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GudulHWInterface"), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GudulHWInterface"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("GudulHWInterface"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "on_init success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GudulHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left joint state interfaces velocity and position
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_state_vels_.left));
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_state_poss_.left));
  // Right joint state interfaces velocity and position
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_state_vels_.right));
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &hw_state_poss_.right));

  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "on_export_state_interfaces success");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GudulHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // left command velocity interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_command_vels_.left));
  // right command velocity interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_command_vels_.right));

  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "on_export_command_interfaces success");
  return command_interfaces;
}

hardware_interface::CallbackReturn GudulHWInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "on_activate: started");

  // Set positions to zero
  hw_state_poss_.left = 0.0;
  hw_state_poss_.right = 0.0;
  // Set velocities to zero
  hw_state_vels_.left = 0.0;
  hw_state_vels_.right = 0.0;

  // command and state should be equal when starting
  // Set commands to zero
  hw_command_vels_.left = hw_state_vels_.left;
  hw_command_vels_.right = hw_state_vels_.right;

  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GudulHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "on_deactivate: started!");
  // command and state should be equal when starting
  // Set commands to zero
  hw_command_vels_.left = hw_state_vels_.left;
  hw_command_vels_.right = hw_state_vels_.right;

  // serial_comm_->write_velos({0.0, 0.0});

  // serial_comm_->close_serial();

  RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GudulHWInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // WheelVelocities velos = serial_comm_->get_velos();
  //  hw_velocities_[0] = velos.left_vel;
  //  hw_velocities_[1] = velos.right_vel;
  hw_state_poss_.left = 0.0;
  hw_state_poss_.right = 0.0;
  hw_state_vels_.left = 0.0;
  hw_state_vels_.right = 0.0;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GudulHWInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  // Write the commands to serial
  // serial_comm_->write_velos({hw_commands_[0], hw_commands_[1]});
  
  return hardware_interface::return_type::OK;
}

}  // namespace gudul

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gudul::GudulHWInterface, hardware_interface::SystemInterface)