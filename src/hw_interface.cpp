#include "gudulbot_hw_interface/hw_interface.hpp"

namespace gudul {

hardware_interface::CallbackReturn GudulHWInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (
      hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(GETLOGGER, "Bismillah tei GOLOD....Failed at on_init");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
          GETLOGGER, "Joint '%s' has %zu command interfaces found. 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          GETLOGGER, "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
          GETLOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          GETLOGGER, "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          GETLOGGER, "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  GETLOGGER.set_level(rclcpp::Logger::Level::Debug);
  _cfg.port = info.hardware_parameters.at("device");
  _cfg.baud = std::stoul(info.hardware_parameters.at("baud_rate"));
  _cfg.timeout_ms = std::stoul(info.hardware_parameters.at("timeout_ms"));
  RCLCPP_DEBUG(GETLOGGER, "on_init configs: port:\"%s\", baud:%u, timeout_ms:%u", _cfg.port.c_str(), _cfg.baud, _cfg.timeout_ms);
  RCLCPP_INFO(GETLOGGER, "on_init success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GudulHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left joint state interfaces velocity and position
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &_hw_state_vels.left));
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &_hw_state_poss.left));
  // Right joint state interfaces velocity and position
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &_hw_state_vels.right));
  state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &_hw_state_poss.right));

  RCLCPP_INFO(GETLOGGER, "on_export_state_interfaces success");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GudulHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // left command velocity interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &_hw_command_vels.left));
  // right command velocity interface
  command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &_hw_command_vels.right));

  RCLCPP_INFO(GETLOGGER, "on_export_command_interfaces success");
  return command_interfaces;
}

hardware_interface::CallbackReturn GudulHWInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(GETLOGGER, "on_activate: started");

  // Set positions to zero
  _hw_state_poss.left = 0.0;
  _hw_state_poss.right = 0.0;
  // Set velocities to zero
  _hw_state_vels.left = 0.0;
  _hw_state_vels.right = 0.0;

  // command and state should be equal when starting
  // Set commands to zero
  _hw_command_vels.left = _hw_state_vels.left;
  _hw_command_vels.right = _hw_state_vels.right;
  RCLCPP_INFO(GETLOGGER, "defining serial");
  _serial_comm = std::make_unique<SerialComm>(_cfg.port, _cfg.baud, _cfg.timeout_ms);
  RCLCPP_INFO(GETLOGGER, "defining serial done");
  _serial_comm->write_commands({_hw_command_vels.left, _hw_command_vels.right});

  RCLCPP_INFO(GETLOGGER, "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GudulHWInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(GETLOGGER, "on_deactivate: started!");
  // command and state should be equal when deactivating
  _hw_command_vels.left = _hw_state_vels.left;
  _hw_command_vels.right = _hw_state_vels.right;

  _serial_comm->write_commands({_hw_command_vels.left, _hw_command_vels.right});
  _serial_comm->close_serial();

  RCLCPP_INFO(GETLOGGER, "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GudulHWInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(GETLOGGER, "read calllleddd......");
  // Read states from serial and give them to controller
  States states = _serial_comm->get_states();
  _hw_state_poss.left = states.positions.left;
  _hw_state_poss.right = states.positions.right;
  _hw_state_vels.left = states.velocities.left;
  _hw_state_vels.right = states.velocities.right;

  RCLCPP_INFO(GETLOGGER, "\tleft_vel:%.2lf, right_vel:%.2lf, left_pos:%.2lf, right_pos:%.2lf", states.velocities.left, states.velocities.right, states.positions.left, states.positions.right);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GudulHWInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  RCLCPP_INFO(GETLOGGER, "write calllleddd......");

  // Write the commands to serial
  _serial_comm->write_commands({_hw_command_vels.left, _hw_command_vels.right});
  RCLCPP_INFO(GETLOGGER, "\tleft_vel:%.2lf, right_vel:%.2lf", _hw_command_vels.left, _hw_command_vels.right);
  return hardware_interface::return_type::OK;
}

}  // namespace gudul

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gudul::GudulHWInterface, hardware_interface::SystemInterface)