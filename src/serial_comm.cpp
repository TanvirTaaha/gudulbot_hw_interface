#include "gudulbot_hw_interface/serial_comm.hpp"

namespace gudul {

SerialComm::SerialComm(const std::string &port, uint32_t baud, uint32_t timeout_ms)
    : _port(port), _serialx("", baud, serialx::Timeout::simpleTimeout(timeout_ms)) {
  try {
    init_serial();
    std::thread(
        [&]() {
          while (_keep_reading) {
            receive_data();
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 10));
          }
        })
        .detach();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(GETLOGGER, "%s", e.what());
    RCLCPP_INFO(GETLOGGER, "Retrying in seconds: 3");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(GETLOGGER, "Retrying in seconds: 2");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(GETLOGGER, "Retrying in seconds: 1");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    init_serial();
  }
  _keep_reading = true;
}
SerialComm::~SerialComm() {
  close_serial();
}

void SerialComm::init_serial() {
  _serialx.setPort(_port);
  _serialx.open();
}

States SerialComm::get_states() {
  static int retries = 0;
  retries++;
  if (retries < READ_RETRY_ATTEMPTS && GET_NOW_MILLIS_FROM_EPOCH() - _LAST_RECEIVED_ms > MAX_RECEIVED_MSG_AGE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 2));
    return get_states();
  }
  return _states;
}

void SerialComm::write_commands(const Commands &cmds) {
  char write_msg_buffer[MSG_LEN];
  sprintf(write_msg_buffer, "a_%.1lf_%.1lf", cmds.velocities.left, cmds.velocities.right);
  try {
    write_data(std::string(write_msg_buffer));
  } catch (const std::exception &e) {
    // RCLCPP_ERROR(rclcpp::get_logger("GudulHWInterface"), "%s", e.what());
    close_serial();
    init_serial();
  }
}

void SerialComm::stop_backgnd_thread() {
  _keep_reading = false;
}

bool SerialComm::is_backgnd_thread_running() {
  return _keep_reading;
}

bool SerialComm::receive_data() {
  if (!_serialx.isOpen()) {
    init_serial();
  }
  static int retries = 0;
  std::string read_msg = _serialx.readline(MSG_LEN);
  retries++;
  if (!read_msg.empty() && read_msg.length() == MSG_LEN && read_msg.at(0) == 'e') {  // Good Structured message
    sscanf(read_msg.c_str(), "e_%lf_%lf_%lf_%lf", &_states.velocities.left, &_states.velocities.left, &_states.positions.left, &_states.positions.right);
    _LAST_RECEIVED_ms = GET_NOW_MILLIS_FROM_EPOCH();
  } else {
    if (retries > READ_RETRY_ATTEMPTS) {
      // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Nothing received for last %d attemps.", READ_RETRY_ATTEMPTS);
      retries = 0;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 10));
    return receive_data();  // Retry to get good message
  }
  return true;
}

void SerialComm::write_data(std::string msg) {
  if (!_serialx.isOpen()) {
    init_serial();
  }
  while (msg.length() < MSG_LEN) {
    msg += "_";
  }
  msg[MSG_LEN] += '\n';
  if (_serialx.isOpen()) {
    // write(*serial_, boost::asio::buffer(msg));
    _serialx.write(msg);
  } else {
    init_serial();
    return write_data(msg);
  }
}

void SerialComm::close_serial() {
  if (_serialx.isOpen()) _serialx.close();
}

void SerialComm::enumerate_ports() {
  std::vector<serialx::PortInfo> devices_found = serialx::list_ports();
  for (serialx::PortInfo device : devices_found) {
    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

}  // namespace gudul