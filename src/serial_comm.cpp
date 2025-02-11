#include "gudulbot_hw_interface/serial_comm.hpp"

namespace gudul {

SerialComm::SerialComm(const std::string &port, uint32_t baud, uint32_t timeout_ms)
    : _port(port), _serialx("", baud, serialx::Timeout::simpleTimeout(timeout_ms)) {
#ifdef DEBUG_LOG_ENABLED
  GETLOGGER.set_level(rclcpp::Logger::Level::Debug);
#endif
  RCLCPP_INFO(GETLOGGER, "%sSerialComm constructor called%s", Fore::BLUE, Style::RESET_ALL);
  init_serial();
}
SerialComm::~SerialComm() {
  stop_backgnd_thread();
  close_serial();
}

void SerialComm::init_serial() {
  try {
    _serialx.setPort(_port);
    _serialx.open();
  } catch (const std::exception &e) {
    RCLCPP_FATAL_STREAM(GETLOGGER, Fore::RED << "Serial port open error: " << e.what() << Style::RESET_ALL);
    RCLCPP_INFO_STREAM(GETLOGGER, Fore::CYAN << "Retrying in seconds: 3" << Style::RESET_ALL);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO_STREAM(GETLOGGER, Fore::CYAN << "Retrying in seconds: 2" << Style::RESET_ALL);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO_STREAM(GETLOGGER, Fore::CYAN << "Retrying in seconds: 1" << Style::RESET_ALL);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    close_serial();
    init_serial();  // recursively retry to open port
  }
  RCLCPP_INFO(GETLOGGER, "%sSerial port opened successfully%s", Fore::GREEN, Style::RESET_ALL);
  if (!is_backgnd_thread_running()) {
    std::thread(
        [&]() {
          while (_keep_reading) {
            if (_pause_reading) {
              std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 100)); // Tiny pause
              continue;
            }
            receive_data();
            _serialx.flush();
            _serialx.flushInput();
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 10));
          }
        })
        .detach();
    _pause_reading = false;
    _keep_reading = true;
  }
}

States SerialComm::get_states() {
  static int retries = 0;
  retries++;
  if (retries < MAX_RETRY_ATTEMPTS && GET_NOW_MILLIS_FROM_EPOCH() - _LAST_RECEIVED_ms > MAX_RECEIVED_MSG_AGE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 2));
    return get_states();
  }
  return _states;
}

void SerialComm::write_commands(const Commands &cmds) {
  static int write_error_count = 0;
  _pause_reading = true;
  sprintf(_write_msg_buffer, "a_%.1lf_%.1lf", cmds.velocities.left, cmds.velocities.right);
  try {
    // RCLCPP_DEBUG(GETLOGGER, "write_commands:%s", _write_msg_buffer);
    write_data(_write_msg_buffer);
  } catch (const std::exception &e) {
    write_error_count++;
    RCLCPP_ERROR(GETLOGGER, "write_commands error:%d, reason:%s", write_error_count, e.what());
    if (write_error_count >= MAX_RETRY_ATTEMPTS) {
      RCLCPP_WARN(GETLOGGER, "Too many errors, attempting serial reconnection");
      write_error_count = 0;
      // close_serial();
      // RCLCPP_DEBUG(GETLOGGER, "init_serial called from write_commands 1");
      // init_serial();
    }
  }
  _pause_reading = false;
}

void SerialComm::stop_backgnd_thread() {
  _keep_reading = false;
}

bool SerialComm::is_backgnd_thread_running() {
  return _keep_reading;
}

int SerialComm::receive_data() {
  if (!_serialx.isOpen()) {
    close_serial();
    RCLCPP_DEBUG(GETLOGGER, "init_serial called from receive_data 2");
    init_serial();
  }
  static int read_error_count = 0;
  std::string read_msg = _serialx.readline(MSG_LEN);
  int read_nums;
  if (!read_msg.empty() && read_msg.length() == MSG_LEN) {  // Good Structured message
    read_nums = sscanf(read_msg.c_str(), "e_%lf_%lf_%lf_%lf", &_states.velocities.left, &_states.velocities.right, &_states.positions.left, &_states.positions.right);
    if (read_nums == 4) {
      _LAST_RECEIVED_ms = GET_NOW_MILLIS_FROM_EPOCH();
    } else {
      read_error_count++;
      RCLCPP_WARN(GETLOGGER, "Got malformed data:%s\"%s\"%s", Fore::MAGENTA, read_msg.c_str(), Style::RESET_ALL);
    }
    return read_nums;
  } else {
    read_error_count++;
    RCLCPP_WARN(GETLOGGER, "Got malformed data length error:%s\"%s\"%s", Fore::MAGENTA, read_msg.c_str(), Style::RESET_ALL);
  }
  if (read_error_count >= MAX_RETRY_ATTEMPTS) {
    RCLCPP_WARN(GETLOGGER, "Too many errors, attempting serial reconnection");
    // close_serial();
    // std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE));
    // RCLCPP_DEBUG(GETLOGGER, "init_serial called from receive_data 3");
    // init_serial();
    read_error_count = 0;
  }
  return 0;
}

void SerialComm::write_data(std::string msg) {
  if (!_serialx.isOpen()) {
    close_serial();
    RCLCPP_DEBUG(GETLOGGER, "init_serial called from write_data 4");
    init_serial();
  }
  while (msg.length() < MSG_LEN) {
    msg += "_";
  }
  msg[MSG_LEN - 1] = '\n';
  // RCLCPP_DEBUG(GETLOGGER, "write_data: data going to be written: len:%lu, str:%s", msg.length(), msg.c_str());

  if (_serialx.isOpen()) {
    size_t written_bytes = _serialx.write(msg);
    _serialx.flush();
    _serialx.flushOutput();
    if (written_bytes != msg.length()) {
      RCLCPP_ERROR(GETLOGGER, "write_data: data not written properly: len:%lu, written:%lu", msg.length(), written_bytes);
    }
  } else {
    close_serial();
    RCLCPP_DEBUG(GETLOGGER, "init_serial called from write_data 5");
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
// a_0.0_0.0______________________________