#include "gudulbot_hw_interface/serial_comm.hpp"

namespace gudul {

SerialComm::SerialComm(std::string port) : port_(port), io_(), ibuffer_(), istream_(&ibuffer_) {
  init_serial();
  std::thread(
      [&]() {
        while (keep_reading) {
          receive_data();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      })
      .detach();
}
SerialComm::~SerialComm() {
  close_serial();
}
void SerialComm::init_serial() {
  try {
    // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Initializing port:%s", port_.c_str());
    printf("Initializing port:%s\n", port_.c_str());
    serial_ = std::make_unique<boost::asio::serial_port>(io_, port_);
    serial_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    keep_reading = true;
  } catch (const std::exception &e) {
    // RCLCPP_ERROR(rclcpp::get_logger("GudulHWInterface"), "Can't init serial port:%s reason:%s", port_.c_str(), e.what());
    // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Retrying in seconds 3");
    printf("Retrying in seconds 3\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Retrying in seconds 2");
    printf("Retrying in seconds 2\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    printf("Retrying in seconds 1\n");
    // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Retrying in seconds 1");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return init_serial();
  }
  // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Serial connection initialized, port:%s", port_.c_str());
}

WheelVelocities SerialComm::get_velos() {
  static int retries = 0;
  retries++;
  if (retries < READ_RETRY_ATTEMPTS && GET_NOW_MILLIS_FROM_EPOCH() - LAST_RECEIVED_ms_ > MAX_RECEIVED_MSG_AGE) {
    std::this_thread::sleep_for(std::chrono::milliseconds(MAX_RECEIVED_MSG_AGE / 2));
    return get_velos();
  }
  return velos_;
}

void SerialComm::write_velos(const WheelVelocities &velos) {
  char write_msg_buffer[MSG_LEN];
  sprintf(write_msg_buffer, "a_%.1lf_%.1lf", velos.left_vel, velos.right_vel);
  try {
    write_data(std::string(write_msg_buffer));
  } catch (const std::exception &e) {
    // RCLCPP_ERROR(rclcpp::get_logger("GudulHWInterface"), "%s", e.what());
    close_serial();
    init_serial();
  }
}

void SerialComm::stop_backgnd_thread() {
  keep_reading = false;
}

bool SerialComm::is_backgnd_thread_running() {
  return keep_reading;
}

bool SerialComm::receive_data() {
  if (!serial_ || !serial_->is_open()) {
    init_serial();
  }
  static int retries = 0;
  std::size_t n = read_until(*serial_, ibuffer_, '\n');
  std::getline(istream_, imsg_);
  ibuffer_.consume(n);
  retries++;
  if (!imsg_.empty() && n == MSG_LEN && imsg_.c_str()[0] == 'e') {  // Good Structured message
    sscanf(imsg_.c_str(), "e_%lf_%lf", &velos_.left_vel, &velos_.right_vel);
    LAST_RECEIVED_ms_ = GET_NOW_MILLIS_FROM_EPOCH();
  } else {
    if (retries > READ_RETRY_ATTEMPTS) {
      // RCLCPP_INFO(rclcpp::get_logger("GudulHWInterface"), "Nothing received for last %d attemps.", READ_RETRY_ATTEMPTS);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return receive_data();  // Retry to get good message
  }
  return true;
}

void SerialComm::write_data(std::string msg) {
  if (!serial_ || !serial_->is_open()) {
    init_serial();
  }
  while (msg.length() < MSG_LEN) {
    msg += "_";
  }
  msg[MSG_LEN] += '\n';
  if (serial_ && serial_->is_open()) {
    write(*serial_, boost::asio::buffer(msg));
  } else {
    init_serial();
    return write_data(msg);
  }
}

void SerialComm::close_serial() {
  if (serial_->is_open()) serial_->close();
}

}  // namespace gudul