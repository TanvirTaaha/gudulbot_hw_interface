#pragma once
#ifndef __GUDUL_SERIAL_COMM_H__
#define __GUDUL_SERIAL_COMM_H__

#include <boost/asio.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <thread>

// #include "rclcpp/rclcpp.hpp"

namespace gudul {
#define MSG_LEN 20
#define READ_RETRY_ATTEMPTS 5
#define MAX_RECEIVED_MSG_AGE 100  // unit:ms, twice than microcontroller delay
#define GET_NOW_MILLIS_FROM_EPOCH() \
  std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()

struct WheelVelocities {
  double left_vel;
  double right_vel;
};

class SerialComm {
 public:
  SerialComm(std::string port);
  ~SerialComm();
  void init_serial();
  WheelVelocities get_velos();
  void write_velos(const WheelVelocities& velos);
  void stop_backgnd_thread();
  bool is_backgnd_thread_running();
  void close_serial();

 private:
  std::string port_;
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> serial_;

  boost::asio::streambuf ibuffer_;
  std::istream istream_;
  std::string imsg_;

  WheelVelocities velos_;
  uint64_t LAST_RECEIVED_ms_;
  bool keep_reading;

  bool receive_data();
  void write_data(std::string msg);
};

}  // namespace gudul

#endif