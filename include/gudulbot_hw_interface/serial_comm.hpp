#pragma once
#ifndef __GUDUL_SERIAL_COMM_H__
#define __GUDUL_SERIAL_COMM_H__

#include <chrono>
#include <cstdio>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serialx/serialx.h"
namespace gudul {
#define MSG_LEN 40  // e_00000.0_00000.0_00000.0_00000.0__\n
#define READ_RETRY_ATTEMPTS 5
#define MAX_RECEIVED_MSG_AGE 100  // unit:ms, twice than microcontroller delay
#define GET_NOW_MILLIS_FROM_EPOCH() \
  std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count()

#ifdef GETLOGGER
#undef GETLOGGER
#endif
#define GETLOGGER rclcpp::get_logger("SerialX")

struct WheelValues {
  double left;
  double right;
};

struct States {
  WheelValues positions;
  WheelValues velocities;
};

struct Commands {
  WheelValues velocities;
};

class SerialComm {
 public:
  SerialComm(const std::string& port, uint32_t baud, uint32_t timeout_ms);
  ~SerialComm();
  void init_serial();
  States get_states();
  void write_commands(const Commands& cmds);
  void stop_backgnd_thread();
  bool is_backgnd_thread_running();
  void close_serial();

 private:
  std::string _port;
  serialx::SerialX _serialx;

  States _states;
  uint64_t _LAST_RECEIVED_ms;
  bool _keep_reading;

  bool receive_data();
  void write_data(std::string msg);
  void enumerate_ports();
};

}  // namespace gudul

#endif