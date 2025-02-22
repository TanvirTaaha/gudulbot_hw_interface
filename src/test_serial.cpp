#include <cstdio>
#include <string>
#include <thread>

#include "serialx/serialx.h"

int main() {
  serialx::SerialX serial("/dev/ttyUSB0", 115200);
  if (!serial.isOpen()) {
    serial.open();
  }
  int ch;
  (void)ch;
  while (1) {
    if (serial.isOpen()) {
      printf("loop");
      printf("\"%s\"\n", serial.readline().c_str());
      serial.flush();
      serial.flushInput();
      std::string str = "a_3339.8_3339.8";
      str.append(40 - str.length() - 2, '_');
      str += '\n';
      serial.write(str);
      serial.flush();
      serial.flushOutput();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  serial.close();
  return 0;
}
// a_0.0_0.0_____________________________