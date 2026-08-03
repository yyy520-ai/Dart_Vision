#ifndef IO__HSEVEN_HPP
#define IO__HSEVEN_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) Vision_receive  //2+4=6
{
  uint8_t head[2] = {'M', 'A'};
  float offset;
  uint16_t crc16;
};

struct __attribute__((packed)) Vision_send  //2+4=6
{
  uint8_t head[2] = {'M', 'A'};
  float yaw;
  uint16_t crc16;
};

class Hseven
{
public:
  double offset;
  Hseven(const std::string & config_path);
  ~Hseven();
  void send(const DartCommand & command);
  void send_not_found();
  double get_offset() const;

private:
  std::string serial_name;

  serial::Serial serial_;
  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex;
  Vision_receive rxdata_;
  Vision_send tx_data_;

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__HSEVEN_HPP