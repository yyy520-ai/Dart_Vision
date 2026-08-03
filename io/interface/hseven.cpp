#include "hseven.hpp"

#include <iomanip>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
namespace io
{
Hseven::Hseven(const std::string & config_path) : offset(0.0)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  serial::Timeout t = serial::Timeout::simpleTimeout(20);
  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(115200);
    serial_.setTimeout(t);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[hseven] Failed to open serial: {}", e.what());
    throw;  // 抛出异常，让调用方（如 dart_debug.cpp）捕获并继续运行
  }
  thread_ = std::thread(&Hseven::read_thread, this);
  tools::logger()->info("[heaven] successful received.");
}

double Hseven::get_offset() const
{
  std::lock_guard<std::mutex> lock(mutex);
  return offset;
}

Hseven::~Hseven()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

void Hseven::send(const DartCommand & command)
{
  // 发送简化 DartCommand（只包含 yaw）
  tx_data_.yaw = static_cast<float>(command.yaw);
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[heaven] Failed to write serial (DartCommand): {}", e.what());
  }
}

void Hseven::send_not_found()
{
  // 发送与 DartCommand 相同格式的包，但 yaw=0.0 表示未找到目标
  Vision_send pkt{};
  pkt.head[0] = 'M';
  pkt.head[1] = 'A';
  pkt.yaw = 0.0f;
  pkt.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt) - sizeof(pkt.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
  } catch (const std::exception & e) {
    // 写失败直接返回
    return;
  }
}

bool Hseven::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[heaven] Failed to read serial: {}", e.what());
    return false;
  }
}

void Hseven::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[heaven] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      tools::logger()->info("[heaven] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[heaven] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

void Hseven::read_thread()
{
  tools::logger()->info("[heaven] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[heaven] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rxdata_), sizeof(rxdata_.head))) {
      error_count++;
      continue;
    }

    if (rxdata_.head[0] != 'M' || rxdata_.head[1] != 'A') continue;

    if (!read(
          reinterpret_cast<uint8_t *>(&rxdata_) + sizeof(rxdata_.head),
          sizeof(rxdata_) - sizeof(rxdata_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rxdata_), sizeof(rxdata_))) {
      tools::logger()->debug("[heaven] CRC16 check failed.");
      std::cout << tools::check_crc16(reinterpret_cast<uint8_t *>(&rxdata_), sizeof(rxdata_))
                << std::endl;
      continue;
    }

    error_count = 0;

    // 仅解析并更新电控下发的 offset
    {
      std::lock_guard<std::mutex> lock(mutex);
      offset = static_cast<double>(rxdata_.offset);
    }

    // 记录接收到的 offset（限制日志频率可能另行添加）
    tools::logger()->info("[heaven] Received offset: {:.6f}", offset);
  }

  tools::logger()->info("[heaven] read_thread stopped.");
}

}  // namespace io