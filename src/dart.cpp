#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/interface/hseven.hpp"
#include "tasks/auto_dart/dart_detector.hpp"
#include "tools/dart_recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | ../configs/dart.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::DartRecorder recorder(100);  //根据实际帧率调整

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Hseven hseven(config_path);
  io::Camera camera(config_path);
  auto_dart::DartDetector detector(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  io::DartCommand dart_cmd{};

  // send rate limiting
  const std::chrono::milliseconds kMinSendInterval(50);
  const double kYawThreshold = 0.01;  // rad, minimal yaw change to trigger send
  auto last_send_time = std::chrono::steady_clock::now() - kMinSendInterval;
  double last_sent_yaw = 0.0;
  bool last_sent_not_found = false;

  while (!exiter.exit()) {
    camera.read(img, t);

    recorder.record(img, t);
    std::vector<auto_dart::LightSpot> lightspots = detector.detect(img);

    if (!lightspots.empty()) {
      auto command = detector.aim(lightspots.front(), hseven.get_offset());
      // build and send DartCommand (only yaw is needed by hseven)
      dart_cmd.yaw = command.yaw;
      auto now = std::chrono::steady_clock::now();
      bool yaw_changed = std::fabs(dart_cmd.yaw - last_sent_yaw) > kYawThreshold;
      if (yaw_changed || now - last_send_time >= kMinSendInterval) {
        try {
          hseven.send(dart_cmd);
          last_send_time = now;
          last_sent_yaw = dart_cmd.yaw;
          last_sent_not_found = false;
        } catch (...) {
          // ignore send errors
        }
      }
    } else {
      tools::logger()->debug("no target");
      auto now = std::chrono::steady_clock::now();
      if (last_sent_not_found == false || now - last_send_time >= kMinSendInterval) {
        try {
          hseven.send_not_found();
          last_send_time = now;
          last_sent_not_found = true;
        } catch (...) {
          // ignore
        }
      }
    }
  }

  return 0;
}