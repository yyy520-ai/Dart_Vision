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
  "{@config-path   | configs/dart.yaml | 位置参数，yaml配置文件路径 }";

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
  io::Command cmd{};

  while (!exiter.exit()) {
    camera.read(img, t);

    recorder.record(img, t);
    std::vector<auto_dart::LightSpot> lightspots = detector.detect(img);

    if (!lightspots.empty()) {
      auto command = detector.aim(lightspots.front(), hseven.offset);
      cmd.yaw = command.yaw;
      cmd.pitch = 0;
      cmd.shoot = false;
      cmd.control = true;
      hseven.send(cmd);
    }
  }

  return 0;
}