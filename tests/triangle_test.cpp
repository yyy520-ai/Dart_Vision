#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/interface/hseven.hpp"
#include "tools/dart_recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/dart.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::DartRecorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Hseven hseven(config_path);
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  std::chrono::steady_clock::time_point t_start;
  std::chrono::steady_clock::time_point t_end;
  int count = 0;
  bool open = false;
  bool start_recording = false;
  while (!exiter.exit()) {
    camera.read(img, t);
    if (hseven.control && !open) {
      open = true;
      t_start = std::chrono::steady_clock::now();
      count++;
    }
    t_end = std::chrono::steady_clock::now();
    if (open) {
      if (t_end - t_start < std::chrono::seconds(17)) {
        cv::Mat img_draw = img.clone();
        tools::draw_text(
          img_draw, fmt::format("{}", count), cv::Point(10, 10), cv::Scalar(0, 0, 255), 1, 2);
        int height = frame.rows;
        int width = frame.cols;

        // 定义线条颜色（BGR格式，这里用白色）
        cv::Scalar lineColor(255, 255, 255);

        for (int i = 1; i < 7; i++) {
          int y = height / 3 + height / 3 / 6 * i;
        }
        for (int i = 1; i < 3; i++) {
          int y = height * i / 3;

          cv::line(img_draw, cv::Point(0, y), cv::Point(width, y), lineColor, 1);
        }

        // 绘制竖线（将图像分成3份）
        for (int i = 1; i < 3; i++) {
          int x = width * i / 3;
          cv::line(img_draw, cv::Point(x, 0), cv::Point(x, height), lineColor, 1);
        }
        recorder.record(img_draw, t);
      }
    } else
      open = false;
  }
  return 0;
}