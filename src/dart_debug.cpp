#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <thread>

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
  // tools::DartRecorder recorder(100); // Recorder 暂时禁用

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  std::unique_ptr<io::Hseven> hseven;
  try {
    hseven = std::make_unique<io::Hseven>(config_path);
    tools::logger()->info("串口初始化成功");
  } catch (const std::exception & e) {
    tools::logger()->warn("串口初始化失败（忽略，仅影响指令发送）：{}", e.what());
    hseven.reset();
  }

  std::unique_ptr<io::Camera> camera;
  try {
    camera = std::make_unique<io::Camera>(config_path);
    if (!camera) throw std::runtime_error("相机对象创建失败");
    tools::logger()->info("相机初始化成功");
  } catch (const std::exception & e) {
    tools::logger()->error("相机初始化失败（GUI无法显示）：{}", e.what());
    return -1;
  }

  auto_dart::DartDetector detector(config_path, true);
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  io::DartCommand dart_cmd{};

  // 平滑与稳定控制变量
  double smooth_yaw = 0.0;
  bool smooth_initialized = false;
  const double kAlpha =
    0.8;  // 推荐：0.8 比较平衡（可在不同场景调整），往下灵敏但抖动，往上平稳但反应慢
  const double kStableThreshold = 0.005;  // rad, 小于此变化视为稳定
  const int kStableFramesRequired = 5;    // 连续稳定帧数才允许发送
  int stable = 0;
  double last_yaw = 0.0;
  // 发送节流，避免在 stable 条件下连续发送重复指令
  const std::chrono::milliseconds kMinSendInterval(50);
  static auto last_send_time = std::chrono::steady_clock::now() - kMinSendInterval;

  exiter.add_callback([&]() {
    try {
      cv::destroyAllWindows();
    } catch (...) {
    }
    camera.reset();
    hseven.reset();
  });

  bool first_frame = true;

  while (!exiter.exit()) {
    if (!camera) break;

    camera->read(img, t);
    if (img.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // recorder.record(img, t); // Recorder 暂时禁用

    // ========== Dart 检测 ==========
    std::vector<auto_dart::LightSpot> lightspots = detector.detect(img);
    tools::logger()->debug("Detected {} lightspots", lightspots.size());

    // ========== 串口指令发送（带平滑与稳定帧门控） ==========
    if (!lightspots.empty() && hseven) {
      try {
        auto command = detector.aim(lightspots.front(), hseven->get_offset());
        double measured_yaw = command.yaw;

        // 初始化平滑值（首次使用）
        if (!smooth_initialized) {
          smooth_yaw = measured_yaw;
          smooth_initialized = true;
        }

        // 指数平滑：smooth = alpha * smooth + (1-alpha) * measured
        smooth_yaw = kAlpha * smooth_yaw + (1.0 - kAlpha) * measured_yaw;
        dart_cmd.yaw = smooth_yaw;

        // 稳定帧计数：当连续帧变化很小才认为稳定
        if (std::fabs(smooth_yaw - last_yaw) < kStableThreshold) {
          stable++;
        } else {
          stable = 0;
        }

        // 只有在达到稳定帧数后才发送，且满足最小发送间隔，避免抖动或重复发送
        auto now = std::chrono::steady_clock::now();
        if (stable > kStableFramesRequired && now - last_send_time > kMinSendInterval) {
          hseven->send(dart_cmd);
          last_send_time = now;
          last_yaw = smooth_yaw;
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("发送指令失败：{}", e.what());
      }
    } else if (hseven) {
      // 没有检测到目标，通知电控（可考虑节流）
      try {
        hseven->send_not_found();
      } catch (const std::exception & e) {
        tools::logger()->warn("发送未找到包失败：{}", e.what());
      }
    }

    cv::Mat drawlightspot = img.clone();
    for (const auto & lightspot : lightspots) {
      tools::draw_points(drawlightspot, lightspot.contour);
      // 画检测框
      cv::Rect box = cv::boundingRect(lightspot.contour);
      cv::rectangle(drawlightspot, box, cv::Scalar(255, 255, 255), 3);

      // 画中心点
      cv::circle(drawlightspot, lightspot.center, 4, cv::Scalar(0, 0, 255), -1);

      cv::Mat tvec, rvec;
      detector.solvePnP(lightspot, tvec, rvec);

      if (!tvec.empty() && !rvec.empty()) {
        tools::draw_text(
          drawlightspot,
          fmt::format(
            "tvec: x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1),
            tvec.at<double>(2)),
          cv::Point2f(10, 60), cv::Scalar(0, 255, 255));
        tools::draw_text(
          drawlightspot,
          fmt::format(
            "rvec: x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1),
            rvec.at<double>(2)),
          cv::Point2f(10, 260), cv::Scalar(0, 255, 255));
      }
    }

    detector.draw_detect_area(drawlightspot);

    cv::Mat display;
    if (drawlightspot.channels() == 1) {
      cv::cvtColor(drawlightspot, display, cv::COLOR_GRAY2BGR);
    } else if (drawlightspot.channels() == 3) {
      display = drawlightspot;  // OpenCV imshow 默认 BGR
    } else if (drawlightspot.channels() == 4) {
      cv::cvtColor(drawlightspot, display, cv::COLOR_BGRA2BGR);
    } else {
      display = drawlightspot.clone();
    }

    cv::Mat resized;
    cv::resize(display, resized, {}, 0.5, 0.5);
    cv::imshow("lightspot", resized);

    if (first_frame) {
      tools::logger()->debug("显示图像大小: {}x{}", resized.cols, resized.rows);
      first_frame = false;
    }

    if (cv::waitKey(1) == 'q') break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  exiter.invoke_callbacks();
  return 0;
}