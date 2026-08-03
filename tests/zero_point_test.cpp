#include "io/interface/hseven.hpp"

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                       | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  io::Hseven hseven(config_path);
  double total_shift = 0.0;
  while (!exiter.exit()) {
    auto start_timestamp = std::chrono::steady_clock::now();
    Eigen::Quaterniond start_q = hseven.imu_at(start_timestamp);
    Eigen::Vector3d start_eulers = tools::eulers(start_q, 2, 1, 0) * 57.3;
     
    std::this_thread::sleep_for(1s);    
    auto end_timestamp = std::chrono::steady_clock::now();
    Eigen::Quaterniond end_q = hseven.imu_at(end_timestamp);
    Eigen::Vector3d end_eulers = tools::eulers(end_q, 2, 1, 0) * 57.3;
    total_shift+= end_eulers[0]-start_eulers[0];
    tools::logger()->info("zero_point shift in 1s (degree):{}",end_eulers[0]-start_eulers[0]);
    tools::logger()->info("total shift: {}",total_shift);
    
  }

  return 0;
}