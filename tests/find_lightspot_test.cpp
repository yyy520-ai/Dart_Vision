#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/Boomeranger.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/boomerang.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])  //
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  auto yaml = YAML::LoadFile(config_path);
  auto img_num = yaml["img_num"].as<std::string>();
  std::string img_path = "./assets/img/" + img_num + ".jpg";
  cv::Mat img;
  img = cv::imread(img_path);
  auto_aim::Boomeranger boomeranger(config_path);

  std::vector<auto_aim::LightSpot> lightspots = boomeranger.detect(img);
  cv::Mat drawlightspot = img.clone();
  for (const auto & lightspot : lightspots) {
    tools::draw_points(drawlightspot, lightspot.contour);
    cv::Mat tvec, rvec;
    boomeranger.solvePnP(lightspot, tvec, rvec);
    tools::draw_text(
      drawlightspot,
      fmt::format(
        "tvec:  x{: .2f} y{: .2f} z{: .2f}", tvec.at<double>(0), tvec.at<double>(1),
        tvec.at<double>(2)),
      cv::Point2f(10, 60), cv::Scalar(0, 255, 255));
    tools::draw_text(
      drawlightspot,
      fmt::format(
        "rvec:  x{: .2f} y{: .2f} z{: .2f}", rvec.at<double>(0), rvec.at<double>(1),
        rvec.at<double>(2)),
      cv::Point2f(10, 260), cv::Scalar(0, 255, 255));
  }
  std::cout << lightspots.size() << std::endl;
  cv::resize(drawlightspot, drawlightspot, {}, 0.5, 0.5);

  cv::imshow("lightspot", drawlightspot);
  cv::waitKey(0);
  return 0;
}
