#ifndef DART_DETECTOR_HPP
#define DART_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

#include "io/command.hpp"
#include "lightspot.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_dart
{
class DartDetector  //飞镖识别
{
public:
  DartDetector(const std::string config_path, bool debug = false);
  io::DartCommand aim(const LightSpot lightspot, double offset);
  void draw_detect_area(cv::Mat & img);
  std::vector<LightSpot> detect(
    const cv::Mat & img);  // 检测图片当中的指示灯,返回检测到的指示灯数组
  void solvePnP(
    const LightSpot lightspot, cv::Mat & tvec, cv::Mat & rvec);  // 获得当前指示灯的方向向量

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  bool debug_;
  double familiar_;
  double min_area_;
  double max_area_;
  double max_light_;
  double height_threshold_;
  double low_threshold_;
  double yaw_offset_;
  bool isCircle(
    const std::vector<cv::Point> & contour);  // 检测该轮廓是否为圆，并对过小的圆进行过滤
};

};  // namespace auto_dart

#endif
