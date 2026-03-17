#include "dart_detector.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
namespace auto_dart
{
DartDetector::DartDetector(const std::string config_path, bool debug) : debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  familiar_ = yaml["familiar"].as<double>();
  min_area_ = yaml["min_area"].as<double>();
  max_area_ = yaml["max_area"].as<double>();
  max_light_ = yaml["max_light"].as<double>();
  height_threshold_ = yaml["height_threshold"].as<double>();
  low_threshold_ = yaml["low_threshold"].as<double>();
  yaw_offset_ = yaml["yaw_offset"].as<double>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}
bool DartDetector::isCircle(const std::vector<cv::Point> & contour)  // 判断这个轮廓是不是圆形
{
  double area = cv::contourArea(contour);              // 计算该轮廓的面积
  if (area<min_area_ || area> max_area_) return false;  // 排除干扰点
  double perimeter = cv::arcLength(contour, true);  // 计算该轮廓的周长，第二个>参数表示其是否封闭
  // 计算圆形度（PS：比较他两的面积）
  double circularity = (4 * PI * area) / (perimeter * perimeter);
  // 设定一个阈值，接近1的值表示更接近圆形
  return circularity > familiar_;
}
std::vector<LightSpot> DartDetector::detect(const cv::Mat & img)
{
  std::vector<cv::Mat> channels;
  cv::split(img, channels);
  cv::Mat green = channels.at(1);
  cv::Mat gauss;
  cv::GaussianBlur(green, gauss, cv::Size(15, 15), 0);  //降噪

  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gauss, binary_img, max_light_, 255, cv::THRESH_BINARY);

  // 显示二值图,调试用
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  if (debug_) {
    cv::imshow("binary", binary_img);
  }

  std::vector<std::vector<cv::Point>> circles;
  for (const auto & contour : contours)  // 找到圆形的轮廓保存到circles当中
  {
    if (isCircle(contour)) {
      double area = cv::contourArea(contour);  // 计算该轮廓的面积
      circles.emplace_back(contour);
    }
  }
  std::vector<LightSpot> lightspots;
  for (const auto & circle : circles) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(circle, center, radius);
    //使用OpenCV的函数来得到最小外接圆的半径和圆心
    auto lightspot = LightSpot(circle, radius, center);
    double height = binary_img.rows * height_threshold_;
    double low = binary_img.rows * low_threshold_;
    if (lightspot.center.y > height && lightspot.center.y < low) lightspots.emplace_back(lightspot);
  }
  return lightspots;
}

void DartDetector::solvePnP(
  const LightSpot lightspot, cv::Mat & tvec, cv::Mat & rvec)  //根据得到的指示灯图像进行坐标转化
{
  std::vector<cv::Point2f> img_points{
    lightspot.top, lightspot.right, lightspot.bottom, lightspot.left};
  cv::solvePnP(lightspot_points, img_points, camera_matrix_, distort_coeffs_, rvec, tvec);
}

io::DartCommand DartDetector::aim(const LightSpot lightspot, double offset)
{
  cv::Mat tvec, rvec;
  this->solvePnP(lightspot, tvec, rvec);

  double yaw = atan2(tvec.at<double>(0), tvec.at<double>(2));
  tools::logger()->info("send yaw:{}", -yaw - yaw_offset_ / 57.3 - offset);
  return {-yaw - yaw_offset_ / 57.3 - offset};
}
void DartDetector::draw_detect_area(cv::Mat & img)
{
  int cx = img.cols / 2;
  cv::line(img, cv::Point(cx, 0), cv::Point(cx, img.rows - 1), cv::Scalar(0, 0, 255), 2);
  double pixels_per_degree = img.cols / 30.0;
  int offset_x = static_cast<int>(yaw_offset_ * pixels_per_degree);
  int line_x = cx + offset_x;
  cv::line(img, cv::Point(line_x, 0), cv::Point(line_x, img.rows - 1), cv::Scalar(0, 0, 255), 2);

  int y1 = static_cast<int>(img.rows * height_threshold_);
  cv::line(img, cv::Point(0, y1), cv::Point(img.cols - 1, y1), cv::Scalar(0, 255, 0), 2);
  int y2 = static_cast<int>(img.rows * low_threshold_);
  cv::line(img, cv::Point(0, y2), cv::Point(img.cols - 1, y2), cv::Scalar(0, 255, 0), 2);
}
}  // namespace auto_dart
