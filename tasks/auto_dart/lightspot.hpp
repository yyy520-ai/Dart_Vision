#ifndef LIGHTSPOT_HPP
#define LIGHTSPOT_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_dart
{
constexpr double PI = 3.1415926;
constexpr double LIGHTSPOT_RADIUS = 55e-3;  // 发光板直径     单位：m

static const std::vector<cv::Point3f> lightspot_points{
  {0, -LIGHTSPOT_RADIUS / 2, 0},   // 点1
  {LIGHTSPOT_RADIUS / 2, 0, 0},    // 点2
  {0, LIGHTSPOT_RADIUS / 2, 0},    // 点3
  {-LIGHTSPOT_RADIUS / 2, 0, 0}};  // 点4
// 以圆心为xy的(0,0)，12点钟开始为点1,顺时钟方向为点2、点3、点4

struct LightSpot  //  绿灯
{
  cv::Point2f center, top, bottom, left, right;  // 圆上下左右四点坐标
  float radius;                                  // 半径         单位：
  std::vector<cv::Point> contour;                // 轮廓点
  LightSpot(std::vector<cv::Point> contour, float radius, cv::Point2f center)
  {
    this->contour = contour;
    this->radius = radius;
    this->center = center;
    this->left = cv::Point(center.x - std::trunc(radius * 1000) / 1000.0f, center.y);
    this->top = cv::Point(center.x, center.y - std::trunc(radius * 1000) / 1000.0f);
    this->bottom = cv::Point(center.x, center.y + std::trunc(radius * 1000) / 1000.0f);
    this->right = cv::Point(center.x + std::trunc(radius * 1000) / 1000.0f, center.y);
    // this->left = cv::Point(center.x - int(radius), center.y);
    // this->top = cv::Point(center.x, center.y - int(radius));
    // this->bottom = cv::Point(center.x, center.y + int(radius));
    // this->right = cv::Point(center.x + int(radius), center.y);
    // 方案一：以最小外接圆得到的半径和圆心来得到上下左右点坐标
    // 问题：远距离识别时会造成很大误差
    // 解决：使用trunc函数来只取小数点后一位
    // std::sort(contour.begin(), contour.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    //   return a.y < b.y;
    // });  // 从小到大进行排序
    // this->top = contour[0];
    // this->bottom = contour.back();
    // std::sort(contour.begin(), contour.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    //   return a.x < b.x;
    // });  // 左到右进行排序
    // this->left = contour[0];
    // this->right = contour.back();
    //方案二：以上下左右极值作为识别到的绿灯上下左右四个点
    //问题：距离测量值会非常不稳定，近距离变化在一米以内，远距离会达到3米幅度。

    // for (std::vector<cv::Point>::iterator it = contour.begin(); it != contour.end(); it++) {
    //   if (fabs(it->x - center.x) > 0.01) {
    //     if (it->y > center.y)
    //       this->bottom = *it;
    //     else
    //       this->top = *it;
    //   }
    //   if (fabs(it->y - center.y) > 0.01) {
    //     if (it->x > center.x)
    //       this->right = *it;
    //     else
    //       this->left = *it;
    //   }
    // }
    //方案三：与圆心xy相同的四个点作为圆的上下左右四个点
    //问题：会出现tvec解不出值的情况，疑似因为四个点会有找不到的情况。
    //解决：两者在一定范围就判定是
    //问题2：解出来的值依旧不稳定
  };
};
};  // namespace auto_dart

#endif
