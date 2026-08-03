#ifndef TOOLS__DART_RECORDER_HPP
#define TOOLS__DART_RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
class DartRecorder
{
public:
  DartRecorder(double fps = 30);
  ~DartRecorder();
  void record(const cv::Mat & img, const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void init(const cv::Mat & img);
  void save_to_file();
};

}  // namespace tools

#endif  // TOOLS__DART_RECORDER_HPP