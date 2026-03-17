#ifndef TOOLS__EXITER_HPP
#define TOOLS__EXITER_HPP

#include <functional>
#include <vector>

namespace tools
{
class Exiter
{
public:
  Exiter();

  bool exit() const;

  // 注册在检测到退出信号后由主线程调用的清理回调
  void add_callback(const std::function<void()> & cb);

  // 由主线程在退出循环后调用，执行所有已注册回调
  void invoke_callbacks();

private:
  std::vector<std::function<void()>> callbacks_;
};

}  // namespace tools

#endif  // TOOLS__EXITER_HPP