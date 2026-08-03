#include "exiter.hpp"

#include <csignal>
#include <functional>
#include <stdexcept>
#include <vector>

namespace tools
{
bool exit_ = false;
bool exiter_inited_ = false;

static std::vector<std::function<void()>> g_callbacks;

Exiter::Exiter()
{
  if (exiter_inited_) throw std::runtime_error("Multiple Exiter instances!");
  // 捕获 SIGINT 和 SIGTERM
  std::signal(SIGINT, [](int) { exit_ = true; });
  std::signal(SIGTERM, [](int) { exit_ = true; });
  exiter_inited_ = true;
}

bool Exiter::exit() const { return exit_; }

void Exiter::add_callback(const std::function<void()> & cb) { g_callbacks.push_back(cb); }

void Exiter::invoke_callbacks()
{
  for (auto & cb : g_callbacks) {
    try {
      cb();
    } catch (...) {
    }
  }
}

}  // namespace tools