#pragma once

#include <functional>
#include <utility>

namespace odin {
namespace sdk {
class ITransportAddress;
}
}  // namespace odin

namespace odin_driver {
namespace sdk_compat {
// Scoped to one synchronous StartStream call. Each device controller has its
// own thread, so simultaneous devices cannot prepare one another's captures.
class PrepareCapture {
 public:
  explicit PrepareCapture(std::function<bool(const odin::sdk::ITransportAddress&)> prepare)
      : prepare_(std::move(prepare)), previous_(current_) {
    current_ = this;
  }
  ~PrepareCapture() {
    current_ = previous_;
  }
  PrepareCapture(const PrepareCapture&) = delete;
  PrepareCapture& operator=(const PrepareCapture&) = delete;

  static bool run(const odin::sdk::ITransportAddress& endpoint) {
    return !current_ || current_->prepare_(endpoint);
  }

 private:
  std::function<bool(const odin::sdk::ITransportAddress&)> prepare_;
  PrepareCapture* previous_;
  inline static thread_local PrepareCapture* current_ = nullptr;
};

class ScopeExit {
 public:
  explicit ScopeExit(std::function<void()> cleanup) : cleanup_(std::move(cleanup)) {}
  ~ScopeExit() {
    if (cleanup_)
      cleanup_();
  }
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  void dismiss() {
    cleanup_ = nullptr;
  }

 private:
  std::function<void()> cleanup_;
};
}  // namespace sdk_compat
}  // namespace odin_driver
