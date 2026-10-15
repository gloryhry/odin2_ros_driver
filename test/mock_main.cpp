#include <cstdlib>
#include <deque>

#include "mock_sdk.hpp"
#include "odin2_ros_driver/driver.hpp"
namespace {
class DelayedDecoder final : public odin_driver::decoding::Backend {
 public:
  explicit DelayedDecoder(std::unique_ptr<Backend> real) : real_(std::move(real)) {}
  bool asynchronous() const override {
    return true;
  }
  void send(const std::vector<uint8_t>& bytes, int64_t token) override {
    real_->send(bytes, token);
    for (auto& frame : real_->receive())
      ready_.emplace_back(std::chrono::steady_clock::now() + std::chrono::milliseconds(400),
                          std::move(frame));
  }
  std::vector<odin_driver::decoding::Picture> receive() override {
    std::vector<odin_driver::decoding::Picture> out;
    while (!ready_.empty() && ready_.front().first <= std::chrono::steady_clock::now()) {
      out.push_back(std::move(ready_.front().second));
      ready_.pop_front();
    }
    return out;
  }

 private:
  std::unique_ptr<Backend> real_;
  std::deque<std::pair<std::chrono::steady_clock::time_point, odin_driver::decoding::Picture>>
      ready_;
};
}  // namespace
int main(int argc, char** argv) {
  if (std::getenv("ODIN_TEST_ASYNC_DECODER")) {
    auto discover = [](odin_driver::sdk::OdinDataFormat format) {
      auto candidates = odin_driver::decoding::discover(format);
      for (auto& candidate : candidates)
        if (!candidate.hardware) {
          auto factory = candidate.create;
          candidate.name = "test/delayed-cpu";
          candidate.create = [factory] { return std::make_unique<DelayedDecoder>(factory()); };
          return std::vector<odin_driver::decoding::Candidate>{std::move(candidate)};
        }
      throw std::runtime_error("test software backend missing");
    };
    return odin_driver::runDriver(argc, argv, odin_driver::makeMockSdk(), discover);
  }
  return odin_driver::runDriver(argc, argv, odin_driver::makeMockSdk());
}
