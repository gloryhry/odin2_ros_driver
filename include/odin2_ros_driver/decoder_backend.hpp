#pragma once

#include <sensor_msgs/Image.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "core.hpp"

namespace odin_driver::decoding {
// Backends never choose software themselves. Selection and recovery belong to Decoder.
enum class Failure { backend, bitstream };
class Error : public std::runtime_error {
 public:
  Error(Failure kind, const std::string& message) : std::runtime_error(message), kind(kind) {}
  Failure kind;
};
struct Picture {
  sensor_msgs::Image image;
  int64_t token = 0;
};
class Backend {
 public:
  virtual ~Backend() = default;
  virtual void send(const std::vector<uint8_t>& bytes, int64_t token) = 0;
  virtual std::vector<Picture> receive() = 0;
  virtual bool asynchronous() const {
    return false;
  }
};
struct Candidate {
  std::string name, device;
  bool hardware = false;
  std::function<std::unique_ptr<Backend>()> create;
};
using Discovery = std::function<std::vector<Candidate>(sdk::OdinDataFormat)>;
using Clock = std::function<std::chrono::steady_clock::time_point()>;
std::vector<Candidate> discover(sdk::OdinDataFormat format);
std::vector<Candidate> jetsonCandidates(sdk::OdinDataFormat format);
sensor_msgs::Image convertNv12(const sdk::OdinCameraPacket& packet);
}  // namespace odin_driver::decoding
