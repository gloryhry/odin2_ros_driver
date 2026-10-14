#pragma once
#include <sensor_msgs/Image.h>

#include <memory>

#include "core.hpp"
namespace odin_driver {
class Decoder {
 public:
  Decoder();
  ~Decoder();
  Decoder(const Decoder&) = delete;
  Decoder& operator=(const Decoder&) = delete;
  void reset(bool preserve_parameters = false);
  std::vector<sensor_msgs::Image> decode(const sdk::OdinCameraPacket&, const std_msgs::Header&);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace odin_driver
