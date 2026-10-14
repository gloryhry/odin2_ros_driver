#pragma once
#include <sensor_msgs/Image.h>

#include <memory>

#include "core.hpp"
#include "decoder_backend.hpp"
namespace odin_driver {
struct DecodeContext {
  std_msgs::Header header;
  uint64_t generation = 0, decode_generation = 0;
  ros::WallTime received;
};
struct DecodedFrame {
  sensor_msgs::Image image;
  DecodeContext context;
};
struct DecoderInfo {
  std::string backend = "uninitialized", device, fallback_reason;
  std::vector<std::string> failures;
  bool hardware_active = false;
};
class Decoder {
 public:
  Decoder();
  // Injectable discovery/clock make fallback and cooldown tests independent of GPUs.
  Decoder(decoding::Discovery discovery, decoding::Clock clock);
  ~Decoder();
  Decoder(const Decoder&) = delete;
  Decoder& operator=(const Decoder&) = delete;
  void reset(bool preserve_parameters = false);
  std::vector<sensor_msgs::Image> decode(const sdk::OdinCameraPacket&, const std_msgs::Header&);
  std::vector<DecodedFrame> submit(const sdk::OdinCameraPacket&, const DecodeContext&);
  std::vector<DecodedFrame> poll();
  bool pending() const;
  bool active() const;
  DecoderInfo info() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace odin_driver
