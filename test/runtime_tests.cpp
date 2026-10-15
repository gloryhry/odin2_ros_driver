#include <gtest/gtest.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <fstream>
#include <mutex>

#include "odin2_ros_driver/DeviceStatus.h"
#include "odin2_ros_driver/EncodedFrame.h"
#include "odin2_ros_driver/driver.hpp"

using namespace odin_driver;
namespace {
bool until(const std::function<bool()>& condition, double seconds = 3) {
  const auto end = ros::WallTime::now() + ros::WallDuration(seconds);
  while (ros::ok() && ros::WallTime::now() < end) {
    if (condition())
      return true;
    ros::WallDuration(.005).sleep();
  }
  return condition();
}

// A device boundary with caller-controlled data ingress and a healthy command plane.
class FeedSdk : public SdkFacade {
 public:
  std::vector<sdk::DiscoveredDevice> discover(uint32_t) override {
    sdk::DiscoveredDevice d;
    d.sn = "manual";
    return {d};
  }
  sdk::OdinDeviceHandle connect(const sdk::DiscoveredDevice&) override {
    return 1;
  }
  void disconnect(sdk::OdinDeviceHandle) override {}
  void callbacks(sdk::OdinDeviceHandle, PacketCallback value) override {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_ = std::move(value);
  }
  Reply start(sdk::OdinDeviceHandle, Channel, const StreamSettings&, uint32_t) override {
    return {};
  }
  Reply close(sdk::OdinDeviceHandle, Channel) override {
    return {};
  }
  void sync(sdk::OdinDeviceHandle, bool, uint32_t) override {}
  Reply execute(sdk::OdinDeviceHandle, const Command& command) override {
    Reply out;
    if (command.kind == CommandKind::SetDeviceMode)
      mode_ = command.mode;
    out.mode = mode_;
    if (command.kind == CommandKind::Capabilities) {
      for (auto channel : channels()) {
        sdk::OdinSensorCapability cap;
        cap.channel = channel;
        cap.modes.push_back({16, 16, 25, static_cast<sdk::OdinDataFormat>(15)});
        if (channel == Channel::kImage0 || channel == Channel::kImage1)
          cap.modes.push_back({2, 2, 25, sdk::OdinDataFormat::kImageNv12});
        out.capabilities.push_back(cap);
      }
    }
    return out;
  }
  void emit(Channel channel, Packet packet) {
    PacketCallback callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = callback_;
    }
    ASSERT_TRUE(bool(callback));
    callback(channel, std::move(packet));
  }

 private:
  std::atomic<uint8_t> mode_{1};
  std::mutex mutex_;
  PacketCallback callback_;
};

template <class Message>
class Inbox {
 public:
  Inbox(ros::NodeHandle& node, const std::string& topic) {
    subscriber = node.subscribe<Message>(topic, 100, [this](const typename Message::ConstPtr& p) {
      std::lock_guard<std::mutex> lock(mutex_);
      values_.push_back(*p);
    });
  }
  std::vector<Message> values() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return values_;
  }
  ros::Subscriber subscriber;

 private:
  mutable std::mutex mutex_;
  std::vector<Message> values_;
};

DeviceSettings settings() {
  auto value = defaults();
  for (auto& stream : value.streams)
    stream.second.enabled = false;
  value.name = "runtime";
  value.sn = "manual";
  value.worker_threads = 1;
  return value;
}

class Running {
 public:
  explicit Running(DeviceSettings cfg, decoding::Discovery discovery = decoding::discover)
      : node("/runtime_" + std::to_string(++names_)),
        status(node, "status"),
        sdk(std::make_shared<FeedSdk>()),
        driver(node, cfg, sdk, std::move(discovery)) {
    driver.start();
  }
  ~Running() {
    driver.stop();
  }
  bool ready() {
    return until([this] {
      auto v = status.values();
      return !v.empty() && v.back().state == "running";
    });
  }
  ros::NodeHandle node;
  Inbox<odin2_ros_driver::DeviceStatus> status;
  std::shared_ptr<FeedSdk> sdk;
  DeviceDriver driver;

 private:
  static std::atomic<unsigned> names_;
};
std::atomic<unsigned> Running::names_{0};

sdk::OdinPointCloudPacket cloud(uint32_t sequence, uint64_t time) {
  sdk::OdinPointCloudPacket p;
  p.frame_count = sequence;
  p.timestamp = time;
  sdk::OdinRawPoint<float> point;
  point.x = 1000;
  point.y = sequence;
  p.points.push_back(point);
  return p;
}
float cloudY(const sensor_msgs::PointCloud2& p) {
  float value;
  std::memcpy(&value, p.data.data() + 4, sizeof(value));
  return value;
}

TEST(Runtime, CounterAndTimestampRestartWithoutImuResumesAtFirstPacket) {
  auto cfg = settings();
  cfg.streams.at(Channel::kRawPoint).enabled = true;
  Running device(cfg);
  Inbox<sensor_msgs::PointCloud2> output(device.node, "points_raw");
  ASSERT_TRUE(device.ready());
  ASSERT_TRUE(until([&] { return output.subscriber.getNumPublishers() > 0; }));
  auto initial = device.status.values().back();
  device.sdk->emit(Channel::kRawPoint, cloud(100000, 100000000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 1; }));
  device.sdk->emit(Channel::kRawPoint, cloud(1, 1000000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 2; }));
  device.sdk->emit(Channel::kRawPoint, cloud(2, 1100000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 3; }));
  ASSERT_TRUE(until(
      [&] { return device.status.values().back().clock_resets == initial.clock_resets + 1; }));
  auto state = device.status.values().back();
  EXPECT_EQ(state.reconnects, initial.reconnects);
  EXPECT_FLOAT_EQ(cloudY(output.values()[1]), .001f);
  EXPECT_FLOAT_EQ(cloudY(output.values()[2]), .002f);
}

TEST(Runtime, RejectedDuplicateDoesNotAdvanceTimestampOrSequence) {
  auto cfg = settings();
  cfg.streams.at(Channel::kRawPoint).enabled = true;
  Running device(cfg);
  Inbox<sensor_msgs::PointCloud2> output(device.node, "points_raw");
  ASSERT_TRUE(device.ready());
  device.sdk->emit(Channel::kRawPoint, cloud(10, 10000000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 1; }));
  device.sdk->emit(Channel::kRawPoint, cloud(10, 10900000000ULL));
  device.sdk->emit(Channel::kRawPoint, cloud(11, 10100000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 2; }));
  EXPECT_FLOAT_EQ(cloudY(output.values().back()), .011f);
  device.sdk->emit(Channel::kRawPoint, cloud(12, UINT64_MAX));
  device.sdk->emit(Channel::kRawPoint, cloud(12, 10200000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 3; }));
  EXPECT_FLOAT_EQ(cloudY(output.values().back()), .012f);
}

TEST(Runtime, SequenceWrapAndSmallReorderingDoNotResetEpoch) {
  auto cfg = settings();
  cfg.streams.at(Channel::kRawPoint).enabled = true;
  Running device(cfg);
  Inbox<sensor_msgs::PointCloud2> output(device.node, "points_raw");
  ASSERT_TRUE(device.ready());
  const auto resets = device.status.values().back().clock_resets;
  device.sdk->emit(Channel::kRawPoint, cloud(UINT32_MAX, 10000000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 1; }));
  device.sdk->emit(Channel::kRawPoint, cloud(0, 10100000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 2; }));
  device.sdk->emit(Channel::kRawPoint, cloud(UINT32_MAX, 10050000000ULL));
  device.sdk->emit(Channel::kRawPoint, cloud(1, 10200000000ULL));
  ASSERT_TRUE(until([&] { return output.values().size() == 3; }));
  EXPECT_FLOAT_EQ(cloudY(output.values().back()), .001f);
  EXPECT_EQ(device.status.values().back().clock_resets, resets);
}

std::vector<std::vector<uint8_t>> accessUnits(const std::string& name, bool hevc) {
  const char* fixtures = std::getenv("ODIN_TEST_FIXTURES");
  if (!fixtures)
    throw std::runtime_error("ODIN_TEST_FIXTURES is required");
  std::ifstream input(std::string(fixtures) + "/" + name, std::ios::binary);
  std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(input), {}};
  std::vector<size_t> starts;
  for (size_t i = 0; i + 4 < bytes.size(); ++i) {
    size_t length = 0;
    if (bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 1)
      length = 3;
    else if (bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 0 && bytes[i + 3] == 1)
      length = 4;
    if (!length)
      continue;
    const auto type = hevc ? (bytes[i + length] >> 1) & 63 : bytes[i + length] & 31;
    if (type == (hevc ? 35 : 9))
      starts.push_back(i);
    i += length - 1;
  }
  std::vector<std::vector<uint8_t>> out;
  for (size_t i = 0; i < starts.size(); ++i)
    out.emplace_back(bytes.begin() + (i ? starts[i] : 0),
                     bytes.begin() + (i + 1 < starts.size() ? starts[i + 1] : bytes.size()));
  return out;
}

decoding::Discovery software() {
  return [](sdk::OdinDataFormat format) {
    auto candidates = decoding::discover(format);
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                    [](const auto& c) { return c.hardware; }),
                     candidates.end());
    return candidates;
  };
}

class EncodedContinuity : public testing::TestWithParam<bool> {};
TEST_P(EncodedContinuity, LowImageRatePreservesEveryReferencePacketAndDecodability) {
  const bool hevc = GetParam();
  auto units = accessUnits(hevc ? "sequence.h265" : "sequence.h264", hevc);
  ASSERT_EQ(units.size(), 8u);
  auto cfg = settings();
  auto& stream = cfg.streams.at(Channel::kImage0);
  stream.enabled = true;
  stream.publish_rate = 1;
  stream.mode.format = hevc ? sdk::OdinDataFormat::kImageH265 : sdk::OdinDataFormat::kImageH264;
  Running device(cfg, software());
  Inbox<odin2_ros_driver::EncodedFrame> encoded(device.node, "camera0/encoded");
  Inbox<sensor_msgs::Image> images(device.node, "camera0/image_raw");
  ASSERT_TRUE(device.ready());
  for (size_t i = 0; i < units.size(); ++i) {
    sdk::OdinCameraPacket p;
    p.format = stream.mode.format;
    p.frame_count = i + 1;
    p.timestamp = 1000000000ULL + i * 40000000;
    p.payload = units[i];
    device.sdk->emit(Channel::kImage0, p);
    ros::WallDuration(.04).sleep();
  }
  ASSERT_TRUE(until([&] { return encoded.values().size() == units.size(); }));
  ASSERT_TRUE(until([&] { return !images.values().empty(); }));
  EXPECT_EQ(images.values().size(), 1u);
  auto captured = encoded.values();
  Decoder receiver(software(), [] { return std::chrono::steady_clock::now(); });
  size_t decoded = 0;
  for (size_t i = 0; i < captured.size(); ++i) {
    EXPECT_EQ(captured[i].frame_count, i + 1);
    EXPECT_EQ(captured[i].data, units[i]);
    sdk::OdinCameraPacket p;
    p.format = stream.mode.format;
    p.payload = captured[i].data;
    decoded += receiver.decode(p, captured[i].header).size();
  }
  EXPECT_EQ(decoded, 8u);
}
INSTANTIATE_TEST_SUITE_P(Video, EncodedContinuity, testing::Bool());

// Block inside the decoder boundary while SDK callbacks and other workers can
// invalidate the packet. The guard must die before Running joins its workers.
struct DecodeGate {
  std::atomic<unsigned> entered{0};
  std::atomic<bool> released{false};
};
struct ReleaseDecodeOnExit {
  std::shared_ptr<DecodeGate> gate;
  ~ReleaseDecodeOnExit() {
    gate->released = true;
  }
};
class BlockingDecoder : public decoding::Backend {
 public:
  explicit BlockingDecoder(std::shared_ptr<DecodeGate> gate) : gate_(std::move(gate)) {}
  void send(const std::vector<uint8_t>&, int64_t token) override {
    token_ = token;
  }
  std::vector<decoding::Picture> receive() override {
    if (!token_)
      return {};
    ++gate_->entered;
    while (!gate_->released)
      ros::WallDuration(.001).sleep();
    decoding::Picture picture;
    picture.token = token_;
    picture.image.width = picture.image.height = 2;
    picture.image.encoding = "bgr8";
    picture.image.step = 6;
    picture.image.data.resize(12);
    token_ = 0;
    return {std::move(picture)};
  }

 private:
  std::shared_ptr<DecodeGate> gate_;
  int64_t token_ = 0;
};
decoding::Discovery blockingDecoder(const std::shared_ptr<DecodeGate>& gate) {
  return [gate](sdk::OdinDataFormat) {
    return std::vector<decoding::Candidate>{{"blocking-test", "test", false, [gate] {
                                               return std::make_unique<BlockingDecoder>(gate);
                                             }}};
  };
}
sdk::OdinCameraPacket camera(uint32_t sequence, uint64_t timestamp) {
  sdk::OdinCameraPacket packet;
  packet.frame_count = sequence;
  packet.timestamp = timestamp;
  packet.format = sdk::OdinDataFormat::kImageMjpeg;
  packet.payload = {0xff, 0xd8, 0xff, 0xd9};
  return packet;
}

bool diagnosticMatches(const Inbox<diagnostic_msgs::DiagnosticArray>& inbox,
                       const std::string& name, uint8_t level, const std::string& message) {
  const auto arrays = inbox.values();
  for (auto array = arrays.rbegin(); array != arrays.rend(); ++array)
    for (const auto& status : array->status)
      if (status.name.find(name + "/driver") != std::string::npos)
        return status.level == level && status.message.find(message) != std::string::npos &&
               (message.find("decoding on CPU") != std::string::npos ||
                status.message.find("decoding on CPU") == std::string::npos);
  return false;
}

TEST(Runtime, CpuFallbackWarnsAndHardwareRecoveryClearsCurrentWarning) {
  auto cfg = settings();
  cfg.name = "decoder_diagnostics";
  cfg.streams.at(Channel::kRawPoint).enabled = true;
  cfg.streams.at(Channel::kImage0).enabled = true;
  cfg.streams.at(Channel::kImage0).mode.format = sdk::OdinDataFormat::kImageMjpeg;
  auto available = std::make_shared<std::atomic<bool>>(false);
  auto gate = std::make_shared<DecodeGate>();
  gate->released = true;
  decoding::Discovery discovery = [available, gate](sdk::OdinDataFormat) {
    return std::vector<decoding::Candidate>{
        {"jetson/test", "test", true, [available, gate] {
           if (!*available)
             throw std::runtime_error("test hardware unavailable");
           return std::make_unique<BlockingDecoder>(gate);
         }},
        {"cpu/test", "test", false, [gate] { return std::make_unique<BlockingDecoder>(gate); }}};
  };
  Running device(cfg, discovery);
  Inbox<diagnostic_msgs::DiagnosticArray> diagnostics(device.node, "/diagnostics");
  Inbox<sensor_msgs::Image> images(device.node, "camera0/image_raw");
  ASSERT_TRUE(device.ready());
  device.sdk->emit(Channel::kRawPoint, cloud(100000, 100000000000ULL));
  device.sdk->emit(Channel::kImage0, camera(100000, 100000000000ULL));
  ASSERT_TRUE(until([&] { return !images.values().empty(); }));
  ASSERT_TRUE(until([&] { return diagnosticMatches(diagnostics, cfg.name,
      diagnostic_msgs::DiagnosticStatus::WARN, "camera0 decoding on CPU"); }));
  *available = true;
  // A real device-clock epoch reset immediately re-probes hardware without
  // sleeping through the separately covered 30-second candidate cooldown.
  device.sdk->emit(Channel::kRawPoint, cloud(1, 1000000000ULL));
  device.sdk->emit(Channel::kImage0, camera(1, 1000000000ULL));
  ASSERT_TRUE(until([&] { return images.values().size() == 2; }));
  ASSERT_TRUE(until([&] { return diagnosticMatches(diagnostics, cfg.name,
      diagnostic_msgs::DiagnosticStatus::OK, "running"); }));
}

TEST(Runtime, EncodedOnlyAndUninitializedCameraDoNotWarnAboutCpuFallback) {
  auto cfg = settings();
  cfg.name = "encoded_diagnostics";
  cfg.streams.at(Channel::kImage0).enabled = true;
  cfg.streams.at(Channel::kImage0).publish_decoded = false;
  cfg.streams.at(Channel::kImage0).mode.format = sdk::OdinDataFormat::kImageMjpeg;
  cfg.streams.at(Channel::kImage1).enabled = true;
  Running device(cfg, software());
  Inbox<diagnostic_msgs::DiagnosticArray> diagnostics(device.node, "/diagnostics");
  ASSERT_TRUE(device.ready());
  device.sdk->emit(Channel::kImage0, camera(1, 1000000000ULL));
  ASSERT_TRUE(until([&] { return diagnosticMatches(diagnostics, cfg.name,
      diagnostic_msgs::DiagnosticStatus::WARN, "camera1 has no recent data"); }));
}

TEST(Runtime, RawNv12ConversionDoesNotWarnAboutCpuDecoding) {
  auto cfg = settings();
  cfg.name = "nv12_diagnostics";
  cfg.streams.at(Channel::kImage0).enabled = true;
  cfg.streams.at(Channel::kImage0).mode.format = sdk::OdinDataFormat::kImageNv12;
  cfg.streams.at(Channel::kImage0).mode.width = 2;
  cfg.streams.at(Channel::kImage0).mode.height = 2;
  Running device(cfg, software());
  Inbox<diagnostic_msgs::DiagnosticArray> diagnostics(device.node, "/diagnostics");
  Inbox<sensor_msgs::Image> images(device.node, "camera0/image_raw");
  ASSERT_TRUE(device.ready());
  auto packet = camera(1, 1000000000ULL);
  packet.format = sdk::OdinDataFormat::kImageNv12;
  packet.width = packet.height = 2;
  packet.payload = {16, 16, 16, 16, 128, 128};
  device.sdk->emit(Channel::kImage0, packet);
  ASSERT_TRUE(until([&] { return !images.values().empty(); }));
  ASSERT_TRUE(until([&] { return diagnosticMatches(diagnostics, cfg.name,
      diagnostic_msgs::DiagnosticStatus::OK, "running"); }));
}

bool channelMatches(const odin2_ros_driver::DeviceStatus& status, Channel channel,
                    const std::function<bool(const odin2_ros_driver::ChannelStatus&)>& match) {
  const auto found =
      std::find_if(status.channels.begin(), status.channels.end(),
                   [&](const auto& c) { return c.channel == static_cast<uint8_t>(channel); });
  return found != status.channels.end() && match(*found);
}

TEST(Runtime, MjpegCompressedRateDoesNotThrottleEncodedOrDoubleCountPublishedPackets) {
  auto cfg = settings();
  auto& stream = cfg.streams.at(Channel::kImage0);
  stream.enabled = true;
  stream.publish_decoded = false;
  stream.publish_rate = 1;
  Running device(cfg);
  Inbox<odin2_ros_driver::EncodedFrame> encoded(device.node, "camera0/encoded");
  Inbox<sensor_msgs::CompressedImage> compressed(device.node, "camera0/image_raw/compressed");
  ASSERT_TRUE(device.ready());
  ASSERT_TRUE(until([&] {
    return encoded.subscriber.getNumPublishers() && compressed.subscriber.getNumPublishers();
  }));
  for (unsigned i = 0; i < 6; ++i) {
    device.sdk->emit(Channel::kImage0, camera(i + 1, 1000000000ULL + i * 40000000));
    ros::WallDuration(.04).sleep();
  }
  ASSERT_TRUE(until([&] { return encoded.values().size() == 6; }));
  ASSERT_TRUE(until([&] { return !compressed.values().empty(); }));
  EXPECT_EQ(compressed.values().size(), 1u);
  EXPECT_EQ(compressed.values()[0].data, encoded.values()[0].data);
  ASSERT_TRUE(until([&] {
    return channelMatches(device.status.values().back(), Channel::kImage0,
                          [](const auto& c) { return c.published == 6 && c.decoded_frames == 0; });
  }));
}

TEST(Runtime, EpochResetFencesBothInFlightDecodersAndReplacesQueuedRawPacket) {
  auto cfg = settings();
  cfg.worker_threads = 2;
  cfg.streams.at(Channel::kRawPoint).enabled = true;
  for (auto channel : {Channel::kImage0, Channel::kImage1}) {
    auto& stream = cfg.streams.at(channel);
    stream.enabled = true;
    stream.mode.format = sdk::OdinDataFormat::kImageMjpeg;
  }
  auto gate = std::make_shared<DecodeGate>();
  Running device(cfg, blockingDecoder(gate));
  ReleaseDecodeOnExit release{gate};
  Inbox<sensor_msgs::PointCloud2> points(device.node, "points_raw");
  Inbox<sensor_msgs::Image> left(device.node, "camera0/image_raw");
  Inbox<sensor_msgs::Image> right(device.node, "camera1/image_raw");
  Inbox<odin2_ros_driver::EncodedFrame> left_encoded(device.node, "camera0/encoded");
  Inbox<odin2_ros_driver::EncodedFrame> right_encoded(device.node, "camera1/encoded");
  ASSERT_TRUE(device.ready());
  ASSERT_TRUE(until([&] {
    return points.subscriber.getNumPublishers() && left.subscriber.getNumPublishers() &&
           right.subscriber.getNumPublishers() && left_encoded.subscriber.getNumPublishers() &&
           right_encoded.subscriber.getNumPublishers();
  }));
  const auto initial = device.status.values().back();
  device.sdk->emit(Channel::kRawPoint, cloud(100000, 100000000000ULL));
  ASSERT_TRUE(until([&] { return points.values().size() == 1; }));
  device.sdk->emit(Channel::kImage0, camera(100000, 100000000000ULL));
  device.sdk->emit(Channel::kImage1, camera(100000, 100000000000ULL));
  ASSERT_TRUE(until([&] { return gate->entered == 2; }));

  // Both workers are occupied, so this old raw packet must remain queued.
  device.sdk->emit(Channel::kRawPoint, cloud(100001, 100100000000ULL));
  ASSERT_TRUE(until([&] {
    return channelMatches(device.status.values().back(), Channel::kRawPoint,
                          [](const auto& c) { return c.received == 2 && c.queue_depth == 1; });
  }));
  // Ingress commits the reset synchronously without waiting for either worker.
  device.sdk->emit(Channel::kRawPoint, cloud(1, 1000000000ULL));
  ASSERT_TRUE(until([&] {
    const auto latest = device.status.values().back();
    return latest.clock_resets == initial.clock_resets + 1 &&
           channelMatches(latest, Channel::kRawPoint,
                          [](const auto& c) { return c.received == 3 && c.queue_depth == 1; });
  }));
  device.sdk->emit(Channel::kImage0, camera(1, 1000000000ULL));
  device.sdk->emit(Channel::kImage1, camera(1, 1000000000ULL));
  gate->released = true;
  ASSERT_TRUE(until([&] {
    const auto l = left.values(), r = right.values();
    const auto le = left_encoded.values(), re = right_encoded.values();
    const auto p = points.values();
    return !l.empty() && !r.empty() && !le.empty() && le.back().frame_count == 1 && !re.empty() &&
           re.back().frame_count == 1 && l.back().header.stamp == le.back().header.stamp &&
           r.back().header.stamp == re.back().header.stamp && p.size() >= 2 &&
           cloudY(p.back()) == .001f;
  }));
  EXPECT_EQ(left.values().size(), 1u);
  EXPECT_EQ(right.values().size(), 1u);
  EXPECT_EQ(left_encoded.values().size(), 1u);
  EXPECT_EQ(right_encoded.values().size(), 1u);
  ASSERT_EQ(points.values().size(), 2u);
  EXPECT_FLOAT_EQ(cloudY(points.values().back()), .001f);
  EXPECT_EQ(device.status.values().back().clock_resets, initial.clock_resets + 1);
  EXPECT_EQ(device.status.values().back().reconnects, initial.reconnects);
}

TEST(Runtime, ForwardSequenceGapFencesInFlightDecodeAndPublishesFreshIndependentFrame) {
  auto cfg = settings();
  auto& stream = cfg.streams.at(Channel::kImage0);
  stream.enabled = true;
  stream.mode.format = sdk::OdinDataFormat::kImageMjpeg;
  auto gate = std::make_shared<DecodeGate>();
  Running device(cfg, blockingDecoder(gate));
  ReleaseDecodeOnExit release{gate};
  Inbox<sensor_msgs::Image> images(device.node, "camera0/image_raw");
  Inbox<odin2_ros_driver::EncodedFrame> encoded(device.node, "camera0/encoded");
  ASSERT_TRUE(device.ready());
  ASSERT_TRUE(until([&] {
    return images.subscriber.getNumPublishers() && encoded.subscriber.getNumPublishers();
  }));
  const auto initial = device.status.values().back();
  device.sdk->emit(Channel::kImage0, camera(10, 10000000000ULL));
  ASSERT_TRUE(until([&] { return gate->entered == 1; }));
  device.sdk->emit(Channel::kImage0, camera(12, 10100000000ULL));
  ASSERT_TRUE(until([&] {
    return channelMatches(device.status.values().back(), Channel::kImage0,
                          [](const auto& c) { return c.sequence_gaps == 1 && c.queue_depth == 1; });
  }));
  gate->released = true;
  ASSERT_TRUE(until([&] {
    const auto i = images.values();
    const auto e = encoded.values();
    return !i.empty() && !e.empty() && e.back().frame_count == 12 &&
           i.back().header.stamp == e.back().header.stamp;
  }));
  EXPECT_EQ(images.values().size(), 1u);
  EXPECT_EQ(encoded.values().size(), 1u);
  EXPECT_EQ(device.status.values().back().clock_resets, initial.clock_resets);
  EXPECT_EQ(device.status.values().back().reconnects, initial.reconnects);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "runtime_regressions");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  return RUN_ALL_TESTS();
}
