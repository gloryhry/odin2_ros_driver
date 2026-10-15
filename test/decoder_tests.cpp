#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "odin2_ros_driver/decoder.hpp"

using namespace odin_driver;
namespace {
struct FakeState {
  int opens = 0, sends = 0;
  bool open_failure = false, runtime_failure = false, corrupt = false;
  bool delayed = false, ready = false;
  size_t receive_limit = 0;
  std::vector<uint8_t> bytes;
};
class FakeBackend : public decoding::Backend {
 public:
  explicit FakeBackend(std::shared_ptr<FakeState> state) : state_(std::move(state)) {}
  bool asynchronous() const override {
    return state_->delayed;
  }
  void send(const std::vector<uint8_t>& bytes, int64_t token) override {
    ++state_->sends;
    state_->bytes = bytes;
    if (state_->runtime_failure)
      throw decoding::Error(decoding::Failure::backend, "device lost");
    if (state_->corrupt)
      throw decoding::Error(decoding::Failure::bitstream, "bad bitstream");
    tokens_.push_back(token);
  }
  std::vector<decoding::Picture> receive() override {
    if (state_->runtime_failure)
      throw decoding::Error(decoding::Failure::backend, "download failed");
    if (state_->delayed && !state_->ready)
      return {};
    std::vector<decoding::Picture> out;
    const auto count =
        state_->receive_limit ? std::min(tokens_.size(), state_->receive_limit) : tokens_.size();
    for (size_t i = 0; i < count; ++i) {
      decoding::Picture picture;
      picture.token = tokens_[i];
      picture.image.width = picture.image.height = 2;
      picture.image.step = 6;
      picture.image.encoding = "bgr8";
      picture.image.data.resize(12);
      out.push_back(std::move(picture));
    }
    tokens_.erase(tokens_.begin(), tokens_.begin() + count);
    return out;
  }

 private:
  std::shared_ptr<FakeState> state_;
  std::vector<int64_t> tokens_;
};
decoding::Candidate candidate(const std::string& name, bool hardware,
                              std::shared_ptr<FakeState> state) {
  return {name, "test-device", hardware, [state]() -> std::unique_ptr<decoding::Backend> {
            ++state->opens;
            if (state->open_failure)
              throw decoding::Error(decoding::Failure::backend, "permission denied");
            return std::make_unique<FakeBackend>(state);
          }};
}
sdk::OdinCameraPacket jpeg() {
  sdk::OdinCameraPacket packet;
  packet.format = sdk::OdinDataFormat::kImageMjpeg;
  packet.payload = {0xff, 0xd8, 0xff, 0xd9};
  return packet;
}
struct Harness {
  std::shared_ptr<FakeState> first = std::make_shared<FakeState>();
  std::shared_ptr<FakeState> second = std::make_shared<FakeState>();
  std::shared_ptr<FakeState> cpu = std::make_shared<FakeState>();
  std::chrono::steady_clock::time_point now{};
  Decoder decoder{[this](sdk::OdinDataFormat) {
                    return std::vector<decoding::Candidate>{candidate("first", true, first),
                                                            candidate("second", true, second),
                                                            candidate("cpu", false, cpu)};
                  },
                  [this] { return now; }};
};
}  // namespace
TEST(HardwareDecoder, FirstHardwareSuccessNeverOpensCpu) {
  Harness h;
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  EXPECT_TRUE(h.decoder.info().hardware_active);
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_EQ(h.second->opens + h.cpu->opens, 0);
}
TEST(HardwareDecoder, OpenAndRuntimeFailureTryOtherHardwareBeforeCpu) {
  Harness h;
  h.first->open_failure = true;
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_EQ(h.cpu->opens, 0);
  h.second->runtime_failure = true;
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "cpu");
  EXPECT_FALSE(h.decoder.info().hardware_active);
  EXPECT_EQ(h.decoder.info().failures.size(), 2);
  EXPECT_NE(h.decoder.info().fallback_reason.find("device lost"), std::string::npos);
}
TEST(HardwareDecoder, CooldownDoesNotProbeEveryFrameAndRecoversAtThirtySeconds) {
  Harness h;
  h.first->open_failure = h.second->open_failure = true;
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  h.first->open_failure = false;
  h.now += std::chrono::seconds(29);
  h.decoder.decode(jpeg(), {});
  EXPECT_EQ(h.first->opens, 1);
  h.now += std::chrono::seconds(1);
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  EXPECT_EQ(h.first->opens, 2);
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_TRUE(h.decoder.info().fallback_reason.empty());
}
TEST(HardwareDecoder, CorruptInputDoesNotBlacklistHardware) {
  Harness h;
  h.first->corrupt = true;
  EXPECT_THROW(h.decoder.decode(jpeg(), {}), decoding::Error);
  EXPECT_TRUE(h.decoder.info().failures.empty());
  h.first->corrupt = false;
  ASSERT_EQ(h.decoder.decode(jpeg(), {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_EQ(h.cpu->opens, 0);
}
TEST(HardwareDecoder, AsyncFinalFrameKeepsOriginalContextWithoutAnotherPacket) {
  Harness h;
  h.first->delayed = true;
  DecodeContext context;
  context.header.stamp = ros::Time(12, 34);
  context.header.frame_id = "left";
  context.generation = 7;
  context.decode_generation = 8;
  ASSERT_TRUE(h.decoder.submit(jpeg(), context).empty());
  EXPECT_TRUE(h.decoder.pending());
  EXPECT_FALSE(h.decoder.info().hardware_active);
  h.first->ready = true;
  auto frames = h.decoder.poll();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames[0].image.header.stamp, context.header.stamp);
  EXPECT_EQ(frames[0].context.generation, 7);
  EXPECT_EQ(frames[0].context.decode_generation, 8);
  EXPECT_FALSE(h.decoder.pending());
  EXPECT_TRUE(h.decoder.info().hardware_active);
}
TEST(HardwareDecoder, ResetDiscardsOldAsyncFramesAndReprobesAfterReconnect) {
  Harness h;
  h.first->delayed = true;
  h.decoder.submit(jpeg(), {});
  h.decoder.reset();
  h.first->ready = true;
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_FALSE(h.decoder.pending());
  ASSERT_EQ(h.decoder.submit(jpeg(), {}).size(), 1);
  EXPECT_EQ(h.first->opens, 2);
}
TEST(HardwareDecoder, AsyncHardwareFailureRetriesFinalIndependentFrameWithoutNewInput) {
  Harness h;
  h.first->delayed = true;
  DecodeContext context;
  context.header.stamp = ros::Time(77, 88);
  ASSERT_TRUE(h.decoder.submit(jpeg(), context).empty());
  h.first->runtime_failure = true;
  const auto frames = h.decoder.poll();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames[0].image.header.stamp, context.header.stamp);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_EQ(h.cpu->opens, 0);
}
TEST(HardwareDecoder, VideoFailureWaitsForKeyframeAndPreservesParameters) {
  Harness h;
  sdk::OdinCameraPacket key;
  key.format = sdk::OdinDataFormat::kImageH264;
  key.payload = {0, 0, 1, 0x67, 0x80, 0, 0, 1, 0x68, 0x80, 0, 0, 1, 0x65, 0x80};
  ASSERT_EQ(h.decoder.decode(key, {}).size(), 1);
  sdk::OdinCameraPacket inter = key;
  inter.payload = {0, 0, 1, 0x41, 0x80};
  h.first->runtime_failure = true;
  EXPECT_THROW(h.decoder.decode(inter, {}), decoding::Error);
  EXPECT_TRUE(h.decoder.decode(inter, {}).empty());
  key.payload = {0, 0, 1, 0x65, 0x80};
  ASSERT_EQ(h.decoder.decode(key, {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_NE(std::find(h.second->bytes.begin(), h.second->bytes.end(), 0x67), h.second->bytes.end());
  EXPECT_NE(std::find(h.second->bytes.begin(), h.second->bytes.end(), 0x68), h.second->bytes.end());
}
TEST(HardwareDecoder, ReconnectAndFormatChangeClearHardwareCooldown) {
  Harness h;
  h.first->open_failure = h.second->open_failure = true;
  h.decoder.decode(jpeg(), {});
  h.first->open_failure = false;
  h.decoder.reset();
  h.decoder.decode(jpeg(), {});
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_EQ(h.first->opens, 2);
  auto packet = jpeg();
  packet.format = sdk::OdinDataFormat::kImageH264;
  packet.payload = {0, 0, 1, 0x65, 0x80};
  h.decoder.decode(packet, {});
  EXPECT_EQ(h.first->opens, 3);
}
TEST(HardwareDecoder, Nv12NeverDiscoversCodecBackends) {
  Harness h;
  sdk::OdinCameraPacket packet;
  packet.format = sdk::OdinDataFormat::kImageNv12;
  packet.width = packet.height = 2;
  packet.payload.resize(6, 128);
  ASSERT_EQ(h.decoder.decode(packet, {}).size(), 1);
  EXPECT_EQ(h.first->opens + h.second->opens + h.cpu->opens, 0);
  EXPECT_EQ(h.decoder.info().backend, "raw/nv12-conversion");
}
TEST(HardwareDecoder, ResolutionAndParameterChangesImmediatelyReprobeHardware) {
  Harness h;
  h.first->open_failure = h.second->open_failure = true;
  auto packet = jpeg();
  packet.width = packet.height = 16;
  h.decoder.decode(packet, {});
  h.first->open_failure = false;
  packet.width = packet.height = 256;
  ASSERT_EQ(h.decoder.decode(packet, {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_EQ(h.first->opens, 2);

  packet.format = sdk::OdinDataFormat::kImageH264;
  packet.payload = {0, 0, 1, 0x67, 0x80, 0, 0, 1, 0x68, 0x80, 0, 0, 1, 0x65, 0x80};
  h.first->open_failure = true;
  h.decoder.decode(packet, {});
  EXPECT_EQ(h.decoder.info().backend, "cpu");
  h.first->open_failure = false;
  packet.payload = {0, 0, 1, 0x67, 0x81};
  EXPECT_TRUE(h.decoder.decode(packet, {}).empty());
  packet.payload = {0, 0, 1, 0x41, 0x80};
  EXPECT_TRUE(h.decoder.decode(packet, {}).empty());
  packet.payload = {0, 0, 1, 0x65, 0x80};
  ASSERT_EQ(h.decoder.decode(packet, {}).size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "first");
  EXPECT_NE(std::find(h.first->bytes.begin(), h.first->bytes.end(), 0x81), h.first->bytes.end());
}
TEST(HardwareDecoder, StalledAsyncHardwareFallsBackAndKeepsQueuesBounded) {
  Harness h;
  h.first->delayed = true;
  for (int i = 0; i < 8; ++i)
    EXPECT_TRUE(h.decoder.submit(jpeg(), {}).empty());
  h.now += std::chrono::seconds(6);
  auto frames = h.decoder.poll();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_FALSE(h.decoder.pending());
  EXPECT_NE(h.decoder.info().failures.front().find("no progress"), std::string::npos);
}
class IndependentAsyncTail : public ::testing::TestWithParam<int> {};
TEST_P(IndependentAsyncTail, RecoversAtFiveSecondsWithoutAnotherInput) {
  Harness h;
  h.first->delayed = true;
  DecodeContext context;
  context.header.frame_id = "left";
  context.generation = 17;
  context.decode_generation = 23;
  for (int i = 0; i < GetParam(); ++i) {
    context.header.stamp = ros::Time(20 + i, 42);
    ASSERT_TRUE(h.decoder.submit(jpeg(), context).empty());
  }
  h.now += std::chrono::milliseconds(4999);
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_TRUE(h.decoder.pending());
  EXPECT_TRUE(h.decoder.info().failures.empty());
  h.now += std::chrono::milliseconds(1);
  const auto frames = h.decoder.poll();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames[0].image.header.stamp, context.header.stamp);
  EXPECT_EQ(frames[0].image.header.frame_id, context.header.frame_id);
  EXPECT_EQ(frames[0].context.generation, context.generation);
  EXPECT_EQ(frames[0].context.decode_generation, context.decode_generation);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_FALSE(h.decoder.pending());
  EXPECT_EQ(h.cpu->opens, 0);
}
INSTANTIATE_TEST_SUITE_P(MjpegPendingCounts, IndependentAsyncTail,
                         ::testing::Values(1, 2, 3, 4, 5, 6, 7));
TEST(HardwareDecoder, NewPendingFrameGetsFullTimeoutAfterLongIdle) {
  Harness h;
  h.first->delayed = h.first->ready = true;
  ASSERT_EQ(h.decoder.submit(jpeg(), {}).size(), 1);
  EXPECT_FALSE(h.decoder.pending());
  h.now += std::chrono::seconds(60);
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_TRUE(h.decoder.info().failures.empty());

  h.first->ready = false;
  ASSERT_TRUE(h.decoder.submit(jpeg(), {}).empty());
  EXPECT_TRUE(h.decoder.pending());
  EXPECT_EQ(h.decoder.info().backend, "first");
  h.now += std::chrono::milliseconds(4999);
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_TRUE(h.decoder.info().failures.empty());
  h.now += std::chrono::milliseconds(1);
  EXPECT_EQ(h.decoder.poll().size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "second");
}
TEST(HardwareDecoder, ActualOutputRefreshesTimeoutWhileOtherFramesRemainPending) {
  Harness h;
  h.first->delayed = true;
  h.first->receive_limit = 1;
  ASSERT_TRUE(h.decoder.submit(jpeg(), {}).empty());
  ASSERT_TRUE(h.decoder.submit(jpeg(), {}).empty());
  h.now += std::chrono::seconds(4);
  h.first->ready = true;
  ASSERT_EQ(h.decoder.poll().size(), 1);
  EXPECT_TRUE(h.decoder.pending());
  h.first->ready = false;
  h.now += std::chrono::seconds(4);
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_TRUE(h.decoder.info().failures.empty());
  h.now += std::chrono::seconds(1);
  EXPECT_EQ(h.decoder.poll().size(), 1);
  EXPECT_EQ(h.decoder.info().backend, "second");
}
class IndependentVideoTail : public ::testing::TestWithParam<sdk::OdinDataFormat> {};
TEST_P(IndependentVideoTail, SingleKeyframeRecoversWithoutAnotherInput) {
  Harness h;
  h.first->delayed = true;
  sdk::OdinCameraPacket key;
  key.format = GetParam();
  if (key.format == sdk::OdinDataFormat::kImageH264)
    key.payload = {0, 0, 1, 0x67, 0x80, 0, 0, 1, 0x68, 0x80, 0, 0, 1, 0x65, 0x80};
  else
    key.payload = {0, 0, 1, 0x40, 0x80, 0, 0, 1, 0x42, 0x80,
                   0, 0, 1, 0x44, 0x80, 0, 0, 1, 0x26, 0x80};
  DecodeContext context;
  context.header.stamp = ros::Time(77, 88);
  context.generation = 11;
  context.decode_generation = 19;
  ASSERT_TRUE(h.decoder.submit(key, context).empty());
  h.now += std::chrono::seconds(5);
  const auto frames = h.decoder.poll();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames[0].image.header.stamp, context.header.stamp);
  EXPECT_EQ(frames[0].context.generation, context.generation);
  EXPECT_EQ(frames[0].context.decode_generation, context.decode_generation);
  EXPECT_EQ(h.decoder.info().backend, "second");
  EXPECT_TRUE(std::search(h.second->bytes.begin(), h.second->bytes.end(), key.payload.begin(),
                          key.payload.end()) != h.second->bytes.end());
  EXPECT_EQ(h.cpu->opens, 0);
  EXPECT_FALSE(h.decoder.pending());
}
TEST_P(IndependentVideoTail, DependentReorderTailDoesNotTriggerPrematureRecovery) {
  for (int pending = 1; pending <= 7; ++pending) {
    SCOPED_TRACE(pending);
    Harness h;
    h.first->delayed = h.first->ready = true;
    sdk::OdinCameraPacket packet;
    packet.format = GetParam();
    packet.payload = {
        0, 0, 1, static_cast<uint8_t>(GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x65 : 0x26),
        0x80};
    ASSERT_EQ(h.decoder.submit(packet, {}).size(), 1);
    h.first->ready = false;
    packet.payload[3] = GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x41 : 0x02;
    for (int i = 0; i < pending; ++i) {
      DecodeContext context;
      context.header.stamp = ros::Time(30 + i, 0);
      ASSERT_TRUE(h.decoder.submit(packet, context).empty());
    }
    h.now += std::chrono::seconds(60);
    EXPECT_TRUE(h.decoder.poll().empty());
    EXPECT_TRUE(h.decoder.pending());
    EXPECT_TRUE(h.decoder.info().failures.empty());
    EXPECT_EQ(h.second->opens + h.cpu->opens, 0);
    h.first->ready = true;
    const auto frames = h.decoder.poll();
    ASSERT_EQ(frames.size(), pending);
    for (int i = 0; i < pending; ++i)
      EXPECT_EQ(frames[i].image.header.stamp, ros::Time(30 + i, 0));
    EXPECT_FALSE(h.decoder.pending());
  }
}
TEST_P(IndependentVideoTail, KeyframeWithDependentTailWaitsForNormalReordering) {
  Harness h;
  h.first->delayed = true;
  sdk::OdinCameraPacket packet;
  packet.format = GetParam();
  packet.payload = {
      0, 0, 1, static_cast<uint8_t>(GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x65 : 0x26),
      0x80};
  ASSERT_TRUE(h.decoder.submit(packet, {}).empty());
  packet.payload[3] = GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x41 : 0x02;
  ASSERT_TRUE(h.decoder.submit(packet, {}).empty());
  h.now += std::chrono::seconds(60);
  EXPECT_TRUE(h.decoder.poll().empty());
  EXPECT_TRUE(h.decoder.pending());
  EXPECT_TRUE(h.decoder.info().failures.empty());
  h.first->ready = true;
  EXPECT_EQ(h.decoder.poll().size(), 2);
}
TEST_P(IndependentVideoTail, DependentFramesCannotGrowBeyondSixtyFourPending) {
  Harness h;
  h.first->delayed = true;
  sdk::OdinCameraPacket packet;
  packet.format = GetParam();
  packet.payload = {
      0, 0, 1, static_cast<uint8_t>(GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x65 : 0x26),
      0x80};
  ASSERT_TRUE(h.decoder.submit(packet, {}).empty());
  packet.payload[3] = GetParam() == sdk::OdinDataFormat::kImageH264 ? 0x41 : 0x02;
  for (int i = 1; i < 64; ++i)
    ASSERT_TRUE(h.decoder.submit(packet, {}).empty());
  EXPECT_EQ(h.first->sends, 64);
  EXPECT_THROW(h.decoder.submit(packet, {}), decoding::Error);
  EXPECT_EQ(h.first->sends, 64);
  EXPECT_FALSE(h.decoder.pending());
  EXPECT_EQ(h.second->opens + h.cpu->opens, 0);
  ASSERT_FALSE(h.decoder.info().failures.empty());
  EXPECT_NE(h.decoder.info().failures.back().find("64 pending"), std::string::npos);
}
INSTANTIATE_TEST_SUITE_P(VideoFormats, IndependentVideoTail,
                         ::testing::Values(sdk::OdinDataFormat::kImageH264,
                                           sdk::OdinDataFormat::kImageH265));
TEST(HardwareDecoder, RealHardwareWhenExplicitlyRequested) {
  if (!std::getenv("ODIN_TEST_REQUIRE_HARDWARE"))
    GTEST_SKIP();
  const char* root = std::getenv("ODIN_TEST_FIXTURES");
  ASSERT_NE(root, nullptr);
  for (const auto& entry : std::vector<std::pair<sdk::OdinDataFormat, std::string>>{
           {sdk::OdinDataFormat::kImageMjpeg, "hw-mjpeg.jpg"},
           {sdk::OdinDataFormat::kImageH264, "hw-h264.bin"},
           {sdk::OdinDataFormat::kImageH265, "hw-h265.bin"}}) {
    std::ifstream file(std::string(root) + "/" + entry.second, std::ios::binary);
    sdk::OdinCameraPacket packet;
    packet.format = entry.first;
    packet.payload = {std::istreambuf_iterator<char>(file), {}};
    Decoder decoder, right_decoder;
    DecodeContext left, right;
    left.header.frame_id = "left";
    left.header.stamp = ros::Time(11, 22);
    right.header.frame_id = "right";
    right.header.stamp = ros::Time(33, 44);
    auto frames = decoder.submit(packet, left);
    auto right_frames = right_decoder.submit(packet, right);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (frames.empty() && decoder.pending() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      frames = decoder.poll();
    }
    while (right_frames.empty() && right_decoder.pending() &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      right_frames = right_decoder.poll();
    }
    ASSERT_EQ(frames.size(), 1) << entry.second;
    ASSERT_EQ(right_frames.size(), 1) << entry.second;
    EXPECT_EQ(frames[0].image.header.frame_id, "left");
    EXPECT_EQ(frames[0].image.header.stamp, left.header.stamp);
    EXPECT_EQ(right_frames[0].image.header.frame_id, "right");
    EXPECT_EQ(right_frames[0].image.header.stamp, right.header.stamp);
    EXPECT_EQ(frames[0].image.width, 256);
    EXPECT_EQ(frames[0].image.height, 256);
    EXPECT_EQ(frames[0].image.encoding, "bgr8");
    EXPECT_GT(frames[0].image.data[2], 200);  // Red fixture, BGR order.
    EXPECT_LT(frames[0].image.data[0], 30);
    Decoder cpu_decoder(
        [](sdk::OdinDataFormat format) {
          auto candidates = decoding::discover(format);
          candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                          [](const auto& c) { return c.hardware; }),
                           candidates.end());
          return candidates;
        },
        [] { return std::chrono::steady_clock::now(); });
    auto cpu_frames = cpu_decoder.submit(packet, left);
    ASSERT_EQ(cpu_frames.size(), 1);
    for (size_t channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(frames[0].image.data[channel], cpu_frames[0].image.data[channel], 4);
    const auto info = decoder.info();
    EXPECT_TRUE(info.hardware_active) << entry.second << ": " << info.fallback_reason;
    EXPECT_TRUE(right_decoder.info().hardware_active) << right_decoder.info().fallback_reason;
    std::cout << entry.second << ": " << info.backend << " device=" << info.device << std::endl;
  }
}
