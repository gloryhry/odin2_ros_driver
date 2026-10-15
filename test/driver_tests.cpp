#include <gtest/gtest.h>

#include <cstring>
#include <fstream>
#include <limits>

#include "OdinFrameAssembler.hpp"
#include "OdinProtocol.hpp"
#include "odin2_ros_driver/conversion.hpp"
#include "odin2_ros_driver/decoder.hpp"
#include "odin2_ros_driver/sdk.hpp"
using namespace odin_driver;
template <class T>
T read(const std::vector<uint8_t>& bytes, size_t offset) {
  T v;
  std::memcpy(&v, bytes.data() + offset, sizeof(T));
  return v;
}
TEST(Conversion, RawUnitsFieldsAndInvalidPoints) {
  sdk::OdinPointCloudPacket p;
  sdk::OdinRawPoint<float> good;
  good.x = 1000;
  good.y = -2000;
  good.z = 500;
  good.intensity = 42;
  good.confidence = 200;
  p.points = {good, {}};
  auto out = rawCloud(p, {});
  EXPECT_FLOAT_EQ(read<float>(out.data, 0), 1);
  EXPECT_FLOAT_EQ(read<float>(out.data, 4), -2);
  EXPECT_FLOAT_EQ(read<float>(out.data, 8), .5);
  EXPECT_EQ(out.data[12], 42);
  EXPECT_EQ(out.data[13], 200);
  EXPECT_EQ(out.fields[4].name, "confidence");
  EXPECT_TRUE(std::isnan(read<float>(out.data, 18)));
  EXPECT_EQ(out.height, 1);
}
TEST(Conversion, RawOfficialLayoutAndRollingScanTime) {
  sdk::OdinPointCloudPacket p;
  sdk::OdinRawPoint<float> good;
  good.x = 1000;
  good.y = -2000;
  good.z = 500;
  good.intensity = 42;
  good.confidence = 200;
  p.points.assign(49152, good);
  p.points[1535] = {};  // Invalid points retain their original rolling-scan index.
  p.points[6143].z = std::numeric_limits<float>::infinity();
  std_msgs::Header h;
  h.stamp = ros::Time(123, 456);
  h.frame_id = "odin2/lidar";
  const auto out = rawCloud(p, h);
  ASSERT_EQ(out.fields.size(), 6);
  const std::vector<std::string> names = {"x", "y", "z", "intensity", "confidence", "offset_time"};
  const std::vector<uint32_t> offsets = {0, 4, 8, 12, 13, 14};
  const std::vector<uint8_t> types = {
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::FLOAT32,
      sensor_msgs::PointField::FLOAT32, sensor_msgs::PointField::UINT8,
      sensor_msgs::PointField::UINT8,   sensor_msgs::PointField::FLOAT32};
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_EQ(out.fields[i].name, names[i]);
    EXPECT_EQ(out.fields[i].offset, offsets[i]);
    EXPECT_EQ(out.fields[i].datatype, types[i]);
    EXPECT_EQ(out.fields[i].count, 1);
  }
  ASSERT_EQ(out.point_step, 18);
  EXPECT_EQ(out.width, p.points.size());
  EXPECT_EQ(out.height, 1);
  EXPECT_EQ(out.row_step, p.points.size() * 18);
  EXPECT_EQ(out.data.size(), out.row_step);
  EXPECT_FALSE(out.is_bigendian);
  EXPECT_FALSE(out.is_dense);
  EXPECT_EQ(out.header.stamp, h.stamp);
  EXPECT_EQ(out.header.frame_id, h.frame_id);
  const std::vector<std::pair<size_t, float>> scan_times = {
      {0, 0}, {1535, 0}, {1536, .00304414f}, {6144, .01217656f}, {49151, .09436834f}};
  for (const auto& sample : scan_times)
    EXPECT_NEAR(read<float>(out.data, sample.first * 18 + 14), sample.second, 1e-8);
  for (size_t i : {1535u, 6143u})
    for (size_t coordinate : {0u, 4u, 8u})
      EXPECT_TRUE(std::isnan(read<float>(out.data, i * 18 + coordinate)));
  EXPECT_FLOAT_EQ(read<float>(out.data, 1536 * 18), 1);
  EXPECT_FLOAT_EQ(read<float>(out.data, 6144 * 18), 1);
}
TEST(Conversion, SlamNegativeWorldCoordinatesAndColor) {
  sdk::OdinSlamPacket p;
  sdk::OdinSlamPoint<float> pt;
  pt.x = -1000;
  pt.y = 100;
  pt.r = 10;
  pt.g = 20;
  pt.b = 30;
  pt.a = 255;
  p.points = {pt};
  auto out = slamCloud(p, {});
  EXPECT_FLOAT_EQ(read<float>(out.data, 0), -1);
  EXPECT_EQ(read<uint32_t>(out.data, 12), 0xff0a141e);
}
TEST(Conversion, ImuAndOdomPreserveSdkMeaning) {
  sdk::OdinImuSample sample;
  sample.gyro_x = 1;
  sample.acc_z = 9.81;
  auto imu = imuMessage(sample, {});
  EXPECT_EQ(imu.orientation_covariance[0], -1);
  EXPECT_FLOAT_EQ(imu.linear_acceleration.z, 9.81);
  sdk::OdinOdomPacket p;
  sdk::OdinOdomData od;
  od.orient[3] = 2;
  for (int i = 0; i < 18; ++i)
    od.cov[i] = i;
  p.data = {od};
  auto raw = odomPacket(p, {}, 1);
  EXPECT_EQ(raw.samples[0].sdk_covariance[17], 17);
  auto standard = odomMessage(od, {}, defaults());
  EXPECT_DOUBLE_EQ(standard.pose.pose.orientation.w, 1);
  EXPECT_EQ(standard.pose.covariance[0], 1e6);
  EXPECT_EQ(standard.child_frame_id, "odin2/imu");
}
TEST(Time, CommonMappingAndRollback) {
  TimeMapper mapper;
  bool reset;
  auto a = mapper.map(Channel::kImu, 1000000000, ros::Time(100, 0), reset);
  ASSERT_TRUE(a);
  auto b = mapper.map(Channel::kRawPoint, 1100000000, ros::Time(100, 200000000), reset);
  ASSERT_TRUE(b);
  EXPECT_EQ(b->toNSec() - a->toNSec(), 100000000);
  EXPECT_FALSE(mapper.map(Channel::kRawPoint, 1090000000, ros::Time(100, 300000000), reset));
  auto c = mapper.map(Channel::kRawPoint, 1, ros::Time(101, 0), reset);
  EXPECT_TRUE(c);
  EXPECT_TRUE(reset);
}
TEST(Time, ZeroHostAndOffset) {
  TimeMapper m("offset", 100000000000LL);
  bool reset;
  EXPECT_FALSE(m.map(Channel::kImu, 1, ros::Time(), reset));
  auto t = m.map(Channel::kImu, 100, ros::Time(1, 0), reset);
  ASSERT_TRUE(t);
  EXPECT_EQ(t->toNSec(), 100000000100ULL);
}
TEST(Queue, BothBoundsAndOversizedPacket) {
  BoundedQueue q;
  q.configure(2, 100);
  Envelope e;
  e.bytes = 40;
  e.channel = Channel::kImu;
  EXPECT_EQ(q.push(e), 0);
  EXPECT_EQ(q.push(e), 0);
  EXPECT_EQ(q.push(e), 1);
  EXPECT_EQ(q.size(), 2);
  e.bytes = 101;
  EXPECT_EQ(q.push(e), 1);
  EXPECT_EQ(q.bytes(), 80);
  q.clear();
  EXPECT_EQ(q.size(), 0);
}
TEST(Config, CapabilityMaskAndUnsupportedAcquisitionParameters) {
  auto cfg = defaults();
  auto s = cfg.streams.at(Channel::kImage0);
  s.enabled = true;
  s.mode = {16, 16, 10, sdk::OdinDataFormat::kImageH265};
  sdk::OdinSensorCapability cap;
  cap.channel = Channel::kImage0;
  cap.modes.push_back({16, 16, 10, static_cast<sdk::OdinDataFormat>(15)});
  EXPECT_NO_THROW(validateStream(Channel::kImage0, s, {cap}));
  s.mode.width = 32;
  EXPECT_THROW(validateStream(Channel::kImage0, s, {cap}), std::invalid_argument);
  s = cfg.streams.at(Channel::kImu);
  s.mode.fps = 100;
  EXPECT_THROW(validateStream(Channel::kImu, s, {}), std::invalid_argument);
}
TEST(Decoder, Nv12AndMalformedPayload) {
  Decoder decoder;
  sdk::OdinCameraPacket p;
  p.format = sdk::OdinDataFormat::kImageNv12;
  p.width = p.height = 16;
  p.payload.resize(384, 128);
  auto images = decoder.decode(p, {});
  ASSERT_EQ(images.size(), 1);
  EXPECT_EQ(images[0].encoding, "bgr8");
  EXPECT_EQ(images[0].data.size(), 768);
  p.payload.pop_back();
  EXPECT_THROW(decoder.decode(p, {}), std::runtime_error);
}
TEST(Decoder, AllEncodedFixtures) {
  const char* root = std::getenv("ODIN_TEST_FIXTURES");
  ASSERT_NE(root, nullptr);
  for (auto pair : std::vector<std::pair<sdk::OdinDataFormat, std::string>>{
           {sdk::OdinDataFormat::kImageMjpeg, "mjpeg.jpg"},
           {sdk::OdinDataFormat::kImageH264, "h264.bin"},
           {sdk::OdinDataFormat::kImageH265, "h265.bin"}}) {
    std::ifstream file(std::string(root) + "/" + pair.second, std::ios::binary);
    sdk::OdinCameraPacket p;
    p.format = pair.first;
    p.payload = {std::istreambuf_iterator<char>(file), {}};
    p.timestamp = 123;
    Decoder d;
    std_msgs::Header h;
    h.stamp = ros::Time(5, 0);
    auto images = d.decode(p, h);
    ASSERT_EQ(images.size(), 1) << pair.second;
    EXPECT_EQ(images[0].width, 16);
    EXPECT_EQ(images[0].header.stamp, h.stamp);
  }
}
TEST(SdkProtocol, ReconstructedImuTimestamps) {
  sdk::OdinFrameAssembler assembler(1, sdk::OdinTransportMode::kUdp);
  std::vector<uint64_t> times;
  assembler.SetImuCallback([&](const sdk::OdinImuPacket& p) {
    times.push_back(p.timestamp);
    EXPECT_EQ(p.samples.size(), 1);
  });
  sdk::OdinDataFrameHeader header;
  header.sof = 0xad;
  header.version = 2;
  header.data_type = 0x20;
  header.flags = 1;
  header.timestamp = 1000000;
  header.interval_ns = 5000;
  header.payload_len = 48;
  header.element_count = 2;
  header.chunk_count = 1;
  std::vector<uint8_t> bytes(32 + 48);
  std::memcpy(bytes.data(), &header, 32);
  assembler.ProcessImuData(bytes.data(), bytes.size());
  ASSERT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 1000000);
  EXPECT_EQ(times[1], 1005000);
}

TEST(Conversion, EmptyAndNonfinite) {
  EXPECT_EQ(rawCloud({}, {}).width, 0);
  EXPECT_TRUE(slamCloud({}, {}).data.empty());
  sdk::OdinImuSample imu;
  imu.acc_x = std::numeric_limits<float>::infinity();
  EXPECT_THROW(imuMessage(imu, {}), std::runtime_error);
  sdk::OdinOdomData od;
  std::fill(std::begin(od.orient), std::end(od.orient), 0);
  EXPECT_THROW(odomMessage(od, {}, defaults()), std::runtime_error);
}
TEST(Time, ExtremeValuesAndRosJump) {
  TimeMapper mapped;
  bool reset;
  ASSERT_TRUE(mapped.map(Channel::kImu, 1, ros::Time(100, 0), reset));
  EXPECT_FALSE(mapped.map(Channel::kImu, INT64_MAX, ros::Time(100, 1), reset));
  EXPECT_TRUE(mapped.map(Channel::kRawPoint, 2, ros::Time(200, 0), reset));
  EXPECT_TRUE(reset);
  TimeMapper offset("offset", INT64_MAX);
  EXPECT_FALSE(offset.map(Channel::kImu, 1, ros::Time(1, 0), reset));
  TimeMapper negative("offset", INT64_MIN);
  EXPECT_FALSE(negative.map(Channel::kImu, 0, ros::Time(1, 0), reset));
}
std::vector<uint8_t> fixture(const std::string& name) {
  const char* root = std::getenv("ODIN_TEST_FIXTURES");
  if (!root)
    throw std::runtime_error("ODIN_TEST_FIXTURES is required");
  std::ifstream file(std::string(root) + "/" + name, std::ios::binary);
  return {std::istreambuf_iterator<char>(file), {}};
}
TEST(Decoder, SeparateParametersAndRecoveryAndStereoIsolation) {
  for (auto codec : {sdk::OdinDataFormat::kImageH264, sdk::OdinDataFormat::kImageH265}) {
    auto bytes = fixture(codec == sdk::OdinDataFormat::kImageH264 ? "h264.bin" : "h265.bin");
    std::vector<size_t> positions;
    for (size_t i = 0; i + 3 < bytes.size(); ++i) {
      if (bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 1) {
        positions.push_back(i > 0 && bytes[i - 1] == 0 ? i - 1 : i);
        i += 2;
      }
    }
    sdk::OdinCameraPacket parameters, picture;
    parameters.format = picture.format = codec;
    for (size_t i = 0; i < positions.size(); ++i) {
      auto start = positions[i], end = i + 1 < positions.size() ? positions[i + 1] : bytes.size();
      auto nal = start + (bytes[start + 2] == 1 ? 3 : 4);
      unsigned type =
          codec == sdk::OdinDataFormat::kImageH264 ? bytes[nal] & 31 : (bytes[nal] >> 1) & 63;
      bool param = codec == sdk::OdinDataFormat::kImageH264 ? type == 7 || type == 8
                                                            : type >= 32 && type <= 34;
      auto& target = param ? parameters.payload : picture.payload;
      target.insert(target.end(), bytes.begin() + start, bytes.begin() + end);
    }
    ASSERT_FALSE(parameters.payload.empty());
    ASSERT_TRUE(cameraKeyframe(picture));
    Decoder left, right;
    EXPECT_TRUE(left.decode(parameters, {}).empty());
    EXPECT_TRUE(right.decode(parameters, {}).empty());
    ASSERT_EQ(left.decode(picture, {}).size(), 1);
    left.reset(true);  // Queue/sequence loss must preserve parameter sets.
    sdk::OdinCameraPacket inter;
    inter.format = codec;
    inter.payload = codec == sdk::OdinDataFormat::kImageH264
                        ? std::vector<uint8_t>{0, 0, 1, 0x41, 0x80}
                        : std::vector<uint8_t>{0, 0, 1, 0x02, 1, 0x80};
    EXPECT_TRUE(left.decode(inter, {}).empty());
    ASSERT_EQ(left.decode(picture, {}).size(), 1);
    ASSERT_EQ(right.decode(picture, {}).size(), 1);
    sdk::OdinCameraPacket broken;
    broken.format = sdk::OdinDataFormat::kImageMjpeg;
    broken.payload = {1, 2, 3, 4};
    EXPECT_THROW(left.decode(broken, {}), std::runtime_error);
    ASSERT_EQ(right.decode(picture, {}).size(), 1);
  }
}
std::vector<uint8_t> wire(uint8_t type, const std::vector<uint8_t>& payload, uint16_t index = 0,
                          uint16_t chunks = 1, uint16_t elements = 1) {
  sdk::OdinDataFrameHeader h;
  h.sof = 0xad;
  h.version = 2;
  h.data_type = type;
  h.frame_seq = 7;
  h.timestamp = 12345;
  h.chunk_count = chunks;
  h.chunk_index = index;
  h.flags = index + 1 == chunks ? 1 : 0;
  h.element_count = elements;
  h.payload_len = payload.size();
  std::vector<uint8_t> bytes(32 + payload.size());
  std::memcpy(bytes.data(), &h, 32);
  std::copy(payload.begin(), payload.end(), bytes.begin() + 32);
  return bytes;
}
TEST(SdkProtocol, OriginalPointConversionAndOffset) {
  sdk::OdinFrameAssembler assembler(1, sdk::OdinTransportMode::kUdp);
  sdk::OdinRawPoint<uint16_t> point;
  point.x = 1000;
  point.y = 28000;
  point.z = 30500;
  point.intensity = 42;
  point.confidence = 200;
  std::vector<uint8_t> payload(sizeof(point));
  std::memcpy(payload.data(), &point, sizeof(point));
  bool called = false;
  assembler.SetPointCloudCallback([&](const sdk::OdinPointCloudPacket& packet) {
    called = true;
    EXPECT_FLOAT_EQ(packet.points[0].y, -2000);
    auto out = rawCloud(packet, {});
    EXPECT_FLOAT_EQ(read<float>(out.data, 0), 1);
    EXPECT_FLOAT_EQ(read<float>(out.data, 4), -2);
    EXPECT_FLOAT_EQ(read<float>(out.data, 8), .5);
  });
  auto bytes = wire(0, payload);
  assembler.ProcessRawPointData(bytes.data(), bytes.size());
  EXPECT_TRUE(called);
}
TEST(SdkProtocol, OriginalImageAssemblyWithTcpFragmentation) {
  for (auto codec : {sdk::OdinDataFormat::kImageMjpeg, sdk::OdinDataFormat::kImageNv12,
                     sdk::OdinDataFormat::kImageH264, sdk::OdinDataFormat::kImageH265}) {
    auto bytes = codec == sdk::OdinDataFormat::kImageNv12
                     ? std::vector<uint8_t>(384, 128)
                     : fixture(codec == sdk::OdinDataFormat::kImageMjpeg  ? "mjpeg.jpg"
                               : codec == sdk::OdinDataFormat::kImageH264 ? "h264.bin"
                                                                          : "h265.bin");
    sdk::OdinFrameAssembler assembler(1, sdk::OdinTransportMode::kTcp);
    assembler.SetImageFormat(codec);
    bool called = false;
    auto callback = [&](const sdk::OdinCameraPacket& p) {
      called = true;
      EXPECT_EQ(p.payload, bytes);
      EXPECT_EQ(p.format, codec);
      EXPECT_EQ(p.timestamp, 12345);
      EXPECT_EQ(p.frame_count, 7);
    };
    assembler.SetImageCallback(callback);
    assembler.SetH264Callback(callback);
    auto header = wire(0x10, bytes, 0, 1, bytes.size());
    auto send = [&](const uint8_t* data, size_t size) {
      if (codec == sdk::OdinDataFormat::kImageH264)
        assembler.ProcessH264Data(data, size);
      else
        assembler.ProcessImageData(data, size);
    };
    send(header.data(), 11);
    EXPECT_FALSE(called);
    send(header.data() + 11, header.size() - 11);
    EXPECT_TRUE(called);
  }
}
TEST(LiveSdk, UnreachableDeviceFailsWithoutCrashing) {
  auto api = makeLiveSdk();
  sdk::DiscoveredDevice d;
  d.model = "ODIN2";
  d.host_ip = "127.0.0.1";
  d.network.ip_address = "127.0.0.2";
  EXPECT_EQ(api->connect(d), 0);
  Command command;
  EXPECT_EQ(api->execute(0, command).code, sdk::OdinResult::kNotInitialized);
  EXPECT_EQ(api->start(0, Channel::kRawPoint, defaults().streams.at(Channel::kRawPoint), 1000).code,
            sdk::OdinResult::kNotInitialized);
  api->disconnect(0);
}
TEST(LiveSdk, StreamFailuresIncludeOperationChannelAndResult) {
  auto api = makeLiveSdk();
  auto settings = defaults();
  for (auto handle : {0u, 123456u}) {
    for (auto channel : {Channel::kRawPoint, Channel::kTelemetry}) {
      auto reply = api->start(handle, channel, settings.streams.at(channel), 1);
      EXPECT_EQ(reply.code, sdk::OdinResult::kNotInitialized);
      EXPECT_NE(reply.message.find(channelName(channel)), std::string::npos);
      EXPECT_NE(reply.message.find("not initialized"), std::string::npos);
      EXPECT_NE(reply.message.find("SDK result 1"), std::string::npos);
      EXPECT_NE(
          reply.message.find(channel == Channel::kTelemetry ? "StartTelemetry" : "StartStream"),
          std::string::npos);
    }
    auto reply = api->close(handle, Channel::kRawPoint);
    EXPECT_EQ(reply.code, sdk::OdinResult::kNotInitialized);
    EXPECT_NE(reply.message.find("CloseStream(raw_point)"), std::string::npos);
    EXPECT_NE(reply.message.find("SDK result 1"), std::string::npos);
  }
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
