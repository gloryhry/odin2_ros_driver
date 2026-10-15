#include "mock_sdk.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <set>
#include <thread>
namespace odin_driver {
namespace {
class MockSdk final : public SdkFacade {
  struct Device {
    std::mutex mutex;
    std::string sn;
    PacketCallback callback;
    std::map<Channel, StreamSettings> streams;
    std::atomic<bool> stopped{false};
    std::thread worker;
    uint8_t mode = 1, sensor = 0, sync = 0;
    bool world = false;
    bool queried = false;
    sdk::OdinLidarFilterParams filters;
    sdk::NetworkAttribute network;
    std::map<sdk::OdinDataFormat, std::vector<uint8_t>> fixtures;
  };
  std::mutex mutex_;
  std::map<uint32_t, std::shared_ptr<Device>> devices_;
  std::set<std::pair<std::string, std::string>> queried_;
  uint32_t next_ = 1;
  std::shared_ptr<Device> find(uint32_t h) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = devices_.find(h);
    return it == devices_.end() ? nullptr : it->second;
  }
  static std::vector<sdk::OdinSensorCapability> capabilities() {
    std::vector<sdk::OdinSensorCapability> caps;
    for (auto c : channels()) {
      sdk::OdinSensorCapability cap;
      cap.channel = c;
      sdk::OdinStreamCfg m;
      if (isCamera(c)) {
        m.width = 16;
        m.height = 16;
        m.fps = 10;
        m.format = static_cast<sdk::OdinDataFormat>(15);
      } else
        m.format = static_cast<sdk::OdinDataFormat>(c == Channel::kRawPoint    ? 0x10
                                                    : c == Channel::kSlamPoint ? 0x20
                                                    : c == Channel::kImu       ? 0x30
                                                    : c == Channel::kTf        ? 0x50
                                                                               : 0x40);
      cap.modes.push_back(m);
      caps.push_back(cap);
    }
    return caps;
  }
  static bool fault(const std::string& sn, const std::string& key, bool consume = false) {
    const auto path = "/odin2_mock/" + sn + "/" + key;
    bool enabled = false;
    ros::param::get(path, enabled);
    if (consume && enabled)
      ros::param::del(path);
    return enabled;
  }
  static void generate(std::shared_ptr<Device> d) {
    uint32_t seq = 0;
    while (!d->stopped) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::map<Channel, StreamSettings> list;
      PacketCallback callback;
      bool world;
      uint8_t mode;
      {
        std::lock_guard<std::mutex> lock(d->mutex);
        list = d->streams;
        callback = d->callback;
        world = d->world;
        mode = d->mode;
      }
      if (!callback || mode != 1 || fault(d->sn, "force_offline") || fault(d->sn, "stop_data"))
        continue;
      ++seq;
      uint64_t ns = uint64_t(seq) * 100000000 + 1000000000;
      for (const auto& pair : list) {
        auto c = pair.first;
        Packet packet;
        if (c == Channel::kRawPoint) {
          sdk::OdinPointCloudPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          sdk::OdinRawPoint<float> point;
          point.x = 1000;
          point.y = -2000;
          point.z = 500;
          point.intensity = 42;
          point.confidence = 200;
          p.points = {point};
          packet = p;
        } else if (c == Channel::kSlamPoint) {
          sdk::OdinSlamPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          sdk::OdinSlamPoint<float> point;
          point.x = world ? 2000 : 1000;
          point.y = 500;
          point.z = 250;
          point.r = 255;
          point.a = 255;
          p.points = {point};
          packet = p;
        } else if (c == Channel::kImu) {
          sdk::OdinImuPacket p;
          p.timestamp = ns;
          sdk::OdinImuSample s;
          s.gyro_x = 0.5;
          s.acc_z = 9.81;
          p.samples = {s};
          packet = p;
        } else if (c == Channel::kOdomLF || c == Channel::kOdomHF) {
          sdk::OdinOdomPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          sdk::OdinOdomData od;
          od.pos[0] = 1;
          od.orient[3] = 1;
          od.linear_velocity[0] = 0.2;
          od.type = c == Channel::kOdomHF ? 1 : 0;
          for (int i = 0; i < 18; ++i)
            od.cov[i] = i;
          p.data = {od};
          packet = p;
        } else if (c == Channel::kTf) {
          sdk::OdinTfPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          p.frame_id = "odom";
          p.child_frame_id = "imu";
          p.translation[0] = 1;
          packet = p;
        } else if (c == Channel::kTelemetry) {
          sdk::OdinTelemetryPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          p.data.uptime_s = seq / 10;
          p.data.cpu_pct = 25;
          p.data.mem_used_kb = 100;
          p.data.mem_total_kb = 1000;
          p.data.mem_use_rate = 10;
          p.data.load_1 = 0.25;
          packet = p;
        } else if (isCamera(c)) {
          if (c == Channel::kImage1 && fault(d->sn, "pause_camera1"))
            continue;
          sdk::OdinCameraPacket p;
          p.timestamp = ns;
          p.frame_count = seq;
          p.width = 16;
          p.height = 16;
          p.format = pair.second.mode.format;
          p.is_keyframe = true;
          if (p.format == sdk::OdinDataFormat::kImageNv12) {
            p.payload.resize(16 * 16 * 3 / 2, 128);
            std::fill(p.payload.begin(), p.payload.begin() + 256, 80);
          } else
            p.payload = d->fixtures[p.format];
          if (c == Channel::kImage1 && fault(d->sn, "corrupt_camera1"))
            p.payload = {1, 2, 3, 4};
          packet = std::move(p);
        }
        if (c == Channel::kTelemetry && fault(d->sn, "fail_telemetry"))
          continue;
        callback(c, packet);
        if (fault(d->sn, "duplicate_packets"))
          callback(c, std::move(packet));
      }
    }
  }

 public:
  ~MockSdk() override {
    while (!devices_.empty())
      disconnect(devices_.begin()->first);
  }
  std::vector<sdk::DiscoveredDevice> discover(uint32_t) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<sdk::DiscoveredDevice> out;
    for (int i = 1; i <= 2; ++i) {
      sdk::DiscoveredDevice d;
      d.model = "ODIN2";
      d.sn = "MOCK" + std::to_string(i);
      d.host_ip = "127.0.0.1";
      d.network.ip_address = "127.0.0." + std::to_string(i);
      ros::param::get("/odin2_mock/" + d.sn + "/ip_address", d.network.ip_address);
      queried_.emplace(d.network.ip_address, d.host_ip);
      out.push_back(d);
    }
    return out;
  }
  uint32_t connect(const sdk::DiscoveredDevice& descriptor) override {
    if (fault(descriptor.sn, "force_offline"))
      return 0;
    auto d = std::make_shared<Device>();
    d->sn = descriptor.sn;
    d->network = descriptor.network;
    d->filters.has_confidence = d->filters.has_range = d->filters.has_adaptive = true;
    std::string root = std::getenv("ODIN_TEST_FIXTURES") ? std::getenv("ODIN_TEST_FIXTURES") : "";
    for (auto pair : std::vector<std::pair<sdk::OdinDataFormat, std::string>>{
             {sdk::OdinDataFormat::kImageMjpeg, "mjpeg.jpg"},
             {sdk::OdinDataFormat::kImageH264, "h264.bin"},
             {sdk::OdinDataFormat::kImageH265, "h265.bin"}}) {
      std::ifstream file(root + "/" + pair.second, std::ios::binary);
      d->fixtures[pair.first] = std::vector<uint8_t>(std::istreambuf_iterator<char>(file), {});
    }
    std::lock_guard<std::mutex> lock(mutex_);
    d->queried = queried_.count({descriptor.network.ip_address, descriptor.host_ip});
    auto h = next_++;
    devices_[h] = d;
    d->worker = std::thread(generate, d);
    return h;
  }
  void disconnect(uint32_t h) override {
    auto d = find(h);
    if (!d)
      return;
    d->stopped = true;
    if (d->worker.joinable())
      d->worker.join();
    std::lock_guard<std::mutex> lock(mutex_);
    devices_.erase(h);
  }
  void callbacks(uint32_t h, PacketCallback callback) override {
    auto d = find(h);
    std::lock_guard<std::mutex> lock(d->mutex);
    d->callback = std::move(callback);
  }
  Reply start(uint32_t h, Channel c, const StreamSettings& s, uint32_t) override {
    auto d = find(h);
    std::lock_guard<std::mutex> lock(d->mutex);
    Reply r;
    bool require_query = false;
    ros::param::get("/odin2_mock/require_device_query", require_query);
    if (require_query && !d->queried) {
      r.code = sdk::OdinResult::kUnknownError;
    } else if (fault(d->sn, "fail_next_start", true) || fault(d->sn, "fail_all_starts")) {
      r.code = sdk::OdinResult::kCommunicationError;
      r.message = "injected stream failure";
    } else if (!(c == Channel::kTelemetry && fault(d->sn, "fail_telemetry")))
      d->streams[c] = s;
    return r;
  }
  Reply close(uint32_t h, Channel c) override {
    auto d = find(h);
    std::lock_guard<std::mutex> lock(d->mutex);
    d->streams.erase(c);
    return {};
  }
  void sync(uint32_t h, bool world, uint32_t) override {
    auto d = find(h);
    std::lock_guard<std::mutex> lock(d->mutex);
    d->world = world;
  }
  Reply execute(uint32_t h, const Command& c) override {
    auto d = find(h);
    Reply r;
    if (!d) {
      r.code = sdk::OdinResult::kNotInitialized;
      return r;
    }
    if (fault(d->sn, "force_offline")) {
      r.code = sdk::OdinResult::kCommunicationError;
      r.message = "injected offline";
      return r;
    }
    if (c.kind == CommandKind::File) {
      if (c.progress)
        c.progress(50);
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      if (!c.upload) {
        std::ofstream out(c.path);
        out << "mock calibration payload";
        if (!out)
          r.code = sdk::OdinResult::kInvalidArgument;
      }
      if (r.ok() && c.progress)
        c.progress(100);
      return r;
    }
    std::lock_guard<std::mutex> lock(d->mutex);
    switch (c.kind) {
      case CommandKind::Info:
        r.firmware = "mock-3.3.0";
        r.network = d->network;
        r.capabilities = capabilities();
        break;
      case CommandKind::Capabilities:
        r.capabilities = capabilities();
        break;
      case CommandKind::ChannelInfo:
        r.channel.subscribed = d->streams.count(c.channel);
        if (r.channel.subscribed) {
          r.channel.transport_mode = d->streams[c.channel].transport;
          r.channel.format = d->streams[c.channel].mode.format;
          r.channel.dst_port = 12345;
        }
        break;
      case CommandKind::GetDeviceMode:
        r.mode = d->mode;
        break;
      case CommandKind::SetDeviceMode:
        d->mode = c.mode;
        break;
      case CommandKind::GetSensorMode:
        r.mode = d->sensor;
        break;
      case CommandKind::SetSensorMode:
        d->sensor = c.mode;
        break;
      case CommandKind::GetSyncMode:
        r.mode = d->sync;
        break;
      case CommandKind::SetSyncMode:
        d->sync = c.mode;
        break;
      case CommandKind::GetFilters:
        // Firmware 0x33 is a placeholder: a successful ACK has an empty TLV.
        break;
      case CommandKind::SetFilters:
        // Firmware 0x32 acknowledges without changing any physical parameter.
        break;
      case CommandKind::GetAdaptive:
        r.adaptive = d->filters.adaptive;
        break;
      case CommandKind::SetAdaptive:
        d->filters.adaptive = c.adaptive;
        if (fault(d->sn, "fail_next_adaptive", true)) {
          r.code = sdk::OdinResult::kTimeout;
          r.message = "injected timeout after adaptive write";
        }
        break;
      case CommandKind::GetNetwork:
        r.network = d->network;
        break;
      case CommandKind::SetNetwork:
        d->network = c.network;
        ros::param::set("/odin2_mock/" + d->sn + "/ip_address", c.network.ip_address);
        break;
      case CommandKind::Ptp:
        r.ptp.sync_state = 2;
        r.ptp.offset_ns = 5;
        break;
      default:
        break;
    }
    return r;
  }
};
}  // namespace
std::shared_ptr<SdkFacade> makeMockSdk() {
  return std::make_shared<MockSdk>();
}
}  // namespace odin_driver
