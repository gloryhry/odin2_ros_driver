#include "odin2_ros_driver/driver.hpp"

#include <arpa/inet.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <odin2_ros_driver/DeviceCommand.h>
#include <odin2_ros_driver/DeviceConfig.h>
#include <odin2_ros_driver/DeviceStatus.h>
#include <odin2_ros_driver/EncodedFrame.h>
#include <odin2_ros_driver/GetAdaptive.h>
#include <odin2_ros_driver/GetChannelConfig.h>
#include <odin2_ros_driver/GetDeviceInfo.h>
#include <odin2_ros_driver/GetFilters.h>
#include <odin2_ros_driver/GetMode.h>
#include <odin2_ros_driver/GetNetwork.h>
#include <odin2_ros_driver/GetPtpStatus.h>
#include <odin2_ros_driver/GetTransferStatus.h>
#include <odin2_ros_driver/SendAlgoData.h>
#include <odin2_ros_driver/SetAdaptive.h>
#include <odin2_ros_driver/SetFilters.h>
#include <odin2_ros_driver/SetMode.h>
#include <odin2_ros_driver/SetNetwork.h>
#include <odin2_ros_driver/SetStream.h>
#include <odin2_ros_driver/StartFileTransfer.h>
#include <sched.h>
#include <sensor_msgs/CompressedImage.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <thread>
#include <tuple>

#include "odin2_ros_driver/conversion.hpp"

namespace odin_driver {
namespace msg = odin2_ros_driver;
namespace {
msg::Result result(const Reply& r) {
  msg::Result v;
  v.success = r.ok();
  v.sdk_code = static_cast<uint8_t>(r.code);
  v.message = r.message;
  return v;
}
Reply failure(const std::string& text, sdk::OdinResult code = sdk::OdinResult::kInvalidArgument) {
  Reply r;
  r.code = code;
  r.message = text;
  return r;
}
msg::NetworkConfig network(const sdk::NetworkAttribute& n) {
  msg::NetworkConfig v;
  v.ip_address = n.ip_address;
  v.gateway = n.gateway;
  v.netmask = n.netmask;
  v.mac = n.mac;
  v.dhcp = n.dhcp;
  v.ptp = n.ptp;
  return v;
}
sdk::NetworkAttribute network(const msg::NetworkConfig& n) {
  sdk::NetworkAttribute v;
  v.ip_address = n.ip_address;
  v.gateway = n.gateway;
  v.netmask = n.netmask;
  v.mac = n.mac;
  v.dhcp = n.dhcp;
  v.ptp = n.ptp;
  return v;
}
msg::FilterConfig filters(const sdk::OdinLidarFilterParams& f) {
  msg::FilterConfig v;
  v.has_confidence = f.has_confidence;
  v.confidence_threshold = f.confidence.confidence_threshold;
  v.has_adaptive = f.has_adaptive;
  v.cdf_percentile = f.adaptive.cdf_percentile;
  v.outdoor_threshold = f.adaptive.outdoor_threshold;
  v.has_range = f.has_range;
  v.min_depth = f.range.min_depth;
  return v;
}
sdk::OdinLidarFilterParams filters(const msg::FilterConfig& f) {
  sdk::OdinLidarFilterParams v;
  v.has_confidence = f.has_confidence;
  v.confidence.confidence_threshold = f.confidence_threshold;
  v.has_adaptive = f.has_adaptive;
  v.adaptive.cdf_percentile = f.cdf_percentile;
  v.adaptive.outdoor_threshold = f.outdoor_threshold;
  v.has_range = f.has_range;
  v.range.min_depth = f.min_depth;
  return v;
}
msg::PtpStatus ptp(const sdk::OdinPtpSyncStatus& p) {
  msg::PtpStatus v;
  v.sync_state = p.sync_state;
  v.servo_state = p.servo_state;
  v.offset_ns = p.offset_ns;
  v.path_delay_ns = p.path_delay_ns;
  v.freq_ppb = p.freq_ppb;
  return v;
}
msg::StreamConfig streamMessage(Channel c, const StreamSettings& s) {
  msg::StreamConfig v;
  v.channel = static_cast<uint8_t>(c);
  v.enabled = s.enabled;
  v.transport = static_cast<uint8_t>(s.transport);
  v.width = s.mode.width;
  v.height = s.mode.height;
  v.fps = s.mode.fps;
  v.format = static_cast<uint8_t>(s.mode.format);
  v.publish_rate = s.publish_rate;
  return v;
}
void checkIp(const std::string& ip) {
  in_addr address{};
  if (inet_pton(AF_INET, ip.c_str(), &address) != 1)
    throw std::invalid_argument("invalid IPv4 address: " + ip);
}
std::string routeAddress(const std::string& ip) {
  checkIp(ip);
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    throw std::runtime_error("route socket failed");
  sockaddr_in remote{};
  remote.sin_family = AF_INET;
  remote.sin_port = htons(60001);
  inet_pton(AF_INET, ip.c_str(), &remote.sin_addr);
  sockaddr_in local{};
  socklen_t length = sizeof(local);
  int status = ::connect(fd, reinterpret_cast<sockaddr*>(&remote), sizeof(remote));
  if (status == 0)
    status = getsockname(fd, reinterpret_cast<sockaddr*>(&local), &length);
  ::close(fd);
  if (status != 0)
    throw std::runtime_error("cannot determine local interface for " + ip);
  char buffer[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &local.sin_addr, buffer, sizeof(buffer));
  return buffer;
}
void affinity(const std::vector<int>& cpus) {
  if (cpus.empty())
    return;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  for (int cpu : cpus)
    CPU_SET(cpu, &mask);
  if (pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask) != 0)
    ROS_WARN("Could not apply requested CPU affinity");
}
std::string cameraRoot(const std::string& topic) {
  auto at = topic.find_last_of('/');
  return at == std::string::npos ? topic : topic.substr(0, at);
}
}  // namespace
struct DeviceDriver::Impl {
  struct Runtime {
    explicit Runtime(const decoding::Discovery& discovery)
        : decoder(discovery, [] { return std::chrono::steady_clock::now(); }) {}
    BoundedQueue queue;
    ros::Publisher primary, encoded, compressed, info, raw_odom;
    std::unique_ptr<camera_info_manager::CameraInfoManager> calibration;
    Decoder decoder;
    DecoderInfo decoder_info;
    msg::ChannelStatus status;
    bool busy = false, have_sequence = false, decoder_pending = false, decoder_active = false;
    uint32_t sequence = 0;
    uint64_t decode_generation = 0, decoder_seen = 0, decoder_epoch = UINT64_MAX;
    ros::WallTime last_receive, last_publish;
    ros::WallTime last_decoded_publish, last_wire_publish;
    std::deque<std::tuple<uint64_t, uint64_t, uint32_t>> published_packets;
  };
  ros::NodeHandle node;
  DeviceSettings cfg;
  std::shared_ptr<SdkFacade> sdk;
  sdk::OdinDeviceHandle handle = 0;
  sdk::DiscoveredDevice descriptor;
  std::vector<sdk::OdinSensorCapability> capabilities;
  std::map<Channel, std::unique_ptr<Runtime>> streams;
  std::mutex mutex;
  std::condition_variable data_cv, control_cv;
  std::deque<std::function<void()>> controls;
  std::thread controller;
  std::vector<std::thread> workers;
  std::atomic<bool> stopping{false};
  bool started = false, ready = false, file_busy = false, ptp_available = false;
  uint8_t operating_mode = 1;
  uint64_t generation = 0, reconnects = 0, next_job = 1;
  TimeMapper clock;
  std::string state = "disconnected", last_error;
  std::map<uint64_t, msg::TransferStatus> jobs;
  sdk::OdinPtpSyncStatus last_ptp;
  std::optional<sdk::OdinLidarFilterParams> desired_filters;
  std::optional<sdk::NetworkAttribute> pending_network;
  ros::Publisher status_pub, transfer_pub;
  ros::WallTime connected_at;
  ros::WallTimer status_timer;
  std::vector<ros::ServiceServer> services;
  tf2_ros::TransformBroadcaster broadcaster;
  diagnostic_updater::Updater diagnostics;
  boost::recursive_mutex dynamic_mutex;
  std::unique_ptr<dynamic_reconfigure::Server<msg::DeviceConfig>> dynamic;
  msg::DeviceConfig last_dynamic;
  bool dynamic_initializing = true;

  Impl(ros::NodeHandle n, DeviceSettings c, std::shared_ptr<SdkFacade> s,
       decoding::Discovery discovery)
      : node(std::move(n)),
        cfg(std::move(c)),
        sdk(std::move(s)),
        clock(cfg.time_mode, cfg.time_offset_ns) {
    desired_filters = cfg.initial_filters;
    for (auto ch : channels()) {
      auto r = std::make_unique<Runtime>(discovery);
      const auto& settings = cfg.streams.at(ch);
      r->queue.configure(settings.queue_size, settings.queue_bytes);
      r->status.channel = static_cast<uint8_t>(ch);
      if (ch == Channel::kRawPoint || ch == Channel::kSlamPoint)
        r->primary = node.advertise<sensor_msgs::PointCloud2>(settings.topic, 2);
      else if (isCamera(ch)) {
        const auto root = cameraRoot(settings.topic);
        r->primary = node.advertise<sensor_msgs::Image>(settings.topic, 2);
        r->encoded = node.advertise<msg::EncodedFrame>(root + "/encoded", 2);
        r->compressed =
            node.advertise<sensor_msgs::CompressedImage>(settings.topic + "/compressed", 2);
        r->info = node.advertise<sensor_msgs::CameraInfo>(root + "/camera_info", 2);
        ros::NodeHandle cam(node, root);
        r->calibration = std::make_unique<camera_info_manager::CameraInfoManager>(
            cam, cfg.name + "_" + channelName(ch), settings.camera_info_url);
      } else if (ch == Channel::kImu)
        r->primary = node.advertise<sensor_msgs::Imu>(settings.topic, 100);
      else if (ch == Channel::kOdomLF || ch == Channel::kOdomHF) {
        r->primary = node.advertise<nav_msgs::Odometry>(settings.topic, 20);
        r->raw_odom = node.advertise<msg::OdomPacket>(settings.topic + "/sdk", 20);
      } else if (ch == Channel::kTf)
        r->primary = node.advertise<tf2_msgs::TFMessage>(settings.topic, 20);
      else
        r->primary = node.advertise<msg::Telemetry>(settings.topic, 4);
      streams[ch] = std::move(r);
    }
    status_pub = node.advertise<msg::DeviceStatus>("status", 1, true);
    transfer_pub = node.advertise<msg::TransferStatus>("file_transfer/status", 10, true);
    diagnostics.setHardwareID(cfg.sn.empty() ? cfg.name : cfg.sn);
    diagnostics.add(cfg.name + "/driver",
                    [this](diagnostic_updater::DiagnosticStatusWrapper& d) { diagnose(d); });
    registerServices();
    dynamic = std::make_unique<dynamic_reconfigure::Server<msg::DeviceConfig>>(dynamic_mutex, node);
    last_dynamic = dynamicSnapshot();
    dynamic->updateConfig(last_dynamic);
    dynamic->setCallback([this](msg::DeviceConfig& v, uint32_t level) { reconfigure(v, level); });
    dynamic_initializing = false;
    status_timer =
        node.createWallTimer(ros::WallDuration(0.5), [this](const auto&) { publishStatus(); });
  }
  ~Impl() {
    stop();
  }
  DeviceSettings snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return cfg;
  }
  template <class S>
  void service(const std::string& name,
               std::function<bool(typename S::Request&, typename S::Response&)> fn) {
    boost::function<bool(typename S::Request&, typename S::Response&)> cb = std::move(fn);
    services.push_back(node.advertiseService(name, cb));
  }
  void start() {
    if (started)
      return;
    started = true;
    for (size_t i = 0; i < snapshot().worker_threads; ++i)
      workers.emplace_back([this] { workerLoop(); });
    controller = std::thread([this] { controlLoop(); });
  }
  void stop() {
    if (!started)
      return;
    status_timer.stop();
    stopping = true;
    control_cv.notify_all();
    data_cv.notify_all();
    if (controller.joinable())
      controller.join();
    for (auto& t : workers)
      if (t.joinable())
        t.join();
    workers.clear();
    started = false;
  }
  Reply runControl(std::function<Reply()> task) {
    auto promise = std::make_shared<std::promise<Reply>>();
    auto future = promise->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping || !ready)
        return failure("device is not ready", sdk::OdinResult::kNotInitialized);
      if (file_busy)
        return failure("file transfer is running", sdk::OdinResult::kStopped);
      if (controls.size() >= 64)
        return failure("control queue is full", sdk::OdinResult::kStopped);
      controls.emplace_back([this, task = std::move(task), promise] {
        try {
          bool available;
          {
            std::lock_guard<std::mutex> lock(mutex);
            available = ready;
          }
          if (stopping || !available)
            promise->set_value(failure("device is not ready", sdk::OdinResult::kNotInitialized));
          else
            promise->set_value(task());
        } catch (const std::exception& e) {
          promise->set_value(failure(e.what(), sdk::OdinResult::kUnknownError));
        }
      });
    }
    control_cv.notify_one();
    return future.get();
  }
  Reply command(Command c) {
    if (c.timeout_ms == 1000)
      c.timeout_ms = snapshot().command_timeout_ms;
    return runControl([this, c = std::move(c)] {
      if (!handle)
        return failure("device disconnected", sdk::OdinResult::kNotInitialized);
      return sdk->execute(handle, c);
    });
  }
  // Runs on the controller: read back, apply, and restore on any failure.
  Reply applyFilters(const sdk::OdinLidarFilterParams& requested) {
    validateFilters(requested);
    Command c;
    c.kind = CommandKind::GetFilters;
    c.timeout_ms = snapshot().command_timeout_ms;
    auto previous = sdk->execute(handle, c);
    if (!previous.ok())
      return previous;
    c.kind = CommandKind::SetFilters;
    c.filters = requested;
    auto changed = sdk->execute(handle, c);
    if (changed.ok()) {
      c.kind = CommandKind::GetFilters;
      changed = sdk->execute(handle, c);
    }
    if (changed.ok()) {
      desired_filters = changed.filters;
      {
        std::lock_guard<std::mutex> lock(mutex);
        cfg.initial_filters = changed.filters;
      }
      return changed;
    }
    c.kind = CommandKind::SetFilters;
    c.filters = previous.filters;
    auto restored = sdk->execute(handle, c);
    if (!restored.ok()) {
      disconnect();
      std::lock_guard<std::mutex> lock(mutex);
      state = "retrying";
      last_error = "filter rollback failed";
    }
    if (restored.ok())
      changed.filters = previous.filters;
    changed.message += restored.ok() ? "; previous filters restored"
                                     : "; filter rollback failed; device not ready";
    return changed;
  }
  void invalidateLocked() {
    ++generation;
    clock.reset();
    for (auto& pair : streams) {
      auto& r = *pair.second;
      r.queue.clear();
      ++r.decode_generation;
      r.have_sequence = false;
      r.last_publish = {};
      r.last_decoded_publish = {};
      r.last_wire_publish = {};
      r.published_packets.clear();
      r.last_receive = {};
      r.decoder_info.hardware_active = false;
    }
    data_cv.notify_all();
  }
  void disconnect() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready = false;
      ptp_available = false;
      invalidateLocked();
      for (auto& p : streams) {
        p.second->status.subscribed = false;
        p.second->status.receiving = false;
      }
    }
    if (handle) {
      sdk->disconnect(handle);
      handle = 0;
    }
  }
  Reply connectDevice() {
    const auto settings = snapshot();
    {
      std::lock_guard<std::mutex> lock(mutex);
      state = "discovering";
    }
    sdk::DiscoveredDevice found;
    if (!settings.device_ip.empty()) {
      checkIp(settings.device_ip);
      found.model = "ODIN2";
      found.sn = settings.sn;
      found.network.ip_address = settings.device_ip;
      found.host_ip =
          settings.host_ip.empty() ? routeAddress(settings.device_ip) : settings.host_ip;
      checkIp(found.host_ip);
    } else {
      auto list = sdk->discover(1000);
      std::vector<sdk::DiscoveredDevice> matches;
      for (auto& d : list)
        if ((settings.sn.empty() || d.sn == settings.sn) &&
            (settings.host_ip.empty() || d.host_ip == settings.host_ip))
          matches.push_back(d);
      if (matches.size() != 1)
        return failure(matches.empty() ? "configured device not discovered"
                                       : "multiple devices found; configure SN or IP",
                       sdk::OdinResult::kNotInitialized);
      found = matches.front();
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      state = "connecting";
      descriptor = found;
    }
    handle = sdk->connect(found);
    if (!handle)
      return failure("SDK connection failed", sdk::OdinResult::kCommunicationError);
    sdk->callbacks(handle, [this](Channel ch, Packet p) { enqueue(ch, std::move(p)); });
    return configureDevice(settings);
  }
  Reply configureDevice(const DeviceSettings& settings) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      state = "configuring";
    }
    Command c;
    c.kind = CommandKind::Capabilities;
    c.timeout_ms = settings.command_timeout_ms;
    auto r = sdk->execute(handle, c);
    if (!r.ok())
      return r;
    {
      std::lock_guard<std::mutex> lock(mutex);
      capabilities = r.capabilities;
    }
    for (const auto& pair : settings.streams)
      validateStream(pair.first, pair.second, r.capabilities);
    if (settings.slam_world && settings.streams.at(Channel::kSlamPoint).enabled) {
      auto dependency = settings.streams.at(Channel::kOdomLF);
      dependency.enabled = true;
      validateStream(Channel::kOdomLF, dependency, r.capabilities);
    }
    c.kind = CommandKind::SetDeviceMode;
    c.mode = 0;
    r = sdk->execute(handle, c);
    if (!r.ok())
      return r;
    sdk->sync(handle, settings.slam_world, settings.sync_lag);
    if (settings.sensor_mode) {
      c.kind = CommandKind::SetSensorMode;
      c.mode = *settings.sensor_mode;
      r = sdk->execute(handle, c);
      if (!r.ok())
        return r;
    }
    if (desired_filters) {
      c.kind = CommandKind::SetFilters;
      c.filters = *desired_filters;
      r = sdk->execute(handle, c);
      if (!r.ok())
        return r;
    }
    for (auto ch : channels()) {
      auto s = settings.streams.at(ch);
      bool dependency = ch == Channel::kOdomLF && settings.slam_world &&
                        settings.streams.at(Channel::kSlamPoint).enabled;
      if (!s.enabled && !dependency)
        continue;
      r = sdk->start(handle, ch, s, settings.command_timeout_ms);
      if (!r.ok())
        return r;
      bool subscribed = true;
      if (ch == Channel::kTelemetry) {
        c.kind = CommandKind::ChannelInfo;
        c.channel = ch;
        auto actual = sdk->execute(handle, c);
        subscribed = actual.ok() && actual.channel.subscribed;
      }
      std::lock_guard<std::mutex> lock(mutex);
      streams.at(ch)->status.subscribed = subscribed;
    }
    c.kind = CommandKind::SetDeviceMode;
    {
      std::lock_guard<std::mutex> lock(mutex);
      c.mode = operating_mode;
    }
    r = sdk->execute(handle, c);
    if (!r.ok())
      return r;
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready = true;
      state = "running";
      connected_at = ros::WallTime::now();
      last_error.clear();
      ++reconnects;
    }
    return r;
  }
  Reply applySettings(const DeviceSettings& candidate) {
    auto previous = snapshot();
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto& p : candidate.streams)
        validateStream(p.first, p.second, capabilities);
    }
    disconnect();
    {
      std::lock_guard<std::mutex> lock(mutex);
      cfg = candidate;
    }
    Reply r;
    try {
      r = connectDevice();
    } catch (const std::exception& e) {
      r = failure(e.what());
    }
    if (r.ok())
      return r;
    disconnect();
    {
      std::lock_guard<std::mutex> lock(mutex);
      cfg = previous;
    }
    Reply restored;
    try {
      restored = connectDevice();
    } catch (const std::exception& e) {
      restored = failure(e.what());
    }
    if (!restored.ok()) {
      disconnect();
      std::lock_guard<std::mutex> lock(mutex);
      state = "retrying";
      last_error = "configuration rollback failed: " + restored.message;
    }
    r.message += "; " + std::string(restored.ok() ? "previous configuration restored"
                                                  : "rollback failed; device not ready");
    return r;
  }
  void controlLoop() {
    affinity(snapshot().cpu_affinity);
    unsigned backoff = 1;
    auto next = std::chrono::steady_clock::now();
    auto health = next;
    while (!stopping) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(mutex);
        if (!controls.empty()) {
          task = std::move(controls.front());
          controls.pop_front();
        }
      }
      if (task) {
        task();
        continue;
      }
      auto now = std::chrono::steady_clock::now();
      bool connected;
      {
        std::lock_guard<std::mutex> lock(mutex);
        connected = ready;
      }
      if (!connected && now >= next) {
        Reply r;
        try {
          r = connectDevice();
        } catch (const std::exception& e) {
          r = failure(e.what(), sdk::OdinResult::kUnknownError);
        }
        if (!r.ok()) {
          disconnect();
          {
            std::lock_guard<std::mutex> lock(mutex);
            state = "retrying";
            last_error = r.message;
          }
          ROS_WARN_STREAM_THROTTLE(10, snapshot().name << ": " << r.message);
          next = std::chrono::steady_clock::now() + std::chrono::seconds(backoff);
          backoff = std::min(30u, backoff * 2);
        } else {
          backoff = 1;
          health = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        }
      } else if (connected && now >= health) {
        Command c;
        c.kind = CommandKind::GetDeviceMode;
        c.timeout_ms = snapshot().command_timeout_ms;
        auto r = sdk->execute(handle, c);
        if (!r.ok()) {
          disconnect();
          std::lock_guard<std::mutex> lock(mutex);
          last_error = r.message;
          state = "retrying";
          next = now + std::chrono::seconds(1);
        } else {
          std::lock_guard<std::mutex> lock(mutex);
          operating_mode = r.mode;
        }
        if (r.ok()) {
          bool stalled = false;
          {
            std::lock_guard<std::mutex> lock(mutex);
            const auto wall = ros::WallTime::now();
            if (operating_mode == 1 && (wall - connected_at).toSec() > 10) {
              for (auto ch : channels()) {
                bool required =
                    cfg.streams.at(ch).enabled || (ch == Channel::kOdomLF && cfg.slam_world &&
                                                   cfg.streams.at(Channel::kSlamPoint).enabled);
                const auto& received = streams.at(ch)->last_receive;
                if (required && ch != Channel::kTelemetry &&
                    (received.isZero() || (wall - received).toSec() > 10)) {
                  stalled = true;
                  last_error = channelName(ch) + " data timeout";
                  break;
                }
              }
            }
          }
          if (stalled) {
            disconnect();
            std::lock_guard<std::mutex> lock(mutex);
            state = "retrying";
            next = now + std::chrono::seconds(1);
            r = failure(last_error, sdk::OdinResult::kCommunicationError);
          }
        }
        if (r.ok()) {
          c.kind = CommandKind::Ptp;
          r = sdk->execute(handle, c);
          std::lock_guard<std::mutex> lock(mutex);
          ptp_available = r.ok();
          if (r.ok())
            last_ptp = r.ptp;
        }
        health = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      }
      std::unique_lock<std::mutex> lock(mutex);
      control_cv.wait_for(lock, std::chrono::milliseconds(100),
                          [this] { return stopping || !controls.empty(); });
    }
    disconnect();
    std::deque<std::function<void()>> remaining;
    {
      std::lock_guard<std::mutex> lock(mutex);
      remaining.swap(controls);
      state = "stopped";
    }
    for (auto& t : remaining)
      t();
  }
  void enqueue(Channel ch, Packet p) {
    try {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping || !ready)
        return;
      auto& r = *streams.at(ch);
      const auto& s = cfg.streams.at(ch);
      r.last_receive = ros::WallTime::now();
      if (ch == Channel::kTelemetry)
        r.status.subscribed = true;
      r.status.receiving = true;
      ++r.status.received;
      if (auto sequence = packetSequence(p)) {
        uint32_t delta = *sequence - r.sequence;
        if (r.have_sequence && (delta == 0 || delta >= 0x80000000u)) {
          ++r.status.out_of_order;
          return;
        }
        if (r.have_sequence && delta > 1 && delta < 0x80000000u) {
          r.status.sequence_gaps += delta - 1;
          ++r.decode_generation;
          if (isCamera(ch)) {
            r.status.queue_drops += r.queue.size();
            r.queue.clear();
          }
        }
        if (!r.have_sequence || delta < 0x80000000u) {
          r.sequence = *sequence;
          r.have_sequence = true;
        }
      }
      if (!s.enabled)
        return;
      const auto received = ros::Time::now();
      bool reset = false;
      auto stamp = clock.map(ch, packetTimestamp(p), received, reset);
      if (reset) {
        ++generation;
        for (auto& v : streams) {
          v.second->queue.clear();
          ++v.second->decode_generation;
          v.second->have_sequence = false;
        }
      }
      if (!stamp) {
        if (received.isZero())
          ++r.status.invalid_packets;
        else
          ++r.status.out_of_order;
        return;
      }
      Envelope value;
      value.channel = ch;
      value.bytes = packetBytes(p);
      value.packet = std::move(p);
      value.received = received;
      value.stamp = *stamp;
      value.wall_received = r.last_receive;
      value.generation = generation;
      if (isCamera(ch) && r.queue.wouldOverflow(value.bytes)) {
        r.status.queue_drops += r.queue.size();
        r.queue.clear();
        ++r.decode_generation;
      }
      value.decode_generation = r.decode_generation;
      auto dropped = r.queue.push(std::move(value));
      r.status.queue_drops += dropped;
      if (dropped)
        ++r.decode_generation;
      data_cv.notify_one();
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM_THROTTLE(5, "SDK callback: " << e.what());
    }
  }
  bool decoderWork(Channel channel, const Runtime& r) const {
    if (!isCamera(channel))
      return false;
    const auto& stream = cfg.streams.at(channel);
    return r.decoder_pending ||
           (r.decoder_active &&
            (!ready || !stream.enabled || !stream.publish_decoded ||
             r.decoder_epoch != generation || r.decoder_seen != r.decode_generation));
  }
  void workerLoop() {
    affinity(snapshot().cpu_affinity);
    size_t cursor = 0;
    while (!stopping) {
      std::optional<Envelope> value;
      std::optional<Channel> poll_channel;
      {
        std::unique_lock<std::mutex> lock(mutex);
        auto have_work = [this] {
          if (stopping)
            return true;
          for (const auto& p : streams)
            if (!p.second->busy && (!p.second->queue.empty() || (decoderWork(p.first, *p.second) &&
                                                                 !p.second->decoder_pending)))
              return true;
          return false;
        };
        bool asynchronous = false;
        for (const auto& entry : streams)
          asynchronous |= entry.second->decoder_pending;
        if (asynchronous)
          data_cv.wait_for(lock, std::chrono::milliseconds(5), have_work);
        else
          data_cv.wait(lock, have_work);
        if (stopping)
          break;
        for (size_t i = 0; i < channels().size(); ++i) {
          auto c = channels()[(cursor + i) % channels().size()];
          auto& r = *streams.at(c);
          if (!r.busy && !r.queue.empty()) {
            r.busy = true;
            value = r.queue.pop();
            cursor = (cursor + i + 1) % channels().size();
            break;
          }
          if (!r.busy && decoderWork(c, r)) {
            r.busy = true;
            poll_channel = c;
            cursor = (cursor + i + 1) % channels().size();
            break;
          }
        }
      }
      if (value || poll_channel) {
        const auto channel = value ? value->channel : *poll_channel;
        try {
          if (value)
            process(*value);
          else
            pollCamera(channel);
        } catch (const std::exception& e) {
          std::lock_guard<std::mutex> lock(mutex);
          auto& r = *streams.at(channel);
          if (isCamera(channel)) {
            ++r.status.decode_errors;
            ++r.decode_generation;
          } else
            ++r.status.invalid_packets;
          last_error = e.what();
        }
        std::lock_guard<std::mutex> lock(mutex);
        streams.at(channel)->busy = false;
        data_cv.notify_all();
      }
    }
  }
  void process(const Envelope& e);
  void pollCamera(Channel);
  void publishDecoded(Channel, std::vector<DecodedFrame>);
  void decoderStatus(Channel);
  void cameraPublished(Runtime&, const DecodeContext&);
  void publishStatus();
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper&);
  void registerServices();
  msg::DeviceConfig dynamicSnapshot();
  void reconfigure(msg::DeviceConfig&, uint32_t);
};
void DeviceDriver::Impl::cameraPublished(Runtime& r, const DecodeContext& context) {
  const auto key =
      std::make_tuple(context.generation, context.header.stamp.toNSec(), context.header.seq);
  if (std::find(r.published_packets.begin(), r.published_packets.end(), key) ==
      r.published_packets.end()) {
    ++r.status.published;
    r.published_packets.push_back(key);
    if (r.published_packets.size() > 128)
      r.published_packets.pop_front();
  }
  r.last_publish = ros::WallTime::now();
  r.status.last_publish_latency_ms =
      std::max(0.0, (r.last_publish - context.received).toSec() * 1000);
}
void DeviceDriver::Impl::decoderStatus(Channel channel) {
  auto& r = *streams.at(channel);
  auto info = r.decoder.info();
  const auto pending = r.decoder.pending();
  std::lock_guard<std::mutex> lock(mutex);
  if (r.decoder_info.backend != info.backend || r.decoder_info.device != info.device ||
      r.decoder_info.hardware_active != info.hardware_active ||
      r.decoder_info.fallback_reason != info.fallback_reason) {
    ROS_INFO_STREAM(cfg.name << " " << channelName(channel) << ": decoder=" << info.backend
                             << " device=" << info.device
                             << " hardware_active=" << info.hardware_active);
    if (!info.fallback_reason.empty())
      ROS_WARN_STREAM(cfg.name << " " << channelName(channel) << ": " << info.fallback_reason);
  }
  r.decoder_info = std::move(info);
  r.decoder_pending = pending;
  r.decoder_active = r.decoder.active();
}
void DeviceDriver::Impl::publishDecoded(Channel channel, std::vector<DecodedFrame> frames) {
  auto& r = *streams.at(channel);
  for (auto& frame : frames) {
    auto info = r.calibration->getCameraInfo();
    info.header = frame.image.header;
    if (info.K[0] == 0) {
      info.width = frame.image.width;
      info.height = frame.image.height;
    }
    std::lock_guard<std::mutex> lock(mutex);
    const auto& stream = cfg.streams.at(channel);
    if (stopping || !ready || !stream.enabled || !stream.publish_decoded ||
        frame.context.generation != generation ||
        frame.context.decode_generation != r.decode_generation)
      continue;
    ++r.status.decoded_frames;
    if (stream.publish_rate > 0 && !r.last_decoded_publish.isZero() &&
        (ros::WallTime::now() - r.last_decoded_publish).toSec() < 1 / stream.publish_rate)
      continue;
    if (info.width == frame.image.width && info.height == frame.image.height)
      r.info.publish(info);
    else
      ROS_WARN_STREAM_THROTTLE(10, cfg.name
                                       << " " << channelName(channel)
                                       << ": calibration dimensions differ; CameraInfo suppressed");
    r.primary.publish(frame.image);
    r.last_decoded_publish = ros::WallTime::now();
    cameraPublished(r, frame.context);
  }
}
void DeviceDriver::Impl::pollCamera(Channel channel) {
  auto& r = *streams.at(channel);
  bool valid;
  bool preserve;
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto& stream = cfg.streams.at(channel);
    preserve = r.decoder_epoch == generation;
    valid = ready && stream.enabled && stream.publish_decoded && preserve &&
            r.decoder_seen == r.decode_generation;
  }
  try {
    if (valid)
      publishDecoded(channel, r.decoder.poll());
    else
      r.decoder.reset(preserve);
  } catch (const std::exception& error) {
    r.decoder.reset(true);
    std::lock_guard<std::mutex> lock(mutex);
    ++r.status.decode_errors;
    last_error = error.what();
  }
  decoderStatus(channel);
}
void DeviceDriver::Impl::process(const Envelope& e) {
  DeviceSettings settings;
  uint64_t decode_generation;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (!ready || e.generation != generation || !cfg.streams.at(e.channel).enabled)
      return;
    settings = cfg;
    auto& r = *streams.at(e.channel);
    decode_generation = e.decode_generation;
    if (isCamera(e.channel) && decode_generation != r.decode_generation)
      return;
    if (settings.streams.at(e.channel).publish_rate > 0 && !r.last_publish.isZero() &&
        (ros::WallTime::now() - r.last_publish).toSec() <
            1 / settings.streams.at(e.channel).publish_rate) {
      // Video must still consume every frame so references remain usable.
      if (!isCamera(e.channel))
        return;
    }
  }
  auto& r = *streams.at(e.channel);
  const auto& s = settings.streams.at(e.channel);
  std_msgs::Header header;
  header.stamp = e.stamp;
  header.frame_id = prefixedFrame(settings, s.frame);
  if (auto seq = packetSequence(e.packet))
    header.seq = *seq;
  std::vector<std::function<void()>> publications;
  auto publish = [&](ros::Publisher pub, auto value) {
    publications.emplace_back([pub, value = std::move(value)] { pub.publish(value); });
  };
  switch (e.channel) {
    case Channel::kRawPoint:
      publish(r.primary, rawCloud(std::get<sdk::OdinPointCloudPacket>(e.packet), header));
      break;
    case Channel::kSlamPoint:
      publish(r.primary, slamCloud(std::get<sdk::OdinSlamPacket>(e.packet), header));
      break;
    case Channel::kImu: {
      const auto& packet = std::get<sdk::OdinImuPacket>(e.packet);
      // SDK v2 emits one timestamped sample per callback, never synthesize intervals.
      if (packet.samples.size() != 1)
        throw std::runtime_error("SDK IMU callback must contain one timestamped sample");
      publish(r.primary, imuMessage(packet.samples.front(), header));
      break;
    }
    case Channel::kOdomLF:
    case Channel::kOdomHF: {
      const auto& packet = std::get<sdk::OdinOdomPacket>(e.packet);
      const auto type = e.channel == Channel::kOdomHF ? 1 : 0;
      publish(r.raw_odom, odomPacket(packet, header, type));
      for (const auto& data : packet.data)
        publish(r.primary, odomMessage(data, header, settings));
      break;
    }
    case Channel::kTf: {
      auto transform = tfMessage(std::get<sdk::OdinTfPacket>(e.packet), header, settings);
      tf2_msgs::TFMessage message;
      message.transforms.push_back(transform);
      publish(r.primary, message);
      if (settings.broadcast_tf)
        publications.emplace_back([this, transform] { broadcaster.sendTransform(transform); });
      break;
    }
    case Channel::kTelemetry:
      publish(r.primary, telemetryMessage(std::get<sdk::OdinTelemetryPacket>(e.packet), header));
      break;
    case Channel::kImage0:
    case Channel::kImage1: {
      const auto& packet = std::get<sdk::OdinCameraPacket>(e.packet);
      if (packet.format != s.mode.format)
        throw std::runtime_error("camera callback format differs from configured codec");
      if (s.publish_encoded) {
        msg::EncodedFrame out;
        out.header = header;
        out.device_timestamp_ns = packet.timestamp;
        out.frame_count = packet.frame_count;
        out.format = static_cast<uint8_t>(packet.format);
        out.width = packet.width;
        out.height = packet.height;
        out.is_keyframe = cameraKeyframe(packet);
        out.data = packet.payload;
        publish(r.encoded, std::move(out));
      }
      if (packet.format == sdk::OdinDataFormat::kImageMjpeg) {
        sensor_msgs::CompressedImage out;
        out.header = header;
        out.format = "bgr8; jpeg compressed bgr8";
        out.data = packet.payload;
        publish(r.compressed, std::move(out));
      }
      if (s.publish_decoded) {
        if (r.decoder_seen != decode_generation || r.decoder_epoch != e.generation) {
          r.decoder.reset(r.decoder_epoch == e.generation);
          r.decoder_seen = decode_generation;
          r.decoder_epoch = e.generation;
        }
        try {
          DecodeContext context;
          context.header = header;
          context.generation = e.generation;
          context.decode_generation = e.decode_generation;
          context.received = e.wall_received;
          publishDecoded(e.channel, r.decoder.submit(packet, context));
        } catch (const std::exception& error) {
          r.decoder.reset(true);
          std::lock_guard<std::mutex> lock(mutex);
          ++r.status.decode_errors;
          last_error = error.what();
          // Preserve encoded output even when decoding fails.
        }
        decoderStatus(e.channel);
      }
      break;
    }
    default:
      throw std::runtime_error("unexpected channel");
  }
  // Holding this short lock makes invalidation atomic with the final publication.
  std::lock_guard<std::mutex> lock(mutex);
  if (stopping || !ready || e.generation != generation || !cfg.streams.at(e.channel).enabled)
    return;
  if (isCamera(e.channel) && e.decode_generation != r.decode_generation)
    return;
  const auto rate_time = isCamera(e.channel) ? r.last_wire_publish : r.last_publish;
  if (s.publish_rate > 0 && !rate_time.isZero() &&
      (ros::WallTime::now() - rate_time).toSec() < 1 / s.publish_rate)
    return;
  if (publications.empty())
    return;
  for (auto& send : publications)
    send();
  if (isCamera(e.channel)) {
    cameraPublished(r, DecodeContext{header, e.generation, e.decode_generation, e.wall_received});
    r.last_wire_publish = ros::WallTime::now();
  } else
    ++r.status.published;
  r.last_publish = ros::WallTime::now();
  r.status.last_publish_latency_ms =
      std::max(0.0, (r.last_publish - e.wall_received).toSec() * 1000);
}
void DeviceDriver::Impl::publishStatus() {
  msg::DeviceStatus out;
  out.header.stamp = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(mutex);
    out.name = cfg.name;
    out.state = state;
    out.serial_number = descriptor.sn;
    out.device_ip = descriptor.network.ip_address;
    out.last_error = last_error;
    out.reconnects = reconnects;
    out.clock_resets = clock.resets();
    out.covariance_configured = cfg.covariance_configured;
    out.ptp_available = ptp_available;
    out.ptp = ptp(last_ptp);
    for (auto ch : channels()) {
      auto& r = *streams.at(ch);
      auto status = r.status;
      status.enabled = cfg.streams.at(ch).enabled;
      status.queue_depth = r.queue.size();
      status.queue_bytes = r.queue.bytes();
      status.last_receive_age =
          r.last_receive.isZero() ? -1 : (ros::WallTime::now() - r.last_receive).toSec();
      status.receiving = !r.last_receive.isZero() && status.last_receive_age < 5;
      out.channels.push_back(status);
    }
  }
  status_pub.publish(out);
  diagnostics.update();
}
void DeviceDriver::Impl::diagnose(diagnostic_updater::DiagnosticStatusWrapper& d) {
  std::lock_guard<std::mutex> lock(mutex);
  if (!ready)
    d.summary(diagnostic_msgs::DiagnosticStatus::ERROR, state + ": " + last_error);
  else
    d.summary(diagnostic_msgs::DiagnosticStatus::OK, "running");
  d.add("state", state);
  d.add("sdk_version", sdk::GetSdkVersion());
  d.add("reconnects", reconnects);
  d.add("clock_resets", clock.resets());
  d.add("ptp_available", ptp_available);
  if (ptp_available) {
    d.add("ptp_state", last_ptp.sync_state);
    d.add("ptp_offset_ns", last_ptp.offset_ns);
  }
  if (ready && !cfg.covariance_configured &&
      (cfg.streams.at(Channel::kOdomLF).enabled || cfg.streams.at(Channel::kOdomHF).enabled))
    d.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                   "using conservative odometry covariance placeholders");
  for (auto ch : channels()) {
    const auto& r = *streams.at(ch);
    const auto name = channelName(ch);
    d.add(name + "/received", r.status.received);
    d.add(name + "/published", r.status.published);
    d.add(name + "/sequence_gaps", r.status.sequence_gaps);
    d.add(name + "/queue_drops", r.status.queue_drops);
    d.add(name + "/decode_errors", r.status.decode_errors);
    d.add(name + "/decoded_frames", r.status.decoded_frames);
    if (isCamera(ch)) {
      d.add(name + "/decoder_backend", r.decoder_info.backend);
      d.add(name + "/decoder_device", r.decoder_info.device);
      d.add(name + "/hardware_active", r.decoder_info.hardware_active);
      d.add(name + "/cpu_fallback_reason", r.decoder_info.fallback_reason);
      std::string failures;
      for (const auto& failure : r.decoder_info.failures) {
        if (!failures.empty())
          failures += "; ";
        failures += failure;
      }
      d.add(name + "/decoder_failures", failures);
    }
    d.add(name + "/queue_depth", r.queue.size());
    d.add(name + "/queue_bytes", r.queue.bytes());
    d.add(name + "/invalid_packets", r.status.invalid_packets);
    d.add(name + "/out_of_order", r.status.out_of_order);
    if (ready && operating_mode == 1 && cfg.streams.at(ch).enabled &&
        (r.last_receive.isZero() || (ros::WallTime::now() - r.last_receive).toSec() > 5))
      d.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                     name + " has no recent data (subscription not confirmed by reception)");
  }
}
msg::DeviceConfig DeviceDriver::Impl::dynamicSnapshot() {
  auto c = snapshot();
  msg::DeviceConfig v;
#define READ_DYNAMIC(name, channel)                          \
  v.name##_enabled = c.streams.at(Channel::channel).enabled; \
  v.name##_publish_rate = c.streams.at(Channel::channel).publish_rate
  READ_DYNAMIC(raw_point, kRawPoint);
  READ_DYNAMIC(slam_point, kSlamPoint);
  READ_DYNAMIC(camera0, kImage0);
  READ_DYNAMIC(camera1, kImage1);
  READ_DYNAMIC(imu, kImu);
  READ_DYNAMIC(odom_lf, kOdomLF);
  READ_DYNAMIC(odom_hf, kOdomHF);
  READ_DYNAMIC(tf, kTf);
  READ_DYNAMIC(telemetry, kTelemetry);
#undef READ_DYNAMIC
  v.apply_filters = false;
  v.confidence_threshold = 0;
  v.min_depth = 150;
  v.cdf_percentile = 7000;
  v.outdoor_threshold = 3000;
  if (c.initial_filters) {
    const auto& f = *c.initial_filters;
    v.confidence_threshold = f.confidence.confidence_threshold;
    v.min_depth = f.range.min_depth;
    v.cdf_percentile = f.adaptive.cdf_percentile;
    v.outdoor_threshold = f.adaptive.outdoor_threshold;
  }
  return v;
}
void DeviceDriver::Impl::reconfigure(msg::DeviceConfig& v, uint32_t) {
  if (dynamic_initializing) {
    v = last_dynamic;
    return;
  }
  auto candidate = snapshot();
#define WRITE_DYNAMIC(name, channel)                  \
  {                                                   \
    auto& s = candidate.streams.at(Channel::channel); \
    s.enabled = v.name##_enabled;                     \
    s.publish_rate = v.name##_publish_rate;           \
  }
  WRITE_DYNAMIC(raw_point, kRawPoint);
  WRITE_DYNAMIC(slam_point, kSlamPoint);
  WRITE_DYNAMIC(camera0, kImage0);
  WRITE_DYNAMIC(camera1, kImage1);
  WRITE_DYNAMIC(imu, kImu);
  WRITE_DYNAMIC(odom_lf, kOdomLF);
  WRITE_DYNAMIC(odom_hf, kOdomHF);
  WRITE_DYNAMIC(tf, kTf);
  WRITE_DYNAMIC(telemetry, kTelemetry);
#undef WRITE_DYNAMIC
  Reply r;
  r = runControl([this, candidate, v] {
    auto previous = snapshot();
    auto current = previous;
    bool needs_update = false;
    for (auto ch : channels()) {
      needs_update |= current.streams.at(ch).enabled != candidate.streams.at(ch).enabled ||
                      current.streams.at(ch).publish_rate != candidate.streams.at(ch).publish_rate;
      current.streams.at(ch).enabled = candidate.streams.at(ch).enabled;
      current.streams.at(ch).publish_rate = candidate.streams.at(ch).publish_rate;
    }
    Reply reply;
    if (needs_update)
      reply = applySettings(current);
    if (reply.ok() && v.apply_filters) {
      sdk::OdinLidarFilterParams f;
      f.has_confidence = f.has_range = f.has_adaptive = true;
      f.confidence.confidence_threshold = v.confidence_threshold;
      f.range.min_depth = v.min_depth;
      f.adaptive.cdf_percentile = v.cdf_percentile;
      f.adaptive.outdoor_threshold = v.outdoor_threshold;
      reply = applyFilters(f);
      if (!reply.ok() && needs_update) {
        auto restored = applySettings(previous);
        if (!restored.ok())
          reply.message += "; stream rollback failed";
      }
    }
    return reply;
  });
  if (!r.ok()) {
    ROS_WARN_STREAM(snapshot().name << " dynamic configuration failed: " << r.message);
    v = dynamicSnapshot();
  } else {
    v.apply_filters = false;
    last_dynamic = v;
  }
}
void DeviceDriver::Impl::registerServices() {
  service<msg::GetDeviceInfo>("get_device_info", [this](auto&, auto& out) {
    Command c;
    c.kind = CommandKind::Info;
    auto r = command(c);
    out.result = result(r);
    out.sdk_version = sdk::GetSdkVersion();
    out.firmware_version = r.firmware;
    out.network = network(r.network);
    {
      std::lock_guard<std::mutex> lock(mutex);
      out.serial_number = descriptor.sn;
      out.model = descriptor.model;
    }
    for (const auto& cap : r.capabilities) {
      msg::SensorCapability v;
      v.channel = static_cast<uint8_t>(cap.channel);
      for (const auto& m : cap.modes) {
        msg::StreamMode mode;
        mode.width = m.width;
        mode.height = m.height;
        mode.fps = m.fps;
        mode.format = static_cast<uint8_t>(m.format);
        v.modes.push_back(mode);
      }
      out.capabilities.push_back(v);
    }
    return true;
  });
  service<msg::GetChannelConfig>("get_channel_config", [this](auto& in, auto& out) {
    if (!isKnownChannel(in.channel)) {
      out.result = result(failure("unknown channel"));
      return true;
    }
    auto ch = static_cast<Channel>(in.channel);
    Command c;
    c.kind = CommandKind::ChannelInfo;
    c.channel = ch;
    auto r = command(c);
    out.result = result(r);
    out.effective = streamMessage(ch, snapshot().streams.at(ch));
    out.subscribed = r.channel.subscribed;
    out.destination_port = r.channel.dst_port;
    out.resolution_id = r.channel.resolution_id;
    out.fps_id = r.channel.fps_id;
    if (r.ok()) {
      out.effective.transport = static_cast<uint8_t>(r.channel.transport_mode);
      out.effective.format = static_cast<uint8_t>(r.channel.format);
    }
    return true;
  });
  service<msg::SetStream>("set_stream", [this](auto& in, auto& out) {
    if (!isKnownChannel(in.config.channel)) {
      out.result = result(failure("unknown channel"));
      return true;
    }
    auto ch = static_cast<Channel>(in.config.channel);
    auto input = in.config;
    auto r = runControl([this, ch, input] {
      auto candidate = snapshot();
      auto& s = candidate.streams.at(ch);
      s.enabled = input.enabled;
      s.transport = static_cast<sdk::OdinTransportMode>(input.transport);
      s.mode.width = input.width;
      s.mode.height = input.height;
      s.mode.fps = input.fps;
      s.mode.format = static_cast<sdk::OdinDataFormat>(input.format);
      s.publish_rate = input.publish_rate;
      return applySettings(candidate);
    });
    out.result = result(r);
    out.effective = streamMessage(ch, snapshot().streams.at(ch));
    {
      boost::recursive_mutex::scoped_lock lock(dynamic_mutex);
      auto v = dynamicSnapshot();
      last_dynamic = v;
      dynamic->updateConfig(v);
    }
    return true;
  });
  auto modes = [this](const std::string& name, CommandKind get, CommandKind set, uint8_t maximum) {
    service<msg::GetMode>("get_" + name, [this, get](auto&, auto& out) {
      Command c;
      c.kind = get;
      auto r = command(c);
      out.result = result(r);
      out.mode = r.mode;
      return true;
    });
    service<msg::SetMode>("set_" + name, [this, get, set, maximum](auto& in, auto& out) {
      if (in.mode > maximum) {
        out.result = result(failure("mode out of range"));
        return true;
      }
      auto r = runControl([this, get, set, mode = in.mode] {
        Command c;
        c.kind = get;
        c.timeout_ms = snapshot().command_timeout_ms;
        auto old = sdk->execute(handle, c);
        if (!old.ok())
          return old;
        c.kind = set;
        c.mode = mode;
        auto reply = sdk->execute(handle, c);
        if (reply.ok()) {
          c.kind = get;
          reply = sdk->execute(handle, c);
        }
        if (!reply.ok()) {
          c.kind = set;
          c.mode = old.mode;
          auto restored = sdk->execute(handle, c);
          reply.mode = old.mode;
          if (!restored.ok()) {
            disconnect();
            std::lock_guard<std::mutex> lock(mutex);
            state = "retrying";
            last_error = "mode rollback failed";
          }
          reply.message += restored.ok() ? "; previous mode restored" : "; rollback failed";
          return reply;
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (set == CommandKind::SetDeviceMode)
            operating_mode = reply.mode;
          if (set == CommandKind::SetSensorMode)
            cfg.sensor_mode = reply.mode;
        }
        if (set == CommandKind::SetSyncMode)
          reply.message = "persisted; reboot required to take effect";
        return reply;
      });
      out.effective_mode = r.mode;
      out.result = result(r);
      return true;
    });
  };
  modes("device_mode", CommandKind::GetDeviceMode, CommandKind::SetDeviceMode, 2);
  modes("sensor_mode", CommandKind::GetSensorMode, CommandKind::SetSensorMode, 3);
  modes("sync_mode", CommandKind::GetSyncMode, CommandKind::SetSyncMode, 1);
  service<msg::GetFilters>("get_filters", [this](auto&, auto& out) {
    Command c;
    c.kind = CommandKind::GetFilters;
    auto r = command(c);
    out.result = result(r);
    out.filters = filters(r.filters);
    return true;
  });
  service<msg::SetFilters>("set_filters", [this](auto& in, auto& out) {
    Reply r;
    try {
      auto f = filters(in.filters);
      validateFilters(f);
      r = runControl([this, f] { return applyFilters(f); });
    } catch (const std::exception& e) {
      r = failure(e.what());
    }
    out.result = result(r);
    out.effective = filters(r.filters);
    return true;
  });
  service<msg::GetAdaptive>("get_adaptive", [this](auto&, auto& out) {
    Command c;
    c.kind = CommandKind::GetAdaptive;
    auto r = command(c);
    out.result = result(r);
    out.cdf_percentile = r.adaptive.cdf_percentile;
    out.outdoor_threshold = r.adaptive.outdoor_threshold;
    return true;
  });
  service<msg::SetAdaptive>("set_adaptive", [this](auto& in, auto& out) {
    if (in.cdf_percentile < 1 || in.cdf_percentile > 10000) {
      out.result = result(failure("cdf_percentile must be 1..10000"));
      return true;
    }
    auto r = runControl([this, percentile = in.cdf_percentile, threshold = in.outdoor_threshold] {
      sdk::OdinLidarFilterParams f;
      f.has_adaptive = true;
      f.adaptive = {percentile, threshold};
      return applyFilters(f);
    });
    out.effective_cdf_percentile = r.filters.adaptive.cdf_percentile;
    out.effective_outdoor_threshold = r.filters.adaptive.outdoor_threshold;
    out.result = result(r);
    return true;
  });
  service<msg::GetPtpStatus>("get_ptp_status", [this](auto&, auto& out) {
    Command c;
    c.kind = CommandKind::Ptp;
    auto r = command(c);
    out.result = result(r);
    out.status = ptp(r.ptp);
    return true;
  });
  service<msg::GetNetwork>("get_network", [this](auto&, auto& out) {
    Command c;
    c.kind = CommandKind::GetNetwork;
    auto r = command(c);
    out.result = result(r);
    out.network = network(r.network);
    return true;
  });
  service<msg::SetNetwork>("set_network", [this](auto& in, auto& out) {
    Reply r;
    Command c;
    c.kind = CommandKind::SetNetwork;
    c.network = network(in.network);
    try {
      if (!c.network.dhcp) {
        checkIp(c.network.ip_address);
        checkIp(c.network.netmask);
        if (!c.network.gateway.empty())
          checkIp(c.network.gateway);
      }
      r = runControl([this, c]() mutable {
        auto settings = snapshot();
        if (c.network.dhcp && settings.sn.empty() && descriptor.sn.empty())
          return failure("DHCP requires a configured or discovered serial number");
        c.timeout_ms = settings.command_timeout_ms;
        auto reply = sdk->execute(handle, c);
        if (reply.ok()) {
          pending_network = c.network;
          c.kind = CommandKind::GetNetwork;
          reply = sdk->execute(handle, c);
          reply.message += "; network persisted; reboot required";
        }
        return reply;
      });
    } catch (const std::exception& e) {
      r = failure(e.what());
    }
    out.effective = network(r.network);
    out.reboot_required = r.ok();
    out.result = result(r);
    return true;
  });
  auto simple = [this](const std::string& name, CommandKind kind, bool reboot_required) {
    service<msg::DeviceCommand>(name, [this, kind, reboot_required](auto&, auto& out) {
      auto r = runControl([this, kind] {
        Command c;
        c.kind = kind;
        c.timeout_ms = snapshot().command_timeout_ms;
        auto reply = sdk->execute(handle, c);
        if (reply.ok() && (kind == CommandKind::Reboot || kind == CommandKind::ResetPose)) {
          disconnect();
          std::lock_guard<std::mutex> lock(mutex);
          if (kind == CommandKind::Reboot && pending_network) {
            cfg.device_ip = pending_network->dhcp ? "" : pending_network->ip_address;
            if (cfg.sn.empty())
              cfg.sn = descriptor.sn;
            pending_network.reset();
          }
          state = "retrying";
        }
        return reply;
      });
      out.result = result(r);
      out.reboot_required = r.ok() && reboot_required;
      return true;
    });
  };
  simple("reset_initial_pose", CommandKind::ResetPose, false);
  simple("restore_defaults", CommandKind::Restore, true);
  simple("reboot", CommandKind::Reboot, false);
  service<msg::SendAlgoData>("send_algo_data", [this](auto& in, auto& out) {
    if (in.data.empty() || in.data.size() > 256 * 1024) {
      out.result = result(failure("algorithm payload must be 1..262144 bytes"));
      return true;
    }
    Command c;
    c.kind = CommandKind::Algo;
    c.bytes = in.data;
    out.result = result(command(c));
    return true;
  });
  service<msg::StartFileTransfer>("start_file_transfer", [this](auto& in, auto& out) {
    if (in.direction > 1 || in.file_type > 3 || (in.direction == 0 && in.file_type == 3) ||
        in.path.empty() || in.path.front() != '/') {
      out.result = result(
          failure("invalid direction/type; file path must be absolute; firmware upload only"));
      return true;
    }
    std::error_code path_error;
    if (in.direction == 1 && !std::filesystem::is_regular_file(in.path, path_error)) {
      out.result = result(failure("upload path is not a regular file"));
      return true;
    }
    Command c;
    c.kind = CommandKind::File;
    c.upload = in.direction == 1;
    c.file_type = static_cast<sdk::OdinFileType>(in.file_type);
    c.path = in.path;
    c.timeout_ms = in.timeout_ms ? in.timeout_ms : 5000;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!ready || file_busy || stopping) {
        out.result =
            result(failure("device is not ready or transfer is busy", sdk::OdinResult::kStopped));
        return true;
      }
      file_busy = true;
      out.job_id = next_job++;
      msg::TransferStatus status;
      status.job_id = out.job_id;
      status.state = msg::TransferStatus::QUEUED;
      status.header.stamp = ros::Time::now();
      jobs[out.job_id] = status;
      while (jobs.size() > 64)
        jobs.erase(jobs.begin());
      const auto id = out.job_id;
      controls.emplace_back([this, c, id]() mutable {
        {
          std::lock_guard<std::mutex> guard(mutex);
          jobs.at(id).state = msg::TransferStatus::RUNNING;
          transfer_pub.publish(jobs.at(id));
        }
        c.progress = [this, id](float value) {
          std::lock_guard<std::mutex> guard(mutex);
          auto& job = jobs.at(id);
          job.progress = value;
          job.header.stamp = ros::Time::now();
          transfer_pub.publish(job);
        };
        Reply r;
        try {
          r = (stopping || !handle)
                  ? failure("device unavailable", sdk::OdinResult::kNotInitialized)
                  : sdk->execute(handle, c);
        } catch (const std::exception& e) {
          r = failure(e.what(), sdk::OdinResult::kUnknownError);
        }
        if (c.upload && c.file_type == sdk::OdinFileType::kFirmware) {
          disconnect();
          std::lock_guard<std::mutex> guard(mutex);
          state = "retrying";
        }
        {
          std::lock_guard<std::mutex> guard(mutex);
          auto& job = jobs.at(id);
          job.state = r.ok() ? msg::TransferStatus::SUCCEEDED : msg::TransferStatus::FAILED;
          job.result = result(r);
          job.header.stamp = ros::Time::now();
          if (r.ok())
            job.progress = 100;
          transfer_pub.publish(job);
          file_busy = false;
        }
      });
      transfer_pub.publish(status);
    }
    out.result = result(Reply{});
    control_cv.notify_one();
    return true;
  });
  service<msg::GetTransferStatus>("get_transfer_status", [this](auto& in, auto& out) {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = jobs.find(in.job_id);
    if (it == jobs.end())
      out.result = result(failure("unknown or expired job ID"));
    else {
      out.result = result(Reply{});
      out.status = it->second;
    }
    return true;
  });
}
DeviceDriver::DeviceDriver(ros::NodeHandle node, DeviceSettings settings,
                           std::shared_ptr<SdkFacade> sdk, decoding::Discovery discovery)
    : impl_(std::make_unique<Impl>(std::move(node), std::move(settings), std::move(sdk),
                                   std::move(discovery))) {}
DeviceDriver::~DeviceDriver() = default;
void DeviceDriver::start() {
  impl_->start();
}
void DeviceDriver::stop() {
  impl_->stop();
}
int runDriver(int argc, char** argv, std::shared_ptr<SdkFacade> sdk,
              decoding::Discovery discovery) {
  ros::init(argc, argv, "odin2_driver");
  ros::NodeHandle private_node("~"), root;
  try {
    auto configs = loadConfig(private_node);
    std::vector<std::unique_ptr<DeviceDriver>> devices;
    for (auto& cfg : configs)
      devices.push_back(
          std::make_unique<DeviceDriver>(ros::NodeHandle(root, cfg.name), cfg, sdk, discovery));
    for (auto& device : devices)
      device->start();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    for (auto& device : devices)
      device->stop();
    spinner.stop();
    return 0;
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("Odin2 startup: " << e.what());
    return 1;
  }
}
}  // namespace odin_driver
