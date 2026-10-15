#include <XmlRpcValue.h>
#include <sched.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include "odin2_ros_driver/core.hpp"

namespace odin_driver {
const std::array<Channel, 9>& channels() {
  static const std::array<Channel, 9> list{
      {Channel::kRawPoint, Channel::kSlamPoint, Channel::kImage0, Channel::kImage1, Channel::kImu,
       Channel::kOdomLF, Channel::kOdomHF, Channel::kTf, Channel::kTelemetry}};
  return list;
}
std::string channelName(Channel c) {
  switch (c) {
    case Channel::kRawPoint:
      return "raw_point";
    case Channel::kSlamPoint:
      return "slam_point";
    case Channel::kImage0:
      return "camera0";
    case Channel::kImage1:
      return "camera1";
    case Channel::kImu:
      return "imu";
    case Channel::kOdomLF:
      return "odom_lf";
    case Channel::kOdomHF:
      return "odom_hf";
    case Channel::kTf:
      return "tf";
    case Channel::kTelemetry:
      return "telemetry";
    default:
      throw std::invalid_argument("Unknown data channel");
  }
}
bool isCamera(Channel c) {
  return c == Channel::kImage0 || c == Channel::kImage1;
}
bool isKnownChannel(uint8_t c) {
  return std::find(channels().begin(), channels().end(), static_cast<Channel>(c)) !=
         channels().end();
}
DeviceSettings defaults() {
  DeviceSettings cfg;
  const std::array<std::string, 9> topics{{"points_raw", "points_slam", "camera0/image_raw",
                                           "camera1/image_raw", "imu/data_raw", "odom_lf",
                                           "odom_hf", "transforms", "telemetry"}};
  const std::array<std::string, 9> frames{
      {"lidar", "imu", "camera0", "camera1", "imu", "odom", "odom", "odom", "base_link"}};
  for (size_t i = 0; i < channels().size(); ++i) {
    StreamSettings s;
    s.topic = topics[i];
    s.frame = frames[i];
    s.enabled = i == 0 || i == 4 || i == 8;
    if (isCamera(channels()[i])) {
      s.transport = sdk::OdinTransportMode::kTcp;
      s.mode.format = sdk::OdinDataFormat::kImageMjpeg;
    }
    if (i == 4)
      s.queue_size = 2000;
    if (i == 5 || i == 6 || i == 7)
      s.queue_size = 256;
    if (i == 8) {
      s.queue_size = 16;
      s.transport = sdk::OdinTransportMode::kTcp;
    }
    cfg.streams[channels()[i]] = s;
  }
  for (size_t i = 0; i < 6; ++i) {
    cfg.pose_covariance[i * 7] = 1e6;
    cfg.twist_covariance[i * 7] = 1e6;
  }
  return cfg;
}
namespace {
using Xml = XmlRpc::XmlRpcValue;
void keys(Xml& value, const std::set<std::string>& allowed, const std::string& scope) {
  for (auto it = value.begin(); it != value.end(); ++it)
    if (!allowed.count(it->first))
      throw std::invalid_argument(scope + ": unknown parameter " + it->first);
}
std::string str(Xml& x, const std::string& key, const std::string& fallback) {
  if (!x.hasMember(key))
    return fallback;
  if (x[key].getType() != Xml::TypeString)
    throw std::invalid_argument(key + " must be a string");
  return static_cast<std::string>(x[key]);
}
double number(Xml& x, const std::string& key, double fallback) {
  if (!x.hasMember(key))
    return fallback;
  auto& v = x[key];
  if (v.getType() == Xml::TypeInt)
    return static_cast<int>(v);
  if (v.getType() == Xml::TypeDouble)
    return static_cast<double>(v);
  throw std::invalid_argument(key + " must be numeric");
}
uint64_t integer(Xml& x, const std::string& key, uint64_t fallback, uint64_t maximum) {
  double n = number(x, key, static_cast<double>(fallback));
  if (!std::isfinite(n) || n < 0 || std::floor(n) != n || n > static_cast<double>(maximum))
    throw std::invalid_argument(key + " is outside its integer range");
  return static_cast<uint64_t>(n);
}
bool boolean(Xml& x, const std::string& key, bool fallback) {
  if (!x.hasMember(key))
    return fallback;
  if (x[key].getType() != Xml::TypeBoolean)
    throw std::invalid_argument(key + " must be boolean");
  return static_cast<bool>(x[key]);
}
void covariance(Xml& x, const std::string& key, std::array<double, 36>& out) {
  auto& v = x[key];
  if (v.getType() != Xml::TypeArray || v.size() != 36)
    throw std::invalid_argument(key + " needs 36 entries");
  for (int i = 0; i < 36; ++i) {
    if (v[i].getType() == Xml::TypeInt)
      out[i] = static_cast<int>(v[i]);
    else if (v[i].getType() == Xml::TypeDouble)
      out[i] = static_cast<double>(v[i]);
    else
      throw std::invalid_argument(key + " entries must be numeric");
    if (!std::isfinite(out[i]))
      throw std::invalid_argument(key + " must be finite");
  }
  for (int i = 0; i < 6; ++i)
    if (out[i * 7] < 0)
      throw std::invalid_argument(key + " has negative variance");
}
}  // namespace
std::vector<DeviceSettings> loadConfig(const ros::NodeHandle& node) {
  Xml devices;
  if (!node.getParam("devices", devices) || devices.getType() != Xml::TypeArray ||
      devices.size() == 0)
    throw std::invalid_argument("~devices must be a nonempty array; load config/default.yaml");
  std::vector<DeviceSettings> list;
  std::set<std::string> names, identities, prefixes;
  for (int i = 0; i < devices.size(); ++i) {
    auto& x = devices[i];
    if (x.getType() != Xml::TypeStruct)
      throw std::invalid_argument("device entry must be a mapping");
    keys(x,
         {"name", "sn", "device_ip", "host_ip", "tf_prefix", "time_mode", "time_offset_ns",
          "slam_world", "broadcast_tf", "sensor_mode", "filters", "sync_lag", "command_timeout_ms",
          "worker_threads", "cpu_affinity", "pose_covariance", "twist_covariance", "streams"},
         "device");
    auto cfg = defaults();
    cfg.name = str(x, "name", cfg.name);
    if (cfg.name.empty() ||
        cfg.name.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") != std::string::npos)
      throw std::invalid_argument("device name must contain only letters, digits or underscore");
    std::string name_error;
    if (!ros::names::validate(cfg.name, name_error))
      throw std::invalid_argument(name_error);
    cfg.sn = str(x, "sn", "");
    cfg.device_ip = str(x, "device_ip", "");
    cfg.host_ip = str(x, "host_ip", "");
    cfg.tf_prefix = str(x, "tf_prefix", cfg.name);
    if (cfg.tf_prefix.empty() || cfg.tf_prefix.front() == '/' ||
        cfg.tf_prefix.find_first_of(" \t\n") != std::string::npos)
      throw std::invalid_argument("tf_prefix must be nonempty and relative");
    cfg.time_mode = str(x, "time_mode", "mapped");
    if (cfg.time_mode != "mapped" && cfg.time_mode != "receive" && cfg.time_mode != "offset")
      throw std::invalid_argument("time_mode: mapped/receive/offset");
    cfg.time_offset_ns = std::stoll(str(x, "time_offset_ns", "0"));
    cfg.slam_world = boolean(x, "slam_world", false);
    cfg.broadcast_tf = boolean(x, "broadcast_tf", false);
    if (x.hasMember("sensor_mode")) {
      const std::map<std::string, uint8_t> modes{
          {"hdr", 0}, {"high_peak", 1}, {"low_peak", 2}, {"adaptive", 3}};
      cfg.sensor_mode = modes.at(str(x, "sensor_mode", "hdr"));
    }
    if (x.hasMember("filters")) {
      auto& v = x["filters"];
      if (v.getType() != Xml::TypeStruct)
        throw std::invalid_argument("filters must be a mapping");
      keys(v, {"confidence_threshold", "min_depth", "cdf_percentile", "outdoor_threshold"},
           "filters");
      sdk::OdinLidarFilterParams f;
      f.has_confidence = v.hasMember("confidence_threshold");
      f.confidence.confidence_threshold = integer(v, "confidence_threshold", 0, 255);
      f.has_range = v.hasMember("min_depth");
      f.range.min_depth = integer(v, "min_depth", 150, 65535);
      f.has_adaptive = v.hasMember("cdf_percentile") || v.hasMember("outdoor_threshold");
      f.adaptive.cdf_percentile = integer(v, "cdf_percentile", 7000, 10000);
      f.adaptive.outdoor_threshold = integer(v, "outdoor_threshold", 3000, 65535);
      validateFilters(f);
      cfg.initial_adaptive = f.adaptive;
    }
    cfg.sync_lag = integer(x, "sync_lag", 10, 1000);
    cfg.command_timeout_ms = integer(x, "command_timeout_ms", 1000, 60000);
    cfg.worker_threads = integer(x, "worker_threads", 2, 32);
    if (cfg.worker_threads == 0 || cfg.sync_lag == 0 || cfg.command_timeout_ms == 0)
      throw std::invalid_argument("thread/timeout/lag must be positive");
    if (x.hasMember("cpu_affinity")) {
      auto& cpus = x["cpu_affinity"];
      if (cpus.getType() != Xml::TypeArray)
        throw std::invalid_argument("cpu_affinity must be an array");
      for (int j = 0; j < cpus.size(); ++j) {
        if (cpus[j].getType() != Xml::TypeInt || static_cast<int>(cpus[j]) < 0 ||
            static_cast<int>(cpus[j]) >= CPU_SETSIZE)
          throw std::invalid_argument("invalid CPU affinity");
        cfg.cpu_affinity.push_back(static_cast<int>(cpus[j]));
      }
    }
    bool p = x.hasMember("pose_covariance"), t = x.hasMember("twist_covariance");
    if (p != t)
      throw std::invalid_argument("configure both pose_covariance and twist_covariance");
    if (p) {
      covariance(x, "pose_covariance", cfg.pose_covariance);
      covariance(x, "twist_covariance", cfg.twist_covariance);
      cfg.covariance_configured = true;
    }
    if (x.hasMember("streams")) {
      auto& streams = x["streams"];
      if (streams.getType() != Xml::TypeStruct)
        throw std::invalid_argument("streams must be a mapping");
      std::set<std::string> known;
      for (auto c : channels())
        known.insert(channelName(c));
      for (auto it = streams.begin(); it != streams.end(); ++it)
        if (!known.count(it->first))
          throw std::invalid_argument("unknown stream: " + it->first);
      for (auto c : channels()) {
        const auto name = channelName(c);
        if (!streams.hasMember(name))
          continue;
        auto& v = streams[name];
        if (v.getType() != Xml::TypeStruct)
          throw std::invalid_argument("stream must be a mapping");
        auto allowed = std::set<std::string>{
            "enabled",     "topic",  "frame_id", "transport", "publish_rate", "queue_size",
            "queue_bytes", "format", "width",    "height",    "fps"};
        if (isCamera(c))
          allowed.insert({"publish_decoded", "publish_encoded", "camera_info_url"});
        keys(v, allowed, name);
        auto& s = cfg.streams[c];
        s.enabled = boolean(v, "enabled", s.enabled);
        s.topic = str(v, "topic", s.topic);
        s.frame = str(v, "frame_id", s.frame);
        auto transport =
            str(v, "transport", s.transport == sdk::OdinTransportMode::kTcp ? "tcp" : "udp");
        if (transport != "tcp" && transport != "udp")
          throw std::invalid_argument("transport: tcp/udp");
        s.transport =
            transport == "tcp" ? sdk::OdinTransportMode::kTcp : sdk::OdinTransportMode::kUdp;
        s.publish_rate = number(v, "publish_rate", 0);
        s.queue_size = integer(v, "queue_size", s.queue_size, 1000000);
        s.queue_bytes = integer(v, "queue_bytes", s.queue_bytes, 1024ULL * 1024 * 1024);
        if (s.queue_size == 0 || s.queue_bytes == 0 || !std::isfinite(s.publish_rate) ||
            s.publish_rate < 0 || s.topic.empty() || s.topic.front() == '/' || s.frame.empty())
          throw std::invalid_argument(
              "invalid stream rate/queue/topic/frame (topics must be relative)");
        s.mode.width = integer(v, "width", 0, 65535);
        s.mode.height = integer(v, "height", 0, 65535);
        s.mode.fps = integer(v, "fps", 0, 65535);
        if (!isCamera(c) && (v.hasMember("format") || s.mode.width || s.mode.height || s.mode.fps))
          throw std::invalid_argument("SDK accepts acquisition mode only for cameras");
        if (isCamera(c)) {
          const std::map<std::string, sdk::OdinDataFormat> formats{
              {"nv12", sdk::OdinDataFormat::kImageNv12},
              {"mjpeg", sdk::OdinDataFormat::kImageMjpeg},
              {"h264", sdk::OdinDataFormat::kImageH264},
              {"h265", sdk::OdinDataFormat::kImageH265}};
          s.mode.format = formats.at(str(v, "format", "mjpeg"));
          s.publish_decoded = boolean(v, "publish_decoded", true);
          s.publish_encoded = boolean(v, "publish_encoded", true);
          s.camera_info_url = str(v, "camera_info_url", "");
        }
      }
    }
    std::set<std::string> topic_names;
    for (const auto& pair : cfg.streams) {
      const auto& stream = pair.second;
      std::string error;
      if (!ros::names::validate(stream.topic, error))
        throw std::invalid_argument(error);
      if (!topic_names.insert(stream.topic).second)
        throw std::invalid_argument("duplicate stream topic");
      if (stream.frame.find_first_of(" \t\n") != std::string::npos)
        throw std::invalid_argument("frame_id contains whitespace");
    }
    if (cfg.slam_world)
      cfg.streams[Channel::kSlamPoint].frame = "odom";
    const auto identity = !cfg.sn.empty()          ? "sn:" + cfg.sn
                          : !cfg.device_ip.empty() ? "ip:" + cfg.device_ip
                                                   : "automatic";
    if (!names.insert(cfg.name).second || !identities.insert(identity).second ||
        !prefixes.insert(cfg.tf_prefix).second)
      throw std::invalid_argument("device names, identities and TF prefixes must be unique");
    if (devices.size() > 1 && identity == "automatic")
      throw std::invalid_argument("multi-device configuration requires SN or IP for every device");
    list.push_back(std::move(cfg));
  }
  return list;
}
void validateStream(Channel c, const StreamSettings& s,
                    const std::vector<sdk::OdinSensorCapability>& caps) {
  if (!isKnownChannel(static_cast<uint8_t>(c)))
    throw std::invalid_argument("unsupported channel");
  if (static_cast<uint8_t>(s.transport) > 1 || !std::isfinite(s.publish_rate) || s.publish_rate < 0)
    throw std::invalid_argument("invalid transport or rate");
  if (c == Channel::kTelemetry && s.transport != sdk::OdinTransportMode::kTcp)
    throw std::invalid_argument("telemetry requires TCP");
  if (!isCamera(c) && (s.mode.width || s.mode.height || s.mode.fps ||
                       s.mode.format != sdk::OdinDataFormat::kUnknown))
    throw std::invalid_argument("only camera channels accept acquisition mode");
  if (!s.enabled)
    return;
  if (c == Channel::kTelemetry)
    return;  // functional channels need not appear in capability tables
  auto it =
      std::find_if(caps.begin(), caps.end(), [c](const auto& cap) { return cap.channel == c; });
  if (it == caps.end())
    throw std::invalid_argument("device does not advertise " + channelName(c));
  if (!isCamera(c))
    return;
  uint8_t f = static_cast<uint8_t>(s.mode.format);
  if (f != 1 && f != 2 && f != 4 && f != 8)
    throw std::invalid_argument("select one camera format bit");
  if ((s.mode.width == 0) != (s.mode.height == 0))
    throw std::invalid_argument("set both width and height");
  if (s.mode.fps && !s.mode.width)
    throw std::invalid_argument("SDK fps selection requires width and height");
  if (f == 1 && (!s.mode.width || s.mode.width % 2 || s.mode.height % 2))
    throw std::invalid_argument("NV12 requires explicit even dimensions");
  if (!std::any_of(it->modes.begin(), it->modes.end(), [&](const auto& m) {
        return m.format != sdk::OdinDataFormat::kUnknown &&
               (static_cast<uint8_t>(m.format) & f) == f &&
               (!s.mode.width || (m.width == s.mode.width && m.height == s.mode.height)) &&
               (!s.mode.fps || m.fps == s.mode.fps);
      }))
    throw std::invalid_argument("camera format/resolution/fps combination is not advertised");
}
void validateFilters(const sdk::OdinLidarFilterParams& f) {
  if (f.has_confidence || f.has_range)
    throw std::invalid_argument(
        "UNSUPPORTED_FILTER: confidence_threshold/min_depth are not supported by firmware; "
        "only adaptive parameters can be configured");
  if (!f.has_confidence && !f.has_adaptive && !f.has_range)
    throw std::invalid_argument("at least one filter must be selected");
  if (f.has_adaptive && (!f.adaptive.cdf_percentile || f.adaptive.cdf_percentile > 10000))
    throw std::invalid_argument("cdf_percentile must be 1..10000");
}
std::string prefixedFrame(const DeviceSettings& cfg, const std::string& frame) {
  auto f = frame;
  while (!f.empty() && f.front() == '/')
    f.erase(0, 1);
  if (f.empty())
    throw std::invalid_argument("empty TF frame");
  auto p = cfg.tf_prefix;
  while (!p.empty() && p.back() == '/')
    p.pop_back();
  if (p.empty() || f.rfind(p + "/", 0) == 0)
    return f;
  return p + "/" + f;
}
size_t packetBytes(const Packet& p) {
  return std::visit(
      [](const auto& v) -> size_t {
        using T = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<T, sdk::OdinPointCloudPacket> ||
                      std::is_same_v<T, sdk::OdinSlamPacket>)
          return sizeof(T) +
                 v.points.size() * sizeof(typename std::decay_t<decltype(v.points)>::value_type);
        else if constexpr (std::is_same_v<T, sdk::OdinCameraPacket>)
          return sizeof(T) + v.payload.size();
        else if constexpr (std::is_same_v<T, sdk::OdinImuPacket>)
          return sizeof(T) + v.samples.size() * sizeof(sdk::OdinImuSample);
        else if constexpr (std::is_same_v<T, sdk::OdinOdomPacket>)
          return sizeof(T) + v.data.size() * sizeof(sdk::OdinOdomData);
        else if constexpr (std::is_same_v<T, sdk::OdinTfPacket>)
          return sizeof(T) + v.frame_id.size() + v.child_frame_id.size();
        else
          return sizeof(T);
      },
      p);
}
uint64_t packetTimestamp(const Packet& p) {
  return std::visit([](const auto& v) { return v.timestamp; }, p);
}
std::optional<uint32_t> packetSequence(const Packet& p) {
  return std::visit(
      [](const auto& v) -> std::optional<uint32_t> {
        if constexpr (std::is_same_v<std::decay_t<decltype(v)>, sdk::OdinImuPacket>)
          return {};
        else
          return v.frame_count;
      },
      p);
}
std::optional<ros::Time> TimeMapper::map(Channel c, uint64_t ns, ros::Time received,
                                         bool& reset_epoch) {
  reset_epoch = false;
  if (received.isZero())
    return {};
  if (ns > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()))
    return {};
  const auto wall = ros::WallTime::now();
  auto last = last_device_.find(c);
  if ((!last_host_.isZero() &&
       (received < last_host_ ||
        std::abs((received - last_host_).toSec() - (wall - last_wall_).toSec()) > 5)) ||
      (last != last_device_.end() && last->second > ns && last->second - ns > 1000000000ULL)) {
    reset();
    reset_epoch = true;
    last = last_device_.end();
  }
  if (last != last_device_.end() && ns < last->second)
    return {};
  last_device_[c] = ns;
  last_host_ = received;
  last_wall_ = wall;
  if (!anchored_) {
    anchor_device_ = ns;
    anchor_ros_ = received.toNSec();
    anchored_ = true;
  }
  int64_t stamp = 0;
  if (mode_ == "receive")
    stamp = static_cast<int64_t>(received.toNSec());
  else if (mode_ == "offset") {
    if (offset_ > 0 && ns > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() - offset_))
      return {};
    stamp = static_cast<int64_t>(ns) + offset_;
  } else {
    const int64_t delta = static_cast<int64_t>(ns) - static_cast<int64_t>(anchor_device_);
    if (delta > 0 &&
        anchor_ros_ > static_cast<uint64_t>(std::numeric_limits<int64_t>::max() - delta))
      return {};
    stamp = static_cast<int64_t>(anchor_ros_) + delta;
  }
  if (stamp <= 0 ||
      static_cast<uint64_t>(stamp) / 1000000000ULL > std::numeric_limits<uint32_t>::max())
    return {};
  ros::Time result;
  result.fromNSec(static_cast<uint64_t>(stamp));
  return result;
}
}  // namespace odin_driver
