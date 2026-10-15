#pragma once
#include <odin_lidar_api_ext.h>
#include <ros/ros.h>

#include <array>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <variant>

namespace odin_driver {
namespace sdk = odin::sdk;
using Channel = sdk::OdinDataChannel;
using Packet = std::variant<sdk::OdinPointCloudPacket, sdk::OdinSlamPacket, sdk::OdinCameraPacket,
                            sdk::OdinImuPacket, sdk::OdinOdomPacket, sdk::OdinTfPacket,
                            sdk::OdinTelemetryPacket>;
const std::array<Channel, 9>& channels();
std::string channelName(Channel channel);
bool isCamera(Channel channel);
bool isKnownChannel(uint8_t channel);
struct StreamSettings {
  bool enabled = false;
  sdk::OdinTransportMode transport = sdk::OdinTransportMode::kUdp;
  sdk::OdinStreamCfg mode;
  std::string topic, frame;
  double publish_rate = 0;
  size_t queue_size = 4, queue_bytes = 32 * 1024 * 1024;
  bool publish_decoded = true, publish_encoded = true;
  std::string camera_info_url;
};
struct DeviceSettings {
  std::string name = "odin2", sn, device_ip, host_ip, tf_prefix = "odin2";
  std::string time_mode = "mapped";
  int64_t time_offset_ns = 0;
  bool slam_world = false, broadcast_tf = false, covariance_configured = false;
  std::optional<uint8_t> sensor_mode;
  std::optional<sdk::OdinAdaptiveFilterParam> initial_adaptive;
  uint32_t sync_lag = 10, command_timeout_ms = 1000;
  size_t worker_threads = 2;
  std::vector<int> cpu_affinity;
  std::array<double, 36> pose_covariance{}, twist_covariance{};
  std::map<Channel, StreamSettings> streams;
};
DeviceSettings defaults();
std::vector<DeviceSettings> loadConfig(const ros::NodeHandle& node);
void validateStream(Channel channel, const StreamSettings& stream,
                    const std::vector<sdk::OdinSensorCapability>& caps);
void validateFilters(const sdk::OdinLidarFilterParams& filters);
std::string prefixedFrame(const DeviceSettings& cfg, const std::string& frame);

struct Envelope {
  Channel channel;
  Packet packet;
  ros::Time received;
  ros::Time stamp;
  ros::WallTime wall_received;
  uint64_t generation = 0, decode_generation = 0;
  size_t bytes = 0;
};
size_t packetBytes(const Packet& packet);
uint64_t packetTimestamp(const Packet& packet);
std::optional<uint32_t> packetSequence(const Packet& packet);
// Bounded, externally synchronized queue: never keeps references to SDK memory.
class BoundedQueue {
 public:
  void configure(size_t count, size_t bytes) {
    limit_ = count;
    byte_limit_ = bytes;
    clear();
  }
  bool wouldOverflow(size_t bytes) const {
    return bytes > byte_limit_ || queue_.size() >= limit_ || bytes_ + bytes > byte_limit_;
  }
  uint64_t push(Envelope value) {
    uint64_t dropped = 0;
    if (value.bytes > byte_limit_)
      return 1;
    while (!queue_.empty() && (queue_.size() >= limit_ || bytes_ + value.bytes > byte_limit_)) {
      bytes_ -= queue_.front().bytes;
      queue_.pop_front();
      ++dropped;
    }
    bytes_ += value.bytes;
    queue_.push_back(std::move(value));
    return dropped;
  }
  Envelope pop() {
    auto v = std::move(queue_.front());
    bytes_ -= v.bytes;
    queue_.pop_front();
    return v;
  }
  void clear() {
    queue_.clear();
    bytes_ = 0;
  }
  bool empty() const {
    return queue_.empty();
  }
  size_t size() const {
    return queue_.size();
  }
  size_t bytes() const {
    return bytes_;
  }

 private:
  size_t limit_ = 4, byte_limit_ = 32 * 1024 * 1024, bytes_ = 0;
  std::deque<Envelope> queue_;
};
class TimeMapper {
 public:
  TimeMapper(std::string mode = "mapped", int64_t offset = 0)
      : mode_(std::move(mode)), offset_(offset) {}
  void reset() {
    anchored_ = false;
    last_device_.clear();
    last_host_ = {};
    last_wall_ = {};
    ++resets_;
  }
  // reset_epoch signals callers to clear queued packets and decoder state.
  std::optional<ros::Time> map(Channel channel, uint64_t device_ns, ros::Time received,
                               bool& reset_epoch);
  uint64_t resets() const {
    return resets_;
  }

 private:
  std::string mode_;
  int64_t offset_ = 0;
  uint64_t anchor_device_ = 0, anchor_ros_ = 0, resets_ = 0;
  bool anchored_ = false;
  ros::Time last_host_;
  ros::WallTime last_wall_;
  std::map<Channel, uint64_t> last_device_;
};
}  // namespace odin_driver
