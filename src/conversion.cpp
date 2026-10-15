#include "odin2_ros_driver/conversion.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace odin_driver {
namespace {
sensor_msgs::PointField field(const std::string& n, uint32_t offset, uint8_t type) {
  sensor_msgs::PointField f;
  f.name = n;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  return f;
}
template <class T>
void put(std::vector<uint8_t>& data, size_t at, T value) {
  std::memcpy(data.data() + at, &value, sizeof(T));
}
template <class P>
bool invalid(const P& p) {
  return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.x <= 0;
}
sensor_msgs::PointCloud2 cloud(const std_msgs::Header& h, size_t count, uint32_t stride) {
  if (count > std::numeric_limits<uint32_t>::max() / stride)
    throw std::runtime_error("point cloud too large");
  sensor_msgs::PointCloud2 out;
  out.header = h;
  out.height = 1;
  out.width = count;
  out.point_step = stride;
  out.row_step = count * stride;
  out.is_bigendian = false;
  out.is_dense = false;
  out.data.resize(out.row_step);
  out.fields = {field("x", 0, 7), field("y", 4, 7), field("z", 8, 7)};
  return out;
}
geometry_msgs::Quaternion quaternion(const float* q) {
  double norm = 0;
  for (int i = 0; i < 4; ++i)
    norm += static_cast<double>(q[i]) * q[i];
  if (!std::isfinite(norm) || norm < 1e-12)
    throw std::runtime_error("invalid quaternion");
  norm = std::sqrt(norm);
  geometry_msgs::Quaternion out;
  out.x = q[0] / norm;
  out.y = q[1] / norm;
  out.z = q[2] / norm;
  out.w = q[3] / norm;
  return out;
}
}  // namespace
sensor_msgs::PointCloud2 rawCloud(const sdk::OdinPointCloudPacket& p, const std_msgs::Header& h) {
  constexpr uint32_t kPointStep = 18;
  constexpr size_t kPointsPerGroup = 256 * 6;
  constexpr float kSubframeInterval = 1.0f / 328.5f;
  auto out = cloud(h, p.points.size(), kPointStep);
  out.fields.push_back(field("intensity", 12, sensor_msgs::PointField::UINT8));
  out.fields.push_back(field("confidence", 13, sensor_msgs::PointField::UINT8));
  out.fields.push_back(field("offset_time", 14, sensor_msgs::PointField::FLOAT32));
  for (size_t i = 0; i < p.points.size(); ++i) {
    const auto& v = p.points[i];
    const bool bad = invalid(v);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const size_t at = i * kPointStep;
    put(out.data, at, bad ? nan : v.x * 0.001f);
    put(out.data, at + 4, bad ? nan : v.y * 0.001f);
    put(out.data, at + 8, bad ? nan : v.z * 0.001f);
    out.data[at + 12] = v.intensity;
    out.data[at + 13] = v.confidence;
    // Preserve invalid points so the rolling scan group follows the SDK index.
    // This float is deliberately unaligned in the official 18-byte wire layout.
    put(out.data, at + 14, (i / kPointsPerGroup) * kSubframeInterval);
  }
  return out;
}
sensor_msgs::PointCloud2 slamCloud(const sdk::OdinSlamPacket& p, const std_msgs::Header& h) {
  auto out = cloud(h, p.points.size(), 16);
  out.fields.push_back(field("rgba", 12, 6));
  for (size_t i = 0; i < p.points.size(); ++i) {
    const auto& v = p.points[i];  // world-transformed points can legitimately have x <= 0
    const bool bad = !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
                     (v.x == 0 && v.y == 0 && v.z == 0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    put(out.data, i * 16, bad ? nan : v.x * 0.001f);
    put(out.data, i * 16 + 4, bad ? nan : v.y * 0.001f);
    put(out.data, i * 16 + 8, bad ? nan : v.z * 0.001f);
    uint32_t rgba = (uint32_t(v.a) << 24) | (uint32_t(v.r) << 16) | (uint32_t(v.g) << 8) | v.b;
    put(out.data, i * 16 + 12, rgba);
  }
  return out;
}
sensor_msgs::Imu imuMessage(const sdk::OdinImuSample& p, const std_msgs::Header& h) {
  const float values[] = {p.gyro_x, p.gyro_y, p.gyro_z, p.acc_x, p.acc_y, p.acc_z};
  for (float v : values)
    if (!std::isfinite(v))
      throw std::runtime_error("non-finite IMU sample");
  sensor_msgs::Imu out;
  out.header = h;
  out.orientation.w = 1;
  out.orientation_covariance[0] = -1;
  out.angular_velocity.x = p.gyro_x;
  out.angular_velocity.y = p.gyro_y;
  out.angular_velocity.z = p.gyro_z;
  out.linear_acceleration.x = p.acc_x;
  out.linear_acceleration.y = p.acc_y;
  out.linear_acceleration.z = p.acc_z;
  return out;
}
nav_msgs::Odometry odomMessage(const sdk::OdinOdomData& p, const std_msgs::Header& h,
                               const DeviceSettings& cfg) {
  nav_msgs::Odometry out;
  out.header = h;
  out.child_frame_id = prefixedFrame(cfg, "imu");
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(p.pos[i]) || !std::isfinite(p.linear_velocity[i]) ||
        !std::isfinite(p.angular_velocity[i]))
      throw std::runtime_error("non-finite odometry");
  out.pose.pose.position.x = p.pos[0];
  out.pose.pose.position.y = p.pos[1];
  out.pose.pose.position.z = p.pos[2];
  out.pose.pose.orientation = quaternion(p.orient);
  out.twist.twist.linear.x = p.linear_velocity[0];
  out.twist.twist.linear.y = p.linear_velocity[1];
  out.twist.twist.linear.z = p.linear_velocity[2];
  out.twist.twist.angular.x = p.angular_velocity[0];
  out.twist.twist.angular.y = p.angular_velocity[1];
  out.twist.twist.angular.z = p.angular_velocity[2];
  std::copy(cfg.pose_covariance.begin(), cfg.pose_covariance.end(), out.pose.covariance.begin());
  std::copy(cfg.twist_covariance.begin(), cfg.twist_covariance.end(), out.twist.covariance.begin());
  return out;
}
odin2_ros_driver::OdomPacket odomPacket(const sdk::OdinOdomPacket& p, const std_msgs::Header& h,
                                        uint8_t type) {
  odin2_ros_driver::OdomPacket out;
  out.header = h;
  out.device_timestamp_ns = p.timestamp;
  out.frame_count = p.frame_count;
  out.source_type = type;
  for (const auto& v : p.data) {
    odin2_ros_driver::OdomSample s;
    s.position.x = v.pos[0];
    s.position.y = v.pos[1];
    s.position.z = v.pos[2];
    s.orientation.x = v.orient[0];
    s.orientation.y = v.orient[1];
    s.orientation.z = v.orient[2];
    s.orientation.w = v.orient[3];
    s.linear_velocity.x = v.linear_velocity[0];
    s.linear_velocity.y = v.linear_velocity[1];
    s.linear_velocity.z = v.linear_velocity[2];
    s.angular_velocity.x = v.angular_velocity[0];
    s.angular_velocity.y = v.angular_velocity[1];
    s.angular_velocity.z = v.angular_velocity[2];
    std::copy(v.cov, v.cov + 18, s.sdk_covariance.begin());
    s.source_type = v.type;
    out.samples.push_back(s);
  }
  return out;
}
geometry_msgs::TransformStamped tfMessage(const sdk::OdinTfPacket& p, const std_msgs::Header& h,
                                          const DeviceSettings& cfg) {
  geometry_msgs::TransformStamped out;
  out.header = h;
  out.header.frame_id = prefixedFrame(cfg, p.frame_id);
  out.child_frame_id = prefixedFrame(cfg, p.child_frame_id);
  if (out.header.frame_id == out.child_frame_id)
    throw std::runtime_error("TF parent equals child");
  for (float v : p.translation)
    if (!std::isfinite(v))
      throw std::runtime_error("non-finite TF translation");
  out.transform.translation.x = p.translation[0];
  out.transform.translation.y = p.translation[1];
  out.transform.translation.z = p.translation[2];
  out.transform.rotation = quaternion(p.rotation);
  return out;
}
odin2_ros_driver::Telemetry telemetryMessage(const sdk::OdinTelemetryPacket& p,
                                             const std_msgs::Header& h) {
  odin2_ros_driver::Telemetry out;
  out.header = h;
  out.device_timestamp_ns = p.timestamp;
  out.frame_count = p.frame_count;
  out.uptime_s = p.data.uptime_s;
  out.cpu_pct = p.data.cpu_pct;
  out.mem_used_kb = p.data.mem_used_kb;
  out.mem_total_kb = p.data.mem_total_kb;
  out.mem_use_rate = p.data.mem_use_rate;
  out.load_1 = p.data.load_1;
  out.load_5 = p.data.load_5;
  out.load_15 = p.data.load_15;
  return out;
}
bool cameraKeyframe(const sdk::OdinCameraPacket& p) {
  if (p.format == sdk::OdinDataFormat::kImageMjpeg || p.format == sdk::OdinDataFormat::kImageNv12)
    return true;
  for (size_t i = 0; i + 4 < p.payload.size(); ++i) {
    size_t n = 0;
    if (p.payload[i] == 0 && p.payload[i + 1] == 0 && p.payload[i + 2] == 1)
      n = i + 3;
    else if (p.payload[i] == 0 && p.payload[i + 1] == 0 && p.payload[i + 2] == 0 &&
             p.payload[i + 3] == 1)
      n = i + 4;
    if (!n || n >= p.payload.size())
      continue;
    unsigned type =
        p.format == sdk::OdinDataFormat::kImageH265 ? (p.payload[n] >> 1) & 63 : p.payload[n] & 31;
    if ((p.format == sdk::OdinDataFormat::kImageH264 && type == 5) ||
        (p.format == sdk::OdinDataFormat::kImageH265 && type >= 16 && type <= 21))
      return true;
  }
  return false;
}
}  // namespace odin_driver
