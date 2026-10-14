#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <vector>

#include "OdinProtocol.hpp"
#include "odin_lidar_def.h"
#include "internal/odin_lidar_inner.h"

namespace odin {
namespace sdk {

/**
 * @brief TCP stream buffer for frame reassembly
 */
struct TcpStreamBuffer {
  std::vector<uint8_t> buffer;
  
  void Append(const uint8_t* data, size_t len) {
    buffer.insert(buffer.end(), data, data + len);
  }
  
  void Clear() { buffer.clear(); }
  
  size_t Size() const { return buffer.size(); }
  
  const uint8_t* Data() const { return buffer.data(); }
  
  void Consume(size_t len) {
    if (len >= buffer.size()) {
      buffer.clear();
    } else {
      buffer.erase(buffer.begin(), buffer.begin() + len);
    }
  }
};

/**
 * @brief Point cloud frame assembly state
 */
struct PointFrameState {
  OdinPointCloudPacketFull packet;
  uint16_t expected_chunks = 0;   ///< header.chunk_count of the in-progress frame
  uint16_t received_chunks = 0;   ///< chunks accumulated so far
  uint16_t last_chunk_index = 0;  ///< last seen chunk_index (gap detection)
  bool last_chunk_received = false;  ///< true if the frame was closed by last_chunk flag
  bool active = false;

  void Reset() {
    packet = OdinPointCloudPacketFull();
    expected_chunks = 0;
    received_chunks = 0;
    last_chunk_index = 0;
    last_chunk_received = false;
    active = false;
  }
};

/**
 * @brief Image frame assembly state
 */
struct ImageFrameState {
  OdinImagePacketFull packet;
  uint16_t expected_chunks = 0;   ///< header.chunk_count of the in-progress frame
  uint16_t received_chunks = 0;   ///< chunks accumulated so far
  uint16_t last_chunk_index = 0;  ///< last seen chunk_index (gap detection)
  bool active = false;

  void Reset() {
    packet = OdinImagePacketFull();
    expected_chunks = 0;
    received_chunks = 0;
    last_chunk_index = 0;
    active = false;
  }
};

/**
 * @brief H264 frame assembly state
 */
struct H264FrameState {
  OdinCameraPacket packet;
  uint16_t expected_chunks = 0;
  uint16_t received_chunks = 0;
  uint16_t last_chunk_index = 0;
  bool active = false;

  void Reset() {
    packet = OdinCameraPacket();
    expected_chunks = 0;
    received_chunks = 0;
    last_chunk_index = 0;
    active = false;
  }
};

/**
 * @brief Odin protocol frame assembler
 * 
 * Handles frame assembly from UDP/TCP packets for different data types.
 * Supports point cloud, image, IMU, and odometry data.
 */
class OdinFrameAssembler {
 public:
  OdinFrameAssembler(OdinDeviceHandle handle, OdinTransportMode transport);
  ~OdinFrameAssembler() = default;

  // Callback types
  using PointCloudCallback = std::function<void(const OdinPointCloudPacket&)>;
  using SlamCallback = std::function<void(const OdinSlamPacket&)>;
  using ImageCallback = std::function<void(const OdinCameraPacket&)>;
  using H264Callback = std::function<void(const OdinCameraPacket&)>;
  using ImuCallback = std::function<void(const OdinImuPacket&)>;
  using OdomCallback = std::function<void(const OdinOdomPacket&, OdomSourceType)>;
  using TfCallback = std::function<void(const OdinTfPacket&)>;

  // Set callbacks
  // Format stamped on frames emitted via the image (non-H264) path, e.g.
  // kImageMjpeg or kImageNv12. Set from the StartStream config so EmitImageFrame
  // reports what the device was actually asked to send.
  void SetImageFormat(OdinDataFormat format) { image_format_ = format; }
  void SetPointCloudCallback(PointCloudCallback cb) { point_cloud_cb_ = cb; }
  void SetSlamCallback(SlamCallback cb) { slam_cb_ = cb; }
  void SetImageCallback(ImageCallback cb) { image_cb_ = cb; }
  void SetH264Callback(H264Callback cb) { h264_cb_ = cb; }
  void SetImuCallback(ImuCallback cb) { imu_cb_ = cb; }
  void SetOdomCallback(OdomCallback cb) { odom_cb_ = cb; }
  void SetTfCallback(TfCallback cb) { tf_cb_ = cb; }

  // Process raw data for different channel types
  void ProcessRawPointData(const uint8_t* data, size_t length);
  void ProcessSlamPointData(const uint8_t* data, size_t length);
  void ProcessImageData(const uint8_t* data, size_t length);
  void ProcessH264Data(const uint8_t* data, size_t length);
  void ProcessImuData(const uint8_t* data, size_t length);
  void ProcessOdomData(const uint8_t* data, size_t length);
  void ProcessTfData(const uint8_t* data, size_t length);

  // Reset state
  void Reset();

 private:
  // Parsing helpers
  bool ParsePointCloudPacket(const uint8_t* data, size_t length, OdinPointCloudPacketFull* packet);
  bool ParseImagePacket(const uint8_t* data, size_t length, OdinImagePacketFull* packet);
  bool ParseOdomPacket(const uint8_t* data, size_t length, OdinOdomPacketFull* packet);
  bool ParseTfPacket(const uint8_t* data, size_t length, OdinTfPacketFull* packet);

  // Frame assembly (v2: driven by chunk_index + chunk_count + flags.last_chunk)
  void ProcessPointFragment(const OdinPointCloudPacketFull& fragment, bool is_slam,
                            uint16_t chunk_index, bool last_chunk);
  void EmitPointFrame(bool is_slam);
  // Fill `lost_chunks` worth of zero points into the in-progress point frame
  // when a chunk_index gap is detected, preserving spatial layout downstream.
  void PadZerosAtGapPosition(uint16_t lost_chunks, bool is_slam);
  // Pad the converted float point vector with zero-valued points so the
  // emitted frame always carries the expected point count even when tail
  // chunks were lost. Templated on the float point type.
  template <typename FloatPoint>
  void PadZerosForTailLoss(std::vector<FloatPoint>& points,
                                 PointFrameState& state,
                                 size_t src_point_size, bool is_slam);
  void ProcessImageFragment(const OdinImagePacketFull& fragment,
                            uint16_t chunk_index, bool last_chunk);
  void EmitImageFrame();
  void ProcessH264Fragment(const OdinCameraPacket& fragment,
                           uint16_t chunk_index, bool last_chunk);
  void EmitH264Frame();

  // TCP buffer processing
  void ProcessTcpBuffer(std::function<void(const uint8_t*, size_t)> handler);

  OdinDeviceHandle handle_;
  OdinTransportMode transport_mode_;
  // Payload format for the image (non-H264) assembly path; see SetImageFormat.
  OdinDataFormat image_format_ = OdinDataFormat::kImageMjpeg;

  // Whether the underlying transport is stream-oriented (requires TCP-style
  // buffered reassembly) versus datagram-oriented (one packet == one frame
  // chunk). Centralised here so future transports only need updating in one
  // place.
  bool IsStreamTransport() const {
    return transport_mode_ == OdinTransportMode::kTcp;
  }

  // Frame states
  PointFrameState raw_point_state_;
  PointFrameState slam_point_state_;
  ImageFrameState image_state_;
  H264FrameState h264_state_;
  std::mutex frame_mutex_;

  // TCP buffer
  TcpStreamBuffer tcp_buffer_;
  std::mutex tcp_buffer_mutex_;

  // Callbacks
  PointCloudCallback point_cloud_cb_;
  SlamCallback slam_cb_;
  ImageCallback image_cb_;
  H264Callback h264_cb_;
  ImuCallback imu_cb_;
  OdomCallback odom_cb_;
  TfCallback tf_cb_;
};

}  // namespace sdk
}  // namespace odin
