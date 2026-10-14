#include "OdinFrameAssembler.hpp"

#include "logger.h"

namespace odin {
namespace sdk {

// Constants for Odin2 data assembly (wire protocol v2)
constexpr size_t kDataHeaderSize = OdinConst::kDataHeaderSize;
constexpr size_t kUdpMaxDataSize = 1472 - kDataHeaderSize;  // payload bytes per UDP chunk (reserve hint)
constexpr uint8_t kFlagLastChunk = 0x01;                    // header.flags bit0 = last_chunk

// One-shot hint: print sysctl suggestions when packet loss is first detected.
// Uses a process-wide static flag so the message is emitted at most once.
static void LogNetBufferHintOnce() {
  static bool shown = false;
  if (shown) return;
  shown = true;
  LOG_WARN("Packet loss detected — this is often caused by insufficient kernel "
           "UDP receive buffer. Consider increasing it (requires sudo):\n"
           "  sudo sysctl -w net.core.rmem_max=16777216     # 16MB\n"
           "  sudo sysctl -w net.core.rmem_default=8388608  #  8MB\n"
           "  sudo sysctl -w net.core.netdev_max_backlog=50000\n"
           "Persist by adding to /etc/sysctl.conf and running 'sudo sysctl -p'.\n");
}

OdinFrameAssembler::OdinFrameAssembler(OdinDeviceHandle handle, OdinTransportMode transport)
    : handle_(handle), transport_mode_(transport) {}

void OdinFrameAssembler::Reset() {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  raw_point_state_.Reset();
  slam_point_state_.Reset();
  image_state_.Reset();
  
  std::lock_guard<std::mutex> tcp_lock(tcp_buffer_mutex_);
  tcp_buffer_.Clear();
}

// =============================================================================
// Parsing helpers
// =============================================================================

bool OdinFrameAssembler::ParsePointCloudPacket(const uint8_t* data, size_t length,
                                                OdinPointCloudPacketFull* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  size_t payload_size = header->payload_len;
  if (kDataHeaderSize + payload_size > length) return false;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = static_cast<uint16_t>(kDataHeaderSize + payload_size);
  packet->point_count = header->element_count;  // points in THIS chunk
  packet->chunk_count = header->chunk_count;
  packet->frame_count = header->frame_seq;
  packet->data_type = header->data_type;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseImagePacket(const uint8_t* data, size_t length,
                                           OdinImagePacketFull* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  size_t payload_size = header->payload_len;
  if (kDataHeaderSize + payload_size > length) return false;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = static_cast<uint16_t>(kDataHeaderSize + payload_size);
  packet->chunk_count = header->chunk_count;
  packet->frame_count = header->frame_seq;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseOdomPacket(const uint8_t* data, size_t length,
                                          OdinOdomPacketFull* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  size_t payload_size = header->payload_len;
  if (kDataHeaderSize + payload_size > length) return false;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = static_cast<uint16_t>(kDataHeaderSize + payload_size);
  packet->data_count = header->element_count;
  packet->frame_count = header->frame_seq;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);
  return true;
}

bool OdinFrameAssembler::ParseTfPacket(const uint8_t* data, size_t length,
                                        OdinTfPacketFull* packet) {
  if (!data || !packet || length < kDataHeaderSize) return false;

  const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(data);
  size_t payload_size = header->payload_len;
  if (kDataHeaderSize + payload_size > length) return false;
  if (payload_size < sizeof(OdinTfData)) return false;
  const uint8_t* payload_ptr = data + kDataHeaderSize;

  packet->version = header->version;
  packet->length = static_cast<uint16_t>(kDataHeaderSize + payload_size);
  packet->frame_count = header->frame_seq;
  packet->timestamp = header->timestamp;
  packet->payload.assign(payload_ptr, payload_ptr + payload_size);

  const OdinTfData* tf = reinterpret_cast<const OdinTfData*>(payload_ptr);
  std::memcpy(packet->translation, tf->translation, sizeof(tf->translation));
  std::memcpy(packet->rotation, tf->rotation, sizeof(tf->rotation));
  // char[16] frame ids may or may not be NUL-terminated; bound by strnlen
  // to safely construct std::string for direct ROS message assignment.
  packet->frame_id.assign(
      tf->frame_id,
      ::strnlen(tf->frame_id, sizeof(tf->frame_id)));
  packet->child_frame_id.assign(
      tf->child_frame_id,
      ::strnlen(tf->child_frame_id, sizeof(tf->child_frame_id)));
  return true;
}

// =============================================================================
// TCP buffer processing
// =============================================================================

void OdinFrameAssembler::ProcessTcpBuffer(
    std::function<void(const uint8_t*, size_t)> handler) {
  std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);

  while (tcp_buffer_.Size() >= kDataHeaderSize) {
    const uint8_t* buf = tcp_buffer_.Data();

    // v2: total frame length = header + pure payload_len
    const OdinDataFrameHeader* header = reinterpret_cast<const OdinDataFrameHeader*>(buf);

    // Validate v2 frame invariants: sof == 0xAD, version == 2
    if (header->sof != OdinConst::kDataSof || header->version != OdinConst::kDataVersion) {
      // Bad header: discard entire buffer and resync
      tcp_buffer_.Clear();
      break;
    }

    size_t frame_len = kDataHeaderSize + header->payload_len;

    if (tcp_buffer_.Size() < frame_len) {
      // Incomplete frame, wait for more data
      break;
    }

    // Complete frame, process it
    handler(buf, frame_len);
    tcp_buffer_.Consume(frame_len);
  }
}

// =============================================================================
// Point cloud processing
// =============================================================================

void OdinFrameAssembler::ProcessPointFragment(const OdinPointCloudPacketFull& fragment,
                                               bool is_slam, uint16_t chunk_index,
                                               bool last_chunk) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  // v2: a new frame starts when frame_seq changes or chunk_index resets to 0.
  bool start_new_frame =
      state.active && (fragment.frame_count != state.packet.frame_count || chunk_index == 0);

  if (start_new_frame) {
    EmitPointFrame(is_slam);
  }

  if (!state.active) {
    state.active = true;
    // Copy header metadata from the first fragment, but reset accumulators
    // (point_count is re-accumulated below from each chunk's element_count).
    state.packet = fragment;
    state.packet.payload.clear();
    state.packet.payload.reserve(static_cast<size_t>(fragment.chunk_count) * kUdpMaxDataSize);
    state.packet.point_count = 0;
    state.expected_chunks = fragment.chunk_count;
    state.received_chunks = 0;
    state.last_chunk_index = 0;
  }

  // Chunk gap detection: when chunk_index jumps, fill the missing chunks with
  // zero points so the assembled frame keeps a stable spatial layout downstream
  // (preserves the legacy "fill lost packets with zeros" behavior under v2).
  if (state.received_chunks > 0 && chunk_index > state.last_chunk_index + 1) {
    uint16_t lost = static_cast<uint16_t>(chunk_index - state.last_chunk_index - 1);
    LOG_WARN("Point cloud chunk gap: expected %u got %u, filling %u chunk(s) with zeros\n",
             static_cast<unsigned>(state.last_chunk_index + 1),
             static_cast<unsigned>(chunk_index),
             static_cast<unsigned>(lost));
    LogNetBufferHintOnce();
    PadZerosAtGapPosition(lost, is_slam);
  }

  // Append payload
  state.packet.payload.insert(state.packet.payload.end(),
                               fragment.payload.begin(), fragment.payload.end());
  state.packet.point_count += fragment.point_count;  // element_count of this chunk
  state.last_chunk_index = chunk_index;
  state.received_chunks++;
  if (last_chunk) {
    state.last_chunk_received = true;
  }

  // Frame complete when last_chunk flag is set or all expected chunks received.
  bool complete = last_chunk ||
      (state.expected_chunks > 0 && state.received_chunks >= state.expected_chunks);
  if (complete) {
    EmitPointFrame(is_slam);
  }
}

void OdinFrameAssembler::EmitPointFrame(bool is_slam) {
  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  if (!state.active || state.packet.payload.empty()) {
    state.Reset();
    return;
  }

  state.packet.chunk_count = state.received_chunks;
  size_t total_length = kDataHeaderSize + state.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  state.packet.length = static_cast<uint16_t>(total_length);

  if (is_slam && slam_cb_) {
    // Convert uint16 to float for SLAM (with -30000 offset for y,z)
    // Coordinate transformation (rotation/translation) will be done by SlamOdomSynchronizer
    size_t src_point_size = sizeof(OdinSlamPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinSlamPoint<uint16_t>* src =
        reinterpret_cast<const OdinSlamPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<OdinSlamPoint<float>> points(point_count);

    for (size_t i = 0; i < point_count; ++i) {
      // Convert to float with offset (in millimeters)
      points[i].x = static_cast<float>(src[i].x);
      points[i].y = static_cast<float>(src[i].y) - 30000.0f;
      points[i].z = static_cast<float>(src[i].z) - 30000.0f;
      points[i].r = src[i].r;
      points[i].g = src[i].g;
      points[i].b = src[i].b;
      points[i].a = src[i].a;
    }
    PadZerosForTailLoss(points, state, src_point_size, /*is_slam=*/true);
    OdinSlamPacket out;
    out.frame_count = state.packet.frame_count;
    out.timestamp = state.packet.timestamp;
    out.points = std::move(points);
    slam_cb_(out);
  } else if (!is_slam && point_cloud_cb_) {
    // Convert uint16 to float for raw point cloud
    size_t src_point_size = sizeof(OdinRawPoint<uint16_t>);
    size_t point_count = state.packet.payload.size() / src_point_size;
    const OdinRawPoint<uint16_t>* src =
        reinterpret_cast<const OdinRawPoint<uint16_t>*>(state.packet.payload.data());

    std::vector<OdinRawPoint<float>> points(point_count);

    for (size_t i = 0; i < point_count; ++i) {
      points[i].x = static_cast<float>(src[i].x);
      points[i].y = static_cast<float>(src[i].y) - 30000.0f;
      points[i].z = static_cast<float>(src[i].z) - 30000.0f;
      points[i].intensity = src[i].intensity;
      points[i].confidence = src[i].confidence;
    }
    PadZerosForTailLoss(points, state, src_point_size, /*is_slam=*/false);
    OdinPointCloudPacket out;
    out.frame_count = state.packet.frame_count;
    out.timestamp = state.packet.timestamp;
    out.points = std::move(points);
    point_cloud_cb_(out);
  }

  state.Reset();
}

template <typename FloatPoint>
void OdinFrameAssembler::PadZerosForTailLoss(std::vector<FloatPoint>& points,
                                                    PointFrameState& state,
                                                    size_t src_point_size,
                                                    bool is_slam) {
  if (src_point_size == 0) return;

  // If the frame was explicitly closed by the device's last_chunk flag, the
  // trailing point count is authoritative: the last chunk may simply be
  // partial. Do not synthesize zero points in that case.
  if (state.last_chunk_received) return;

  // Estimate the expected number of points per frame from the chunk count
  // advertised in the header. Each UDP chunk carries at most
  // kUdpMaxDataSize / src_point_size points, so this is an upper-bound
  // estimate — if the frame already has at least that many points, no
  // padding is needed.
  if (state.expected_chunks == 0) return;
  const size_t pts_per_chunk = kUdpMaxDataSize / src_point_size;
  if (pts_per_chunk == 0) return;
  const size_t expected_points =
      static_cast<size_t>(state.expected_chunks) * pts_per_chunk;

  if (points.size() >= expected_points) return;

  const size_t missing_points = expected_points - points.size();
  LOG_WARN("%s point frame_seq=%u incomplete: %zu/%zu points, padding %zu zero point(s)\n",
           is_slam ? "SLAM" : "Raw",
           static_cast<unsigned>(state.packet.frame_count),
           points.size(), expected_points, missing_points);
  LogNetBufferHintOnce();

  // Default-constructed FloatPoint has all-zero fields (x/y/z = 0, etc.).
  points.resize(expected_points);
}

// Explicit instantiations for the two float point types used by EmitPointFrame.
template void OdinFrameAssembler::PadZerosForTailLoss<OdinSlamPoint<float>>(
    std::vector<OdinSlamPoint<float>>&, PointFrameState&, size_t, bool);
template void OdinFrameAssembler::PadZerosForTailLoss<OdinRawPoint<float>>(
    std::vector<OdinRawPoint<float>>&, PointFrameState&, size_t, bool);

void OdinFrameAssembler::PadZerosAtGapPosition(uint16_t lost_chunks, bool is_slam) {
  if (lost_chunks == 0) return;

  PointFrameState& state = is_slam ? slam_point_state_ : raw_point_state_;

  // Estimate per-chunk point count. v2 chunks are not strictly fixed-size, so
  // use the same heuristic as the legacy v1 zero-fill: assume each lost chunk
  // carries up to (kUdpMaxDataSize / point_size) points (the typical UDP
  // payload). Downstream consumers tolerate slightly under/over filled frames;
  // the goal is to keep the spatial layout aligned, not bit-exact reconstruction.
  const size_t point_size =
      is_slam ? sizeof(OdinSlamPoint<uint16_t>) : sizeof(OdinRawPoint<uint16_t>);
  if (point_size == 0) return;
  const size_t pts_per_chunk = kUdpMaxDataSize / point_size;
  if (pts_per_chunk == 0) return;

  const size_t lost_points = static_cast<size_t>(lost_chunks) * pts_per_chunk;
  const size_t fill_bytes = lost_points * point_size;

  state.packet.payload.insert(state.packet.payload.end(), fill_bytes, 0);

  uint32_t new_total = static_cast<uint32_t>(state.packet.point_count) + lost_points;
  if (new_total > 65535) new_total = 65535;
  state.packet.point_count = static_cast<uint16_t>(new_total);

  // Account for the missing chunks so chunk_count accounting and completion
  // checks remain consistent with expected_chunks.
  state.received_chunks = static_cast<uint16_t>(state.received_chunks + lost_chunks);
  state.last_chunk_index = static_cast<uint16_t>(state.last_chunk_index + lost_chunks);
}

void OdinFrameAssembler::ProcessRawPointData(const uint8_t* data, size_t length) {
  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacketFull packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(frame_data);
        ProcessPointFragment(packet, false, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
      }
    });
  } else {
    OdinPointCloudPacketFull packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(data);
      ProcessPointFragment(packet, false, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
    }
  }
}

void OdinFrameAssembler::ProcessSlamPointData(const uint8_t* data, size_t length) {
  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinPointCloudPacketFull packet;
      if (ParsePointCloudPacket(frame_data, frame_len, &packet)) {
        const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(frame_data);
        ProcessPointFragment(packet, true, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
      }
    });
  } else {
    OdinPointCloudPacketFull packet;
    if (ParsePointCloudPacket(data, length, &packet)) {
      const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(data);
      ProcessPointFragment(packet, true, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
    }
  }
}

// =============================================================================
// Image processing
// =============================================================================

void OdinFrameAssembler::ProcessImageFragment(const OdinImagePacketFull& fragment,
                                               uint16_t chunk_index, bool last_chunk) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  // v2: a new frame starts when frame_seq changes or chunk_index resets to 0.
  bool start_new_frame =
      image_state_.active && (fragment.frame_count != image_state_.packet.frame_count ||
                               chunk_index == 0);

  if (start_new_frame) {
    EmitImageFrame();
  }

  if (!image_state_.active) {
    image_state_.active = true;
    image_state_.packet = fragment;
    image_state_.packet.payload.clear();
    image_state_.expected_chunks = fragment.chunk_count;
    image_state_.received_chunks = 0;
    image_state_.last_chunk_index = 0;
  }

  // Append payload (arrival order)
  image_state_.packet.payload.insert(image_state_.packet.payload.end(),
                                      fragment.payload.begin(), fragment.payload.end());
  image_state_.last_chunk_index = chunk_index;
  image_state_.received_chunks++;

  // v2: frame complete by last_chunk flag or all expected chunks received.
  bool complete = last_chunk ||
      (image_state_.expected_chunks > 0 &&
       image_state_.received_chunks >= image_state_.expected_chunks);
  if (complete) {
    EmitImageFrame();
  }
}

void OdinFrameAssembler::EmitImageFrame() {
  if (!image_state_.active || image_state_.packet.payload.empty()) {
    image_state_.Reset();
    return;
  }

  // v2: payload is the raw encoded byte stream delivered as-is (JPEG carries
  // its own SOI/EOI; NV12 is raw planar data). No trailing 4B length field.
  size_t total_length = kDataHeaderSize + image_state_.packet.payload.size();
  if (total_length > 65535) total_length = 65535;
  image_state_.packet.chunk_count = image_state_.received_chunks;
  image_state_.packet.length = static_cast<uint16_t>(total_length);

  if (image_cb_) {
    // Convert the full-member internal frame to the public OdinCameraPacket.
    // format comes from the StartStream config (kImageMjpeg / kImageNv12);
    // is_keyframe=true (each JPEG/NV12 frame is independently decodable).
    // frame_count is forwarded from the v2 frame_seq for drop detection.
    // width/height are left 0 here; the StartStream lambda fills them from
    // the user-provided OdinStreamCfg (same approach as H264).
    OdinCameraPacket out;
    out.format      = image_format_;
    out.timestamp   = image_state_.packet.timestamp;
    out.frame_count = image_state_.packet.frame_count;
    out.is_keyframe = true;
    out.payload     = std::move(image_state_.packet.payload);
    image_cb_(out);
  }

  image_state_.Reset();
}

void OdinFrameAssembler::ProcessImageData(const uint8_t* data, size_t length) {
  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer([this](const uint8_t* frame_data, size_t frame_len) {
      OdinImagePacketFull packet;
      if (ParseImagePacket(frame_data, frame_len, &packet)) {
        const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(frame_data);
        ProcessImageFragment(packet, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
      }
    });
  } else {
    OdinImagePacketFull packet;
    if (ParseImagePacket(data, length, &packet)) {
      const auto* h = reinterpret_cast<const OdinDataFrameHeader*>(data);
      ProcessImageFragment(packet, h->chunk_index, (h->flags & kFlagLastChunk) != 0);
    }
  }
}

// =============================================================================
// IMU/Odom processing
// =============================================================================

void OdinFrameAssembler::ProcessImuData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    if (frame_len < kDataHeaderSize || !imu_cb_) return;

    const OdinDataFrameHeader* header =
        reinterpret_cast<const OdinDataFrameHeader*>(frame_data);
    const size_t payload_size = header->payload_len;
    if (kDataHeaderSize + payload_size > frame_len) return;

    // v2: payload is a pure array of OdinImuSample (24B: gyro + acc only).
    // Per-sample timestamps are reconstructed as header.timestamp + j*interval_ns;
    // each chunk is self-contained (no cross-chunk reassembly).
    constexpr size_t kSampleSize = sizeof(OdinImuSample);  // 24
    if (payload_size == 0 || payload_size % kSampleSize != 0) return;

    const size_t sample_count = payload_size / kSampleSize;
    const uint8_t* raw_data = frame_data + kDataHeaderSize;
    const uint64_t base_ts = header->timestamp;
    const uint64_t interval_ns = header->interval_ns;

    for (size_t j = 0; j < sample_count; ++j) {
      OdinImuSample sample{};
      std::memcpy(&sample, raw_data + j * kSampleSize, kSampleSize);

      // Emit one single-sample packet per sample; the reconstructed per-sample
      // timestamp is delivered via OdinImuPacket.timestamp.
      OdinImuPacket single_packet;
      single_packet.timestamp = base_ts + static_cast<uint64_t>(j) * interval_ns;
      single_packet.samples.resize(1);
      single_packet.samples[0] = sample;
      imu_cb_(single_packet);
    }
  };

  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

void OdinFrameAssembler::ProcessOdomData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinOdomPacketFull packet;
    if (ParseOdomPacket(frame_data, frame_len, &packet)) {
      
      // Extract odom_type from payload
      OdomSourceType odom_type = OdomSourceType::kOdom2ImuLow;
      if (packet.payload.size() >= sizeof(OdinOdomData)) {
        const OdinOdomData* odom_data =
            reinterpret_cast<const OdinOdomData*>(packet.payload.data());
        odom_type = static_cast<OdomSourceType>(odom_data->type);
      }

      if (odom_cb_) {
        OdinOdomPacket out;
        out.frame_count = packet.frame_count;
        out.timestamp = packet.timestamp;
        // Parse raw bytes into typed OdinOdomData entries
        size_t count = packet.payload.size() / sizeof(OdinOdomData);
        out.data.resize(count);
        if (count > 0) {
          std::memcpy(out.data.data(), packet.payload.data(),
                      count * sizeof(OdinOdomData));
        }
        odom_cb_(out, odom_type);
      }
    }
  };

  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

void OdinFrameAssembler::ProcessTfData(const uint8_t* data, size_t length) {
  auto process_packet = [this](const uint8_t* frame_data, size_t frame_len) {
    OdinTfPacketFull packet;
    if (ParseTfPacket(frame_data, frame_len, &packet)) {
      if (tf_cb_) {
        OdinTfPacket out;
        out.frame_count = packet.frame_count;
        out.timestamp = packet.timestamp;
        std::memcpy(out.translation, packet.translation, sizeof(out.translation));
        std::memcpy(out.rotation, packet.rotation, sizeof(out.rotation));
        out.frame_id = std::move(packet.frame_id);
        out.child_frame_id = std::move(packet.child_frame_id);
        tf_cb_(out);
      }
    }
  };

  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(process_packet);
  } else {
    process_packet(data, length);
  }
}

// =============================================================================
// H264 processing
// =============================================================================

/// @brief Detect whether an Annex-B bitstream contains a keyframe (IDR or SPS NAL).
static bool DetectH264Keyframe(const std::vector<uint8_t>& payload) {
  // Scan for start codes (00 00 00 01 or 00 00 01) and inspect NAL unit type.
  const size_t size = payload.size();
  for (size_t i = 0; i + 4 < size; ++i) {
    bool is_start_code_4 = (payload[i] == 0x00 && payload[i+1] == 0x00 &&
                            payload[i+2] == 0x00 && payload[i+3] == 0x01);
    bool is_start_code_3 = (i + 3 < size &&
                            payload[i] == 0x00 && payload[i+1] == 0x00 &&
                            payload[i+2] == 0x01);
    size_t nal_byte_offset = 0;
    if (is_start_code_4) {
      nal_byte_offset = i + 4;
    } else if (is_start_code_3) {
      nal_byte_offset = i + 3;
    } else {
      continue;
    }
    if (nal_byte_offset >= size) break;
    uint8_t nal_type = payload[nal_byte_offset] & 0x1F;
    // IDR slice (5) or SPS (7) indicates a keyframe boundary
    if (nal_type == 5 || nal_type == 7) {
      return true;
    }
    // Skip ahead past the start code to avoid re-matching
    i = nal_byte_offset;
  }
  return false;
}

void OdinFrameAssembler::ProcessH264Fragment(const OdinCameraPacket& fragment,
                                              uint16_t chunk_index, bool last_chunk) {
  std::lock_guard<std::mutex> lock(frame_mutex_);

  // A new frame starts when frame_seq changes or chunk_index resets to 0.
  bool start_new_frame =
      h264_state_.active && (fragment.frame_count != h264_state_.packet.frame_count ||
                              chunk_index == 0);

  if (start_new_frame) {
    // The previous frame is being flushed by the NEXT frame's fragment instead
    // of its own last_chunk flag / chunk_count — its tail chunks were lost
    // (silent otherwise when the device leaves chunk_count = 0).
    LOG_WARN("H264 frame_seq=%u tail lost: flushed by next frame, %u chunk(s) received (%zuB)\n",
             static_cast<unsigned>(h264_state_.packet.frame_count),
             static_cast<unsigned>(h264_state_.received_chunks),
             h264_state_.packet.payload.size());
    LogNetBufferHintOnce();
    EmitH264Frame();
  }

  if (!h264_state_.active) {
    h264_state_.active = true;
    h264_state_.packet = fragment;
    h264_state_.packet.payload.clear();
    h264_state_.expected_chunks = 0;  // Will be set from header
    h264_state_.received_chunks = 0;
    h264_state_.last_chunk_index = 0;
    // Head loss detection: the first fragment of a new frame must be chunk 0.
    if (chunk_index != 0) {
      LOG_WARN("H264 frame_seq=%u head lost: first received chunk is %u (chunks 0..%u missing)\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(chunk_index),
               static_cast<unsigned>(chunk_index - 1));
      LogNetBufferHintOnce();
    }
  }

  // Chunk continuity detection: unlike point cloud there is no zero-fill for
  // video — the damaged frame is still delivered (decoder conceals), but log
  // both forward gaps (chunk loss) and backward jumps (UDP reordering).
  if (h264_state_.received_chunks > 0) {
    if (chunk_index > h264_state_.last_chunk_index + 1) {
      uint16_t lost = static_cast<uint16_t>(chunk_index - h264_state_.last_chunk_index - 1);
      LOG_WARN("H264 chunk gap in frame_seq=%u: expected chunk %u got %u, %u chunk(s) lost\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(h264_state_.last_chunk_index + 1),
               static_cast<unsigned>(chunk_index), static_cast<unsigned>(lost));
      LogNetBufferHintOnce();
    } else if (chunk_index <= h264_state_.last_chunk_index) {
      LOG_WARN("H264 chunk reorder in frame_seq=%u: got chunk %u after %u "
               "(payload assembled out of order)\n",
               static_cast<unsigned>(fragment.frame_count),
               static_cast<unsigned>(chunk_index),
               static_cast<unsigned>(h264_state_.last_chunk_index));
    }
  }

  // Append payload (arrival order)
  h264_state_.packet.payload.insert(h264_state_.packet.payload.end(),
                                     fragment.payload.begin(), fragment.payload.end());
  h264_state_.last_chunk_index = chunk_index;
  h264_state_.received_chunks++;

  // Frame complete by last_chunk flag or all expected chunks received.
  bool complete = last_chunk ||
      (h264_state_.expected_chunks > 0 &&
       h264_state_.received_chunks >= h264_state_.expected_chunks);
  if (complete) {
    EmitH264Frame();
  }
}

void OdinFrameAssembler::EmitH264Frame() {
  if (!h264_state_.active || h264_state_.packet.payload.empty()) {
    h264_state_.Reset();
    return;
  }

  // Report truncated frames (missing chunks) before delivery.
  if (h264_state_.expected_chunks > 0 &&
      h264_state_.received_chunks < h264_state_.expected_chunks) {
    LOG_WARN("H264 frame_seq=%u truncated: %u/%u chunks received (%zuB)\n",
             static_cast<unsigned>(h264_state_.packet.frame_count),
             static_cast<unsigned>(h264_state_.received_chunks),
             static_cast<unsigned>(h264_state_.expected_chunks),
             h264_state_.packet.payload.size());
  }

  h264_state_.packet.is_keyframe = DetectH264Keyframe(h264_state_.packet.payload);

  if (h264_cb_) {
    h264_cb_(h264_state_.packet);
  }

  h264_state_.Reset();
}

void OdinFrameAssembler::ProcessH264Data(const uint8_t* data, size_t length) {
  auto parse_and_process = [this](const uint8_t* frame_data, size_t frame_len) {
    if (frame_len < kDataHeaderSize) return;

    const OdinDataFrameHeader* header =
        reinterpret_cast<const OdinDataFrameHeader*>(frame_data);
    size_t payload_size = header->payload_len;
    if (kDataHeaderSize + payload_size > frame_len) return;
    const uint8_t* payload_ptr = frame_data + kDataHeaderSize;

    OdinCameraPacket fragment;
    fragment.format      = OdinDataFormat::kImageH264;
    fragment.frame_count = header->frame_seq;
    fragment.timestamp   = header->timestamp;
    fragment.payload.assign(payload_ptr, payload_ptr + payload_size);

    // Store chunk_count for expected_chunks tracking
    if (!h264_state_.active || fragment.frame_count != h264_state_.packet.frame_count) {
      // New frame
    }
    h264_state_.expected_chunks = header->chunk_count;

    ProcessH264Fragment(fragment, header->chunk_index,
                        (header->flags & kFlagLastChunk) != 0);
  };

  if (IsStreamTransport()) {
    {
      std::lock_guard<std::mutex> lock(tcp_buffer_mutex_);
      tcp_buffer_.Append(data, length);
    }
    ProcessTcpBuffer(parse_and_process);
  } else {
    parse_and_process(data, length);
  }
}

}  // namespace sdk
}  // namespace odin
