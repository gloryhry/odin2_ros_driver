#ifndef ODIN_LIDAR_INNER_H_
#define ODIN_LIDAR_INNER_H_

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// ============================================================================
// Wire-format types (internal use only, not exposed to SDK users)
// ============================================================================

#pragma pack(push, 1)

/// @brief Wire-format TF transform payload (data_type=0x32, 60 bytes per frame).
/// @details Aligned with ROS2 geometry_msgs/TransformStamped semantics:
///          frame_id (parent) and child_frame_id are fixed-length char[16]
///          strings (NUL-terminated when shorter than 16; not guaranteed NUL
///          terminated when occupying all 16 bytes).
struct OdinTfData {
  float translation[3] = {0.0f, 0.0f, 0.0f};         ///< Translation (m)
  float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f};      ///< Quaternion (x, y, z, w)
  char frame_id[16] = {0};                           ///< Parent frame id (string, up to 16 chars)
  char child_frame_id[16] = {0};                     ///< Child frame id (string, up to 16 chars)
};
static_assert(sizeof(OdinTfData) == 60, "Unexpected OdinTfData size");

#pragma pack(pop)

// ============================================================================
// Internal Structures (not exposed to SDK users)
// ============================================================================

/**
 * @brief Command response structure
 */
struct OdinCommandResponse {
  uint16_t seq = 0;                                ///< Sequence number
  uint16_t cmd_id = 0;                             ///< Command ID
  std::vector<uint8_t> payload;                    ///< Response payload
};

/**
 * @brief Synchronous command response structure
 */
struct OdinCommandSyncResponse {
  OdinResult result;             ///< Operation result
  OdinCommandResponse response;  ///< Command response
};

/** @brief Command response callback type */
using OdinCommandCallback = void (*)(OdinResult result, const OdinCommandResponse *response,
                                     void *client_data);

/**
 * @brief Full-member image frame used internally by the frame assembler.
 *
 * The assembler needs protocol/assembly metadata (version, length, chunk_count,
 * frame_count) to drive chunk reassembly and new-frame detection.
 *
 * On emission, timestamp/frame_count/payload are forwarded to the public
 * OdinCameraPacket (format=kImageMjpeg, is_keyframe=true). width/height are left 0
 * by the assembler and filled by the StartStream lambda from OdinStreamCfg.
 * The protocol fields version/length/chunk_count are not exposed.
 */
struct OdinImagePacketFull {
  uint8_t version = 0;           ///< Protocol version (=2)
  uint16_t length = 0;           ///< Assembled frame size (header + payload)
  uint16_t chunk_count = 0;      ///< Number of chunks assembled (v2 chunk_count)
  uint32_t frame_count = 0;      ///< Frame sequence number (v2 frame_seq)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<uint8_t> payload;  ///< Raw JPEG bytes (with SOI/EOI markers)
};

/**
 * @brief Full-member point cloud frame used internally by the frame assembler.
 *
 * Contains protocol/assembly metadata (version, length, chunk_count, data_type)
 * needed during chunk reassembly. On emission, only point_count/frame_count/
 * timestamp/payload are forwarded to the public OdinPointCloudPacket.
 */
struct OdinPointCloudPacketFull {
  uint8_t version = 0;           ///< Protocol version (=2)
  uint16_t length = 0;           ///< Assembled frame size (header + payload)
  uint16_t point_count = 0;      ///< Total points in the frame
  uint16_t chunk_count = 0;      ///< Number of chunks assembled (v2 chunk_count)
  uint32_t frame_count = 0;      ///< Frame sequence number (v2 frame_seq)
  uint8_t data_type = 0;         ///< Channel type (0x00 raw / 0x01 slam)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<uint8_t> payload;  ///< Point payload (raw uint16 or converted float)
};

/**
 * @brief Full-member IMU packet used internally by the frame assembler.
 *
 * Protocol fields version/length are not exposed to the user.
 */
struct OdinImuPacketFull {
  uint8_t version = 0;           ///< Protocol version (=2)
  uint16_t length = 0;           ///< Frame size (header + payload)
  uint16_t sample_count = 0;     ///< Number of IMU samples in this packet
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<uint8_t> payload;  ///< Raw IMU sample bytes (array of OdinImuSample)
};

/**
 * @brief Full-member odometry packet used internally by the frame assembler.
 *
 * Protocol fields version/length are not exposed to the user.
 */
struct OdinOdomPacketFull {
  uint8_t version = 0;           ///< Protocol version (=2)
  uint16_t length = 0;           ///< Frame size (header + payload)
  uint16_t data_count = 0;       ///< Number of data entries
  uint32_t frame_count = 0;      ///< Frame sequence number (v2 frame_seq)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<uint8_t> payload;  ///< Raw odom bytes (OdinOdomData)
};

/**
 * @brief Full-member TF packet used internally by the frame assembler.
 *
 * Protocol fields version/length/payload(raw wire bytes) are not exposed.
 * The parsed translation/rotation/frame_id/child_frame_id are forwarded.
 */
struct OdinTfPacketFull {
  uint8_t version = 0;           ///< Protocol version (=2)
  uint16_t length = 0;           ///< Frame size (header + payload)
  uint32_t frame_count = 0;      ///< Frame sequence number (v2 frame_seq)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  float translation[3] = {0.0f, 0.0f, 0.0f}; ///< Parsed translation (m)
  float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f}; ///< Parsed quaternion (x, y, z, w)
  std::string frame_id;          ///< Parent frame id (parsed string)
  std::string child_frame_id;    ///< Child frame id (parsed string)
  std::vector<uint8_t> payload;  ///< Raw 60-byte wire payload
};

}  // namespace sdk
}  // namespace odin

#endif  // ODIN_LIDAR_INNER_H_