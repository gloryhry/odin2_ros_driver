#pragma once

/**
 * @file OdinProtocol.hpp
 * @brief Odin on-wire protocol frame headers and constants.
 * @details Defines the data frame header (protocol v2, 32B naturally aligned) and the
 *          command frame header, plus the OdinConst size/offset constants derived via
 *          sizeof()/offsetof(). The data wire is v2 (breaking change vs the legacy 27B
 *          header); the command wire is unchanged.
 */

#include <cstddef>  // for offsetof

#include "../IProtocol/IProtocol.hpp"
#include "FastCRC/FastCRC.h"

namespace odin {
namespace sdk {

/* ============================================================================
 * Odin Data Frame Header (wire protocol v2)
 * ============================================================================ */

/**
 * @brief On-wire data frame header (protocol v2, 32 bytes, naturally aligned).
 * @details Prefixes every data chunk (point cloud / image / IMU / odom) on UDP/TCP.
 *          v2 supersedes the legacy 27B header: first byte `sof` is fixed to 0xAD,
 *          second byte `version` is fixed to 0x02. `payload_len` is the pure payload
 *          size (excludes this header); reassembly is driven by `chunk_index` +
 *          `chunk_count` (+ `flags` bit0 = last_chunk); per-sample IMU timestamps are
 *          reconstructed as `timestamp + j*interval_ns`; the data-plane `crc32` defaults
 *          to 0 (structural fields are used for validation). Layout is verified by
 *          static_asserts below.
 */
struct OdinDataFrameHeader {
  uint8_t sof = 0;             ///< Start of frame marker, fixed 0xAD (OdinConst::kDataSof)
  uint8_t version = 0;         ///< Protocol version (v2 = 2) init by OdinConst::kDataVersion
  uint8_t data_type = 0;       ///< Channel type (0x00 raw/0x01 slam/0x10-0x11 image/0x20 imu/0x30 odom/0x40 algo/0x41 calib/0x42 telemetry)
  uint8_t flags = 0;           ///< Bit0 = last_chunk; remaining bits reserved
  uint32_t frame_seq = 0;      ///< Frame sequence number
  uint16_t chunk_index = 0;    ///< Zero-based chunk index within the frame
  uint16_t chunk_count = 0;    ///< Total chunk count of the frame
  uint16_t element_count = 0;  ///< Element count in this chunk (points/samples/bytes)
  uint16_t payload_len = 0;    ///< Pure payload byte count of this chunk (excludes header)
  uint64_t timestamp = 0;      ///< Timestamp (ns) of the first element in this chunk
  uint32_t interval_ns = 0;    ///< IMU only: inter-sample interval (ns); 0 for other channels
  uint32_t crc32 = 0;          ///< Payload CRC32 (data plane defaults to 0)
};
static_assert(sizeof(OdinDataFrameHeader) == 32, "OdinDataFrameHeader (v2) must be 32 bytes");
static_assert(offsetof(OdinDataFrameHeader, frame_seq) == 4, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, chunk_index) == 8, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, chunk_count) == 10, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, element_count) == 12, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, payload_len) == 14, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, timestamp) == 16, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, interval_ns) == 24, "v2 header layout");
static_assert(offsetof(OdinDataFrameHeader, crc32) == 28, "v2 header layout");

/* ============================================================================
 * Odin Command Frame Header
 * ============================================================================
 * Frame format (18-byte header + payload):
 *   [0]      SOF (0xAE)
 *   [1]      Version
 *   [2-3]    Length (payload size)
 *   [4-7]    Sequence (uint32_t)
 *   [8-9]    Command ID
 *   [10]     Command Type (req/ack)
 *   [11]     Send Type (host/device)
 *   [12-13]  CRC16 (header checksum, covers [0-11])
 *   [14-17]  CRC32 (payload checksum)
 *   [18...]  Payload
 */
#pragma pack(push, 1)
struct OdinCommandFrameHeader {
  uint8_t sof;
  uint8_t version;
  uint16_t length;      // payload size
  uint32_t seq;
  uint16_t cmd_id;
  uint8_t cmd_type;
  uint8_t send_type;
  uint16_t crc16;       // header checksum (covers sof to send_type)
  uint32_t crc32;       // payload checksum
};
#pragma pack(pop)

/* ============================================================================
 * Odin Protocol Constants
 * ============================================================================
 * Uses sizeof() and offsetof() for auto-calculation, constants auto-update when struct changes
 */
namespace OdinConst {
constexpr uint8_t kSof = 0xAE;
constexpr uint8_t kVersion = 0x02;      // control protocol version (subscribe/port decoupled)
constexpr uint8_t kDataSof = 0xAD;     // data frame start-of-frame marker
constexpr uint8_t kDataVersion = 0x02;  // data wire protocol version (v2)
constexpr uint8_t kCmdTypeReq = 0x00;
constexpr uint8_t kCmdTypeAck = 0x01;
constexpr uint8_t kSendTypeHost = 0x00;
constexpr uint8_t kSendTypeDevice = 0x01;

// Auto-calculated size constants (auto-update when struct changes)
constexpr size_t kHeaderSize = sizeof(OdinCommandFrameHeader);
constexpr size_t kDataHeaderSize = sizeof(OdinDataFrameHeader);

// CRC16 checksum range: from sof to send_type (excludes crc16 and crc32)
constexpr size_t kCrc16EndOffset = offsetof(OdinCommandFrameHeader, crc16);
}  // namespace OdinConst

// Compile-time struct size validation
static_assert(sizeof(OdinDataFrameHeader) == 32, "Unexpected data header size (v2)");
static_assert(sizeof(OdinCommandFrameHeader) == 18, "Unexpected command header size");
static_assert(OdinConst::kCrc16EndOffset == 12, "Unexpected CRC16 offset");

/* ============================================================================
 * OdinPacket - Concrete IPacket Implementation
 * ============================================================================
 *
 * Compatible with existing OdinCommandPacket structure:
 *   - version, length, seq, cmd_id, cmd_type, send_type, crc16, crc32, payload
 */

class OdinPacket : public IPacket {
 public:
  OdinPacket() = default;

  // Construct from existing data (convenience for migration from legacy code)
  OdinPacket(uint16_t cmd_id, uint32_t seq, uint8_t cmd_type, uint8_t send_type,
             const uint8_t* payload = nullptr, size_t payload_size = 0)
      : seq_(seq), cmd_id_(cmd_id), cmd_type_(cmd_type), send_type_(send_type) {
    if (payload && payload_size > 0) {
      payload_.assign(payload, payload + payload_size);
    }
  }

  // IPacket interface implementation
  uint32_t GetCommandId() const override { return cmd_id_; }
  uint32_t GetSequence() const override { return seq_; }
  const uint8_t* GetPayload() const override {
    return payload_.empty() ? nullptr : payload_.data();
  }
  size_t GetPayloadSize() const override { return payload_.size(); }

  void SetCommandId(uint32_t id) override { cmd_id_ = static_cast<uint16_t>(id); }
  void SetSequence(uint32_t seq) override { seq_ = seq; }
  void SetPayload(const uint8_t* data, size_t size) override {
    if (data && size > 0) {
      payload_.assign(data, data + size);
    } else {
      payload_.clear();
    }
  }

  uint32_t GetAttribute(uint32_t key) const override {
    switch (key) {
      case PacketAttr::kCmdType:
        return cmd_type_;
      case PacketAttr::kSendType:
        return send_type_;
      case PacketAttr::kVersion:
        return version_;
      case PacketAttr::kCrc16:
        return crc16_;
      case PacketAttr::kCrc32:
        return crc32_;
      default:
        return 0;
    }
  }

  void SetAttribute(uint32_t key, uint32_t value) override {
    switch (key) {
      case PacketAttr::kCmdType:
        cmd_type_ = static_cast<uint8_t>(value);
        break;
      case PacketAttr::kSendType:
        send_type_ = static_cast<uint8_t>(value);
        break;
      case PacketAttr::kVersion:
        version_ = static_cast<uint8_t>(value);
        break;
      case PacketAttr::kCrc16:
        crc16_ = static_cast<uint16_t>(value);
        break;
      case PacketAttr::kCrc32:
        crc32_ = static_cast<uint32_t>(value);
        break;
    }
  }

  std::unique_ptr<IPacket> Clone() const override {
    std::unique_ptr<OdinPacket> pkt(new OdinPacket());
    pkt->version_ = version_;
    pkt->seq_ = seq_;
    pkt->cmd_id_ = cmd_id_;
    pkt->cmd_type_ = cmd_type_;
    pkt->send_type_ = send_type_;
    pkt->crc16_ = crc16_;
    pkt->crc32_ = crc32_;
    pkt->payload_ = payload_;
    return pkt;
  }

  // Odin-specific accessors (convenient direct access)
  uint8_t GetCmdType() const { return cmd_type_; }
  uint8_t GetSendType() const { return send_type_; }
  uint16_t GetCmdId() const { return cmd_id_; }
  uint32_t GetSeq() const { return seq_; }
  void SetCmdType(uint8_t v) { cmd_type_ = v; }
  void SetSendType(uint8_t v) { send_type_ = v; }

 private:
  uint8_t version_ = OdinConst::kVersion;
  uint32_t seq_ = 0;
  uint16_t cmd_id_ = 0;
  uint8_t cmd_type_ = OdinConst::kCmdTypeReq;
  uint8_t send_type_ = OdinConst::kSendTypeHost;
  uint16_t crc16_ = 0;
  uint32_t crc32_ = 0;
  std::vector<uint8_t> payload_;
};

/* ============================================================================
 * OdinProtocolV1 - Concrete IProtocol Implementation
 * ============================================================================
 *
 * Frame format (18-byte header + payload):
 *   [0]     SOF (0xAE)
 *   [1]     Version
 *   [2-3]   Length (payload size)
 *   [4-7]   Sequence (uint32_t)
 *   [8-9]   Command ID
 *   [10]    Command Type (req/ack)
 *   [11]    Send Type (host/device)
 *   [12-13] CRC16 (header checksum)
 *   [14-17] CRC32 (payload checksum)
 *   [18...] Payload
 */

class OdinProtocolV1 : public IProtocol {
 public:
  bool Pack(const IPacket& packet, std::vector<uint8_t>& output) override;

  int Parse(const uint8_t* buffer, size_t length, std::unique_ptr<IPacket>& packet) override;

  std::unique_ptr<IPacket> CreatePacket() override {
    return std::unique_ptr<IPacket>(new OdinPacket());
  }

  size_t GetHeaderSize() const override { return OdinConst::kHeaderSize; }
  const char* GetProtocolName() const override { return "OdinV1"; }
  uint32_t GetProtocolVersion() const override { return 1; }

  // Legacy API compatibility (kept for migration convenience)
  bool Pack(uint16_t cmd_id, uint32_t seq, uint8_t cmd_type, uint8_t send_type,
            const uint8_t* payload, size_t payload_size, std::vector<uint8_t>& output) {
    OdinPacket pkt(cmd_id, seq, cmd_type, send_type, payload, payload_size);
    return Pack(pkt, output);
  }

 private:
  mutable FastCRC16 crc16_;
  mutable FastCRC32 crc32_;
};

}  // namespace sdk
}  // namespace odin
