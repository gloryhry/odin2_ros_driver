#pragma once

/// @file payload_builder.h
/// @brief Generic payload serialization/deserialization for Odin wire protocol.
///
/// PayloadBuilder handles both flat (non-TLV) payloads and TLV-structured
/// payloads through a unified buffer. Callers chain WriteU8/AddTlv etc. to
/// describe the wire layout declaratively.
///
/// TlvParser provides the symmetric read path for TLV-encoded ACK payloads.
///
/// Design principles:
///   - Does not know about any specific protocol command (ChannelConfig, etc.)
///   - **All multi-byte TLV values are BIG-ENDIAN** (per protocol spec).
///     There is no little-endian TLV path; use WriteRaw/WriteU16LE for
///     non-TLV header bytes that require little-endian encoding.
///   - AddTlv<T> accepts any integral T; sizeof(T) determines wire length
///     automatically — no separate AddTlvBE / AddTlvU32BE variants needed.
///   - RequireTlv() + Build() enforces mandatory TLV presence
///   - Chain-style API: every mutator returns *this

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <type_traits>
#include <vector>

namespace odin {
namespace sdk {

// =============================================================================
// PayloadBuilder  —  serialize parameters → bytes
// =============================================================================

class PayloadBuilder {
 public:
  PayloadBuilder() = default;

  // ---------------------------------------------------------------------------
  // Non-TLV: flat header bytes
  // ---------------------------------------------------------------------------

  /// Append a single byte.
  PayloadBuilder& WriteU8(uint8_t v);

  /// Append a 16-bit value in little-endian byte order.
  PayloadBuilder& WriteU16LE(uint16_t v);

  /// Append a 16-bit value in big-endian byte order.
  PayloadBuilder& WriteU16BE(uint16_t v);

  /// Append raw bytes.
  PayloadBuilder& WriteRaw(const uint8_t* data, size_t len);
  PayloadBuilder& WriteRaw(const std::vector<uint8_t>& data);

  // ---------------------------------------------------------------------------
  // TLV: [type: 1B] [length: sizeof(T) B] [value: big-endian bytes]
  //
  // Only integral types (uint8_t, uint16_t, uint32_t, ...) are accepted.
  // sizeof(T) determines the wire length; multi-byte values are always BE.
  //   AddTlv(type, uint8_t(v))   → [type][1][v]
  //   AddTlv(type, uint16_t(v))  → [type][2][BE high][BE low]
  //   AddTlv(type, uint32_t(v))  → [type][4][BE × 4]
  // ---------------------------------------------------------------------------

  /// Write a TLV for any integral type T (big-endian, length = sizeof(T)).
  template<typename T>
  typename std::enable_if<std::is_integral<T>::value, PayloadBuilder&>::type
  AddTlv(uint8_t type, T value) {
    uint8_t bytes[sizeof(T)];
    // Cast to unsigned to avoid implementation-defined right-shift on signed types.
    typedef typename std::make_unsigned<T>::type UT;
    uint64_t v = static_cast<uint64_t>(static_cast<UT>(value));
    for (size_t i = sizeof(T); i > 0; --i) {
      bytes[i - 1] = static_cast<uint8_t>(v & 0xFF);
      v >>= 8;
    }
    return AddRawTlv(type, bytes, sizeof(T));
  }

  /// Write a TLV with arbitrary raw value bytes (used for nested TLV blocks).
  PayloadBuilder& AddRawTlv(uint8_t type, const uint8_t* data, size_t len);

  // ---------------------------------------------------------------------------
  // Validation + output
  // ---------------------------------------------------------------------------

  /// Declare that TLV @p type must be present when Build() is called.
  /// If a required TLV was not added, Build() returns an empty vector.
  PayloadBuilder& RequireTlv(uint8_t type);

  /// Serialize to bytes. Returns empty vector if any required TLV is missing.
  std::vector<uint8_t> Build() const;

 private:
  std::vector<uint8_t> buf_;
  std::set<uint8_t>    written_tlv_types_;
  std::set<uint8_t>    required_tlv_types_;
};

// =============================================================================
// TlvParser  —  deserialize bytes → structured TLV access
// =============================================================================

class TlvParser {
 public:
  /// Parse TLV entries from @p data[0..len).
  /// Non-TLV header bytes before the TLV region should be skipped by the
  /// caller (pass data + header_size, len - header_size).
  static TlvParser Parse(const uint8_t* data, size_t len);

  /// Check whether TLV @p type was found during parsing.
  bool Has(uint8_t type) const;

  /// Get the 1-byte value of TLV @p type, or @p def if not found.
  uint8_t GetU8(uint8_t type, uint8_t def = 0) const;

  /// Get the 2-byte big-endian value of TLV @p type, or @p def if not found.
  uint16_t GetU16BE(uint8_t type, uint16_t def = 0) const;

  /// Get the 4-byte big-endian value of TLV @p type, or @p def if not found.
  uint32_t GetU32BE(uint8_t type, uint32_t def = 0) const;

  /// Get the raw value bytes of TLV @p type, or nullptr if not found.
  const std::vector<uint8_t>* GetRaw(uint8_t type) const;

 private:
  std::map<uint8_t, std::vector<uint8_t>> entries_;
};

}  // namespace sdk
}  // namespace odin
