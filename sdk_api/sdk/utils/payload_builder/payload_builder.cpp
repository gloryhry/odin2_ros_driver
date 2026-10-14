#include "payload_builder.h"

namespace odin {
namespace sdk {

// =============================================================================
// PayloadBuilder — non-TLV writers
// =============================================================================

PayloadBuilder& PayloadBuilder::WriteU8(uint8_t v) {
  buf_.push_back(v);
  return *this;
}

PayloadBuilder& PayloadBuilder::WriteU16LE(uint16_t v) {
  buf_.push_back(static_cast<uint8_t>(v & 0xFF));
  buf_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  return *this;
}

PayloadBuilder& PayloadBuilder::WriteU16BE(uint16_t v) {
  buf_.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  buf_.push_back(static_cast<uint8_t>(v & 0xFF));
  return *this;
}

PayloadBuilder& PayloadBuilder::WriteRaw(const uint8_t* data, size_t len) {
  if (data && len > 0) {
    buf_.insert(buf_.end(), data, data + len);
  }
  return *this;
}

PayloadBuilder& PayloadBuilder::WriteRaw(const std::vector<uint8_t>& data) {
  buf_.insert(buf_.end(), data.begin(), data.end());
  return *this;
}

// =============================================================================
// PayloadBuilder — TLV writers (template implementations live in the header)
// =============================================================================

PayloadBuilder& PayloadBuilder::AddRawTlv(uint8_t type,
                                           const uint8_t* data,
                                           size_t len) {
  buf_.push_back(type);
  buf_.push_back(static_cast<uint8_t>(len));
  if (data && len > 0) {
    buf_.insert(buf_.end(), data, data + len);
  }
  written_tlv_types_.insert(type);
  return *this;
}

// =============================================================================
// PayloadBuilder — validation + build
// =============================================================================

PayloadBuilder& PayloadBuilder::RequireTlv(uint8_t type) {
  required_tlv_types_.insert(type);
  return *this;
}

std::vector<uint8_t> PayloadBuilder::Build() const {
  // Check all required TLV types are present.
  for (std::set<uint8_t>::const_iterator it = required_tlv_types_.begin();
       it != required_tlv_types_.end(); ++it) {
    if (written_tlv_types_.count(*it) == 0) {
      return std::vector<uint8_t>();  // missing required TLV
    }
  }
  return buf_;
}

// =============================================================================
// TlvParser
// =============================================================================

TlvParser TlvParser::Parse(const uint8_t* data, size_t len) {
  TlvParser parser;
  if (!data || len == 0) return parser;

  size_t offset = 0;
  while (offset + 2 <= len) {
    uint8_t type = data[offset];
    uint8_t vlen = data[offset + 1];
    offset += 2;
    if (offset + vlen > len) break;  // truncated — stop parsing

    std::vector<uint8_t> value(data + offset, data + offset + vlen);
    parser.entries_[type] = value;
    offset += vlen;
  }
  return parser;
}

bool TlvParser::Has(uint8_t type) const {
  return entries_.count(type) > 0;
}

uint8_t TlvParser::GetU8(uint8_t type, uint8_t def) const {
  std::map<uint8_t, std::vector<uint8_t> >::const_iterator it = entries_.find(type);
  if (it == entries_.end() || it->second.empty()) return def;
  return it->second[0];
}

uint16_t TlvParser::GetU16BE(uint8_t type, uint16_t def) const {
  std::map<uint8_t, std::vector<uint8_t> >::const_iterator it = entries_.find(type);
  if (it == entries_.end() || it->second.size() < 2) return def;
  return (static_cast<uint16_t>(it->second[0]) << 8) |
          static_cast<uint16_t>(it->second[1]);
}

uint32_t TlvParser::GetU32BE(uint8_t type, uint32_t def) const {
  std::map<uint8_t, std::vector<uint8_t> >::const_iterator it = entries_.find(type);
  if (it == entries_.end() || it->second.size() < 4) return def;
  return (static_cast<uint32_t>(it->second[0]) << 24) |
         (static_cast<uint32_t>(it->second[1]) << 16) |
         (static_cast<uint32_t>(it->second[2]) <<  8) |
          static_cast<uint32_t>(it->second[3]);
}

const std::vector<uint8_t>* TlvParser::GetRaw(uint8_t type) const {
  std::map<uint8_t, std::vector<uint8_t> >::const_iterator it = entries_.find(type);
  if (it == entries_.end()) return nullptr;
  return &it->second;
}

}  // namespace sdk
}  // namespace odin
