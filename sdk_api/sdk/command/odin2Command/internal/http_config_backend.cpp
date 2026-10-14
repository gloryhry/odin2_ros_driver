#include "http_config_backend.hpp"

#include "http_client.h"
#include "logger.h"

namespace odin {
namespace sdk {

namespace {

// Minimal JSON helpers (avoid pulling a JSON library into this small surface).
std::string ExtractString(const std::string& body, const std::string& key) {
  std::string search = "\"" + key + "\":\"";
  size_t pos = body.find(search);
  if (pos == std::string::npos) return "";
  pos += search.length();
  size_t end = body.find("\"", pos);
  if (end == std::string::npos) return "";
  return body.substr(pos, end - pos);
}

bool ExtractBool(const std::string& body, const std::string& key) {
  std::string search = "\"" + key + "\":";
  size_t pos = body.find(search);
  if (pos == std::string::npos) return false;
  pos += search.length();
  while (pos < body.size() && (body[pos] == ' ' || body[pos] == '\t')) {
    ++pos;
  }
  return body.substr(pos, 4) == "true";
}

}  // namespace

HttpConfigBackend::HttpConfigBackend() = default;
HttpConfigBackend::~HttpConfigBackend() = default;

bool HttpConfigBackend::Connect(const DiscoveredDevice& device) {
  const std::string& ip = device.network.ip_address;
  if (ip.empty()) {
    LOG_ERROR("HttpConfigBackend::Connect: device has no IP\n");
    return false;
  }
  device_ip_ = ip;
  connected_.store(true);
  LOG_INFO("HttpConfigBackend: bound to %s:%u\n", device_ip_.c_str(), port_);
  return true;
}

void HttpConfigBackend::Disconnect() {
  connected_.store(false);
  device_ip_.clear();
}

std::string HttpConfigBackend::BuildBaseUrl() const {
  return "http://" + device_ip_ + ":" + std::to_string(port_);
}

OdinResult HttpConfigBackend::GetNetworkAttribute(NetworkAttribute& attr,
                                                  uint32_t timeout_ms) {
  (void)timeout_ms;  // http::Client manages its own timeout internally.
  if (!connected_.load()) return OdinResult::kNotInitialized;

  http::Client client(BuildBaseUrl());
  std::string response;
  if (!client.get_network(response)) {
    LOG_ERROR("HttpConfigBackend::GetNetworkAttribute failed\n");
    return OdinResult::kUnknownError;
  }

  attr.ip_address = ExtractString(response, "ip_address");
  attr.gateway = ExtractString(response, "gateway");
  attr.netmask = ExtractString(response, "netmask");
  attr.mac = ExtractString(response, "mac");
  attr.dhcp = ExtractBool(response, "dhcp");
  attr.ptp = ExtractBool(response, "ptp");

  LOG_INFO("HttpConfigBackend: GetNetworkAttribute ip=%s gw=%s mask=%s mac=%s dhcp=%d ptp=%d\n",
           attr.ip_address.c_str(), attr.gateway.c_str(), attr.netmask.c_str(),
           attr.mac.c_str(), attr.dhcp ? 1 : 0, attr.ptp ? 1 : 0);
  return OdinResult::kOk;
}

OdinResult HttpConfigBackend::SetNetworkAttribute(const NetworkAttribute& attr,
                                                  uint32_t timeout_ms) {
  (void)timeout_ms;
  if (!connected_.load()) return OdinResult::kNotInitialized;

  http::Client client(BuildBaseUrl());
  std::string body = "{";
  body += "\"ip_address\":\"" + attr.ip_address + "\",";
  body += "\"gateway\":\"" + attr.gateway + "\",";
  body += "\"netmask\":\"" + attr.netmask + "\",";
  body += "\"dhcp\":" + std::string(attr.dhcp ? "true" : "false") + ",";
  body += "\"ptp\":" + std::string(attr.ptp ? "true" : "false");
  body += "}";

  if (!client.set_network(body)) {
    LOG_ERROR("HttpConfigBackend::SetNetworkAttribute failed\n");
    return OdinResult::kUnknownError;
  }
  LOG_INFO("HttpConfigBackend: SetNetworkAttribute ip=%s gw=%s mask=%s dhcp=%d ptp=%d\n",
           attr.ip_address.c_str(), attr.gateway.c_str(), attr.netmask.c_str(),
           attr.dhcp ? 1 : 0, attr.ptp ? 1 : 0);
  return OdinResult::kOk;
}

OdinResult HttpConfigBackend::Reboot(uint32_t timeout_ms) {
  (void)timeout_ms;
  if (!connected_.load()) return OdinResult::kNotInitialized;

  http::Client client(BuildBaseUrl());
  LOG_INFO("HttpConfigBackend: sending reboot...\n");
  if (!client.reboot()) {
    LOG_ERROR("HttpConfigBackend::Reboot failed\n");
    return OdinResult::kUnknownError;
  }
  LOG_INFO("HttpConfigBackend: reboot accepted\n");
  return OdinResult::kOk;
}

}  // namespace sdk
}  // namespace odin
