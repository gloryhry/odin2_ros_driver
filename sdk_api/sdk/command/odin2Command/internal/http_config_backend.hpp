#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

/**
 * @brief HTTP/REST backend used internally by OdinCommand.
 *
 * Speaks to the device's web management API (network attribute query /
 * update, reboot). Not a public service - OdinCommand composes it.
 */
class HttpConfigBackend {
 public:
  HttpConfigBackend();
  ~HttpConfigBackend();

  bool Connect(const DiscoveredDevice& device);
  void Disconnect();
  bool IsConnected() const { return connected_.load(); }

  OdinResult GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms);
  OdinResult SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms);
  OdinResult Reboot(uint32_t timeout_ms);

  // Optional override of the default port (8080).
  void SetPort(uint16_t port) { port_ = port; }

 private:
  std::string BuildBaseUrl() const;

  std::string device_ip_;
  uint16_t port_ = 8080;
  std::atomic<bool> connected_{false};
};

}  // namespace sdk
}  // namespace odin
