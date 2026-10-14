#pragma once

#include <memory>
#include <string>

// HotplugCallbacks / OnAttachCallback / OnDetachCallback are public types
// defined in odin_lidar_def.h. This internal interface only reuses them.
#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// =============================================================================
// IHotplugListener - Abstract interface for device hotplug detection
// =============================================================================

class IHotplugListener {
 public:
  virtual ~IHotplugListener() = default;

  /**
   * @brief Start listening for device hotplug events
   * @param callbacks Structure containing on_attach and on_detach callbacks
   * @param enumerate_existing If true, report already-connected devices via on_attach
   * @return true if started successfully
   */
  virtual bool Start(const HotplugCallbacks& callbacks, bool enumerate_existing = true) = 0;

  /**
   * @brief Stop listening for device hotplug events
   */
  virtual void Stop() = 0;

  /**
   * @brief Check if listener is running
   * @return true if running
   */
  virtual bool IsRunning() const = 0;

  /**
   * @brief Set the polling interval for device detection
   * @param interval_ms Interval in milliseconds
   */
  virtual void SetPollingInterval(uint32_t interval_ms) = 0;

  /**
   * @brief Set the timeout for considering a device offline
   * @param timeout_ms Timeout in milliseconds (device not responding)
   */
  virtual void SetOfflineTimeout(uint32_t timeout_ms) = 0;

  /**
   * @brief Refresh the keep-alive timestamp of a device.
   *
   * This is the single keep-alive signal the listener listens to. The SDK
   * routes successful heartbeats here so a connected device's last_seen keeps
   * advancing even when it stops answering discovery probes (e.g. while busy
   * streaming). Offline detection is unified across "discovered but never
   * connected" and "connected then dropped" cases: in both, last_seen simply
   * stops advancing and the discovery-timeout path fires on_detach.
   *
   * The full DiscoveredDevice is passed so each listener picks whatever
   * identifier fits it (IP for network, bus/port/SN for USB, ...) without
   * leaking that choice through the interface.
   *
   * The default implementation is a no-op so listeners that do not need it
   * (e.g. USB, where libusb delivers a hardware DEVICE_LEFT event) can ignore
   * the signal.
   *
   * @param device Device descriptor whose keep-alive timestamp to refresh
   */
  virtual void RefreshDeviceAlive(const DiscoveredDevice& device) { (void)device; }
};

}  // namespace sdk
}  // namespace odin
