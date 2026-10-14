#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "odin_lidar_def_ext.h"

namespace odin {
namespace sdk {

// Forward declaration - keep ICommand transport-agnostic.
// ITransportAddress is still used by GetLocalEndpoint() to expose a bind
// address opaquely; concrete transport types stay out of this header.
struct ITransportAddress;

/**
 * @brief Progress callback for long-running file operations.
 *
 * @param progress  0.0f ~ 100.0f. A negative value indicates failure.
 */
using CommandProgressCallback = std::function<void(float progress)>;

/**
 * @brief Device control channel - the single device-facing surface.
 *
 * ICommand is intentionally protocol-pluralistic: a concrete implementation
 * may internally speak ONE or SEVERAL backend protocols (e.g. an Odin TLV
 * channel for runtime control, an HTTP client for management endpoints, a
 * file-transfer client for OTA, or a single USB pipe that carries all of
 * them). Callers operate on device-level semantics (SetSensorMode, Reboot,
 * UploadFirmware, ...) and never see the routing.
 *
 * Concrete impls today:
 *   - OdinCommand : Ethernet. TLV/TCP for control + HTTP for mgmt + HTTPS for file.
 *   - (future) UsbCommand : USB. Single bulk channel for everything.
 */
class ICommand {
 public:
  virtual ~ICommand() = default;

  // ===========================================================================
  // Connection management
  // ===========================================================================

  /**
   * @brief Connect to device.
   *
   * The implementation extracts whatever transport-specific fields it needs
   * from the discovered-device descriptor. The device layer never inspects
   * those fields.
   */
  virtual bool Connect(const DiscoveredDevice& discovered_device,
                       OdinDeviceHandle handle) = 0;

  virtual void Disconnect() = 0;
  virtual bool IsConnected() const = 0;

  // ===========================================================================
  // Identity / capability queries
  // ===========================================================================
  virtual OdinResult GetFirmwareVersion(std::string& version,
                                        uint32_t timeout_ms = 1000) = 0;

  virtual OdinResult GetSensorCapability(
      std::vector<OdinSensorCapability>& capabilities,
      const std::vector<OdinDataChannel>& channels,
      uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Runtime control
  // ===========================================================================

  virtual OdinResult SetDeviceMode(OdinDeviceMode mode,
                                      uint32_t timeout_ms = 1000) = 0;

  virtual OdinResult SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Get device mode
   * @param mode Output current device mode
   */
  virtual OdinResult GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Get LiDAR sensor mode
   * @param mode Output current LiDAR mode
   */
  virtual OdinResult GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Stream control
  // ===========================================================================
  /**
   * @brief Send start-stream command.
   *
   * The local-side bind endpoint is allocated and owned by the implementation;
   * callers retrieve it via GetLocalEndpoint().
   */
  virtual OdinResult SendStartStream(OdinDataChannel channel,
                                     OdinTransportMode transport,
                                     const OdinStreamCfg* cfg = nullptr,
                                     uint32_t timeout_ms = 1000) = 0;

  virtual OdinResult SendCloseStream(OdinDataChannel channel,
                                     uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Local-side bind endpoint for an active streaming channel.
   *
   * Valid only between a successful SendStartStream() and the matching
   * SendCloseStream(). Typed as ITransportAddress so callers cannot reach
   * into transport-specific fields (port, USB ep id, ...).
   */
  virtual const ITransportAddress& GetLocalEndpoint(
      OdinDataChannel channel) const = 0;

  /**
   * @brief Query per-channel subscription state (cmd_id=0x0A)
   * @param channel  Channel to query
   * @param info     Output channel config info (TLV fields)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                                      uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Set LiDAR filter parameters (cmd_id=0x32)
   * @param params Filter parameter set (at least one has_xxx must be true)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetFilterParam(const OdinLidarFilterParams& params,
                                    uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query LiDAR filter parameters (cmd_id=0x33)
   * @param params Output: populated with current values; has_xxx set for each returned block
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetFilterParam(OdinLidarFilterParams& params,
                                    uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Restore default config (cmd_id=0x09)
  // ===========================================================================

  /**
   * @brief Restore factory default configuration (cmd_id=0x09)
   *
   * Deletes the custom configuration file. A reboot is required for the
   * default configuration to take effect.
   *
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult RestoreDefaultConfig(uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Time synchronization (0x20-0x2F)
  // ===========================================================================

  /**
   * @brief Set default time synchronization mode (cmd_id=0x20)
   *
   * Persisted to custom YAML; requires device reboot to take effect.
   *
   * @param mode Sync mode (kSysTick=system clock, kPtp=PTP hardware timestamp)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetDefaultSyncMode(OdinTimeSyncMode mode,
                                        uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query persisted default sync mode (cmd_id=0x21)
   *
   * Returns the sync_mode from config YAML (default or custom).
   * This is NOT the real-time PTP status; use GetPtpSyncStatus for that.
   *
   * @param mode Output: current persisted sync mode
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetDefaultSyncMode(OdinTimeSyncMode& mode,
                                        uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query PTP synchronization status (cmd_id=0x22)
   *
   * Real-time snapshot, not persisted.
   *
   * @param status Output: PTP sync status fields
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetPtpSyncStatus(OdinPtpSyncStatus& status,
                                      uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Standalone adaptive param commands (0x34/0x35)
  // ===========================================================================

  /**
   * @brief Set adaptive mode parameters (cmd_id=0x34)
   *
   * Configures LiDAR adaptive mode decision parameters (CDF percentile,
   * outdoor threshold). Only active when LiDAR mode is ADAPTIVE (mode 3).
   *
   * @param param Adaptive parameters
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetAdaptiveParam(const OdinAdaptiveFilterParam& param,
                                      uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query adaptive mode parameters (cmd_id=0x35)
   * @param param Output: current adaptive parameters
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetAdaptiveParam(OdinAdaptiveFilterParam& param,
                                      uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Algorithm commands (0x70-0x7F)
  // ===========================================================================

  /**
   * @brief Reset initial pose (cmd_id=0x70)
   *
   * Resets algorithm state (pose estimate, map, factor graph) so SLAM
   * re-initializes from scratch. Does not interrupt data streaming.
   *
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult ResetInitialPose(uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Heartbeat
  // ===========================================================================

  virtual bool IsHeartbeatSupported() const = 0;

  virtual bool SendHeartbeat(uint32_t interval_ms,
                             uint32_t timeout_ms = 1000) = 0;

  // ===========================================================================
  // Device management (folded in from the former IConfigService).
  // ===========================================================================
  //
  // These operations historically lived in a separate IConfigService driven
  // by HTTP. The split was protocol-driven and leaked through to the device
  // layer; it is now folded into ICommand so that callers stay agnostic of
  // which backend serves which operation.

  /**
   * @brief Read the device's current network attribute (ip / netmask / ...).
   */
  virtual OdinResult GetNetworkAttribute(NetworkAttribute& attr,
                                         uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Apply a new network attribute. Device usually reboots afterwards.
   */
  virtual OdinResult SetNetworkAttribute(const NetworkAttribute& attr,
                                         uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Reboot the whole device.
   *
   * The implementation MUST tear down its own internal channels (TLV, HTTP,
   * ...) before returning kOk, since the device will drop existing
   * connections immediately. Callers should re-Connect afterwards.
   */
  virtual OdinResult Reboot(uint32_t timeout_ms = 3000) = 0;

  // ===========================================================================
  // File transfer (folded in from the former IFileService).
  // ===========================================================================
  //
  // Generic file upload / download plus a packaged firmware OTA flow.
  // The caller is responsible for higher-level orchestration around OTA
  // (suspending heartbeat, switching to upgrade mode, etc.); this interface
  // only owns the file-transfer state machine.

  virtual OdinResult Upload(OdinFileType type,
                            const std::string& local_path,
                            CommandProgressCallback cb) = 0;

  virtual OdinResult Download(OdinFileType type,
                              const std::string& save_path,
                              CommandProgressCallback cb) = 0;

  virtual OdinResult UploadFirmware(const std::string& firmware_path,
                                    CommandProgressCallback cb,
                                    int timeout_s = 300) = 0;

  // ===========================================================================
  // Algorithm downlink passthrough (host -> device, data_type=0x40).
  //
  // Spec source of truth: docs/protocol/PROTOCOL.md sections 1.3 (0x40
  // ChannelConfig subscription, cmd_id 0x06) and 2.4 (v2 data frame framing).
  // ===========================================================================
  //
  // The host acts as a TCP server: the implementation opens a listening port
  // and subscribes the module to it (data_type=0x40) automatically on
  // Connect(), and tears it down on Disconnect(). The module connects in
  // asynchronously, so the link only needs to be up at send time; there is no
  // explicit start/stop on this interface.
  //
  // SendAlgoData() fragments [data, data+len) into v2 0x40 frames (per §2.4)
  // and writes them on the module connection. The payload is opaque: the SDK
  // never interprets it; semantics are an application contract.
  //
  // @param data  Opaque algorithm bytes (not interpreted).
  // @param len   Byte count of `data` (1 .. 256 KB).
  // @return kOk on success; kNotInitialized if the module has not connected to
  //         the downlink server yet; kInvalidArgument on null/size violations;
  //         kSocketError on write failure.
  virtual OdinResult SendAlgoData(const uint8_t* data, size_t len) = 0;

  // ===========================================================================
  // Calibration file fetch (asynchronous, data_type=0x41).
  //
  // The SDK receives the file in a background thread and saves it to
  // `save_path`. Progress callback convention (same as HTTP path):
  //   -  0..99 : incremental chunk progress (multi-chunk 0x41 path only)
  //   - 100.0f : transfer complete, file saved to `save_path`
  //   -  < 0.0f: failure
  //
  // Spec: docs/protocol/PROTOCOL.md section 1.3 (0x41).
  // ===========================================================================
  virtual OdinResult FetchCalibrationFile(const std::string& save_path,
                                          CommandProgressCallback cb,
                                          uint32_t timeout_ms = 5000) = 0;

  // ===========================================================================
  // Telemetry (data_type=0x42, continuous stream, device -> host).
  //
  // StartTelemetryServer() brings up the TCP server and subscribes the 0x42
  // channel. Device pushes JSON frames at ~1 Hz once subscribed.
  // RegisterTelemetryCallback() stores the user callback for parsed data.
  // ===========================================================================
  virtual void StartTelemetry() = 0;
  virtual void RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) = 0;
};

}  // namespace sdk
}  // namespace odin
