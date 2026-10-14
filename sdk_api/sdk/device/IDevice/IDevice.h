#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "odin_lidar_def_ext.h"  // Types - includes ext types for advanced commands

namespace odin {
namespace sdk {

/**
 * @brief Device interface - abstract base class for all device types
 *
 * This interface allows OdinSdkImpl to work with different device types
 * (network, USB, 5G, Bluetooth) through a common abstraction.
 */
class IDevice {
 public:
  virtual ~IDevice() = default;

  // =========================================================================
  // Connection Management
  // =========================================================================

  /**
   * @brief Initialize and connect to the device
   * @param discovered_device Device info from discovery
   * @return true if connection successful
   */
  virtual bool Connect(const DiscoveredDevice& discovered_device) = 0;

  /**
   * @brief Disconnect from the device
   */
  virtual void Disconnect() = 0;

  /**
   * @brief Check if device is connected
   * @return true if connected
   */
  virtual bool IsConnected() const = 0;

  // =========================================================================
  // Device Information
  // =========================================================================

  /**
   * @brief Get device handle
   * @return Device handle
   */
  virtual OdinDeviceHandle GetHandle() const = 0;

  /**
   * @brief Get device serial number
   * @return Serial number string
   */
  virtual std::string GetSerialNumber() const = 0;

  /**
   * @brief Get device model name
   * @return Model name string
   */
  virtual std::string GetModel() const = 0;

  /**
   * @brief Get connection type
   * @return Connection type enum
   */
  virtual MTConnectionType GetConnectionType() const = 0;

  /**
   * @brief Get discovered device info (saved from Connect)
   * @return DiscoveredDevice structure
   */
  virtual const DiscoveredDevice& GetDiscoveredDevice() const = 0;

  // =========================================================================
  // Command Interface
  // =========================================================================

  /**
   * @brief Get device firmware version (synchronous)
   * @param version Output firmware version string (e.g., "1.2.3")
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Set operating mode synchronously
   * @param mode Operating mode (kStandby, kNormal, kUpgrade)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Data Streaming & Callbacks
  // =========================================================================

  /**
   * @brief Register callbacks for data processing
   * Device handles its own data parsing and invokes these callbacks
   */
  virtual void RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) = 0;
  virtual void RegisterSlamCallback(OdinSlamCallback cb, void* user_data) = 0;
  virtual void RegisterCameraCallback(OdinCameraCallback cb, void* user_data) = 0;
  virtual void RegisterCameraCallback2(OdinCameraCallback cb, void* user_data) = 0;
  virtual void RegisterImuCallback(OdinImuCallback cb, void* user_data) = 0;
  virtual void RegisterOdomCallback(OdinOdomCallback cb, void* user_data) = 0;
  virtual void RegisterTfCallback(OdinTfCallback cb, void* user_data) = 0;

  /**
   * @brief Start telemetry subscription (data_type=0x42, opt-in).
   *
   * Brings up a TCP server and subscribes the 0x42 channel so the device
   * starts pushing JSON telemetry frames (~1 Hz). Must be called explicitly
   * after ConnectDevice() when telemetry is desired.
   */
  virtual OdinResult StartTelemetry() = 0;

  /**
   * @brief Register telemetry callback (data_type=0x42, continuous stream).
   */
  virtual void RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) = 0;

  /**
   * @brief Stop data channels
   */
  virtual void StopDataChannels() = 0;

  // =========================================================================
  // File Operations
  // =========================================================================

  /**
   * @brief Send file to device.
   *
   * For firmware upgrade (kFirmware type), drives the complete OTA flow:
   * uploads the firmware, waits for the device-side state machine to reach
   * a terminal state (DONE / FAILED), and reports progress 0~100% via cb.
   *
   * The choice of underlying transport/protocol (e.g., HTTP, HTTPS, USB) is
   * an implementation detail and MUST NOT be assumed by callers.
   *
   * @param type File type (kFirmware for OTA upgrade)
   * @param file_path Local file path
   * @param cb Progress callback (0-100%)
   * @return Operation result
   */
  virtual OdinResult SendFile(OdinFileType type, const std::string& file_path,
                              OdinUpgradeProgressCallback cb) = 0;

  /**
   * @brief Get file from device.
   *
   * For `kCalibrationFile`, the implementation prefers the active TCP
   * connection (0x41 data channel) and falls back to HTTP/HTTPS only if
   * the channel cannot be initiated. Progress callback convention:
   *   - 100.0  : transfer complete, file saved to `save_path`
   *   - < 0.0  : failure (channel error or timeout)
   *   - 0..100 : incremental progress (HTTP path only)
   *
   * @param type       File type
   * @param save_path  Local save path
   * @param cb         Progress/done callback
   * @param timeout_ms Timeout for 0x41 channel transfer (default 5000 ms)
   * @return kOk if transfer was initiated; error code otherwise
   */
  virtual OdinResult GetFile(OdinFileType type, const std::string& save_path,
                             OdinUpgradeProgressCallback cb,
                             uint32_t timeout_ms = 5000) = 0;

  // =========================================================================
  // Sensor Configuration
  // =========================================================================

  /**
   * @brief Set sensor mode (synchronous)
   * @param mode Sensor mode value (0=HDR, 1=High Peak, 2=Low Peak, 3=Adaptive)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Get LiDAR mode (cmd_id=0x09) - Synchronous
   * @param mode Output current LiDAR mode (0=HDR, 1=High Peak, 2=Low Peak, 3=Adaptive)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Send algorithm downlink passthrough data to the device (0x40).
   *
   * Host->device opaque algorithm bytes. The downlink TCP server and its 0x40
   * subscription are set up automatically on connect; the host listens
   * asynchronously and the module connects in, so the link only needs to be up
   * at send time. The SDK fragments the payload per protocol §2.4 and does NOT
   * interpret it; semantics are an application contract.
   *
   * Wire spec: docs/protocol/PROTOCOL.md section 2.4 (v2 data frame).
   *
   * @param data Opaque algorithm bytes (1 .. 256 KB).
   * @param len  Byte count of `data`.
   * @return kOk on success; kNotInitialized if the module is not connected;
   *         kInvalidArgument on null/size violations; kSocketError on write
   *         failure.
   */
  virtual OdinResult SendAlgoData(const uint8_t* data, size_t len) = 0;

  /**
   * @brief Get device mode (cmd_id=0x08) - Synchronous
   * @param mode Output current device mode (0x00: Standby, 0x01: Normal)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Start a data stream (cmd_id=0x05) - Synchronous
   * @param channel Data channel type
   * @param transport Transport mode (UDP/TCP)
   * @param mode Image stream mode (optional, only for Image channels)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult StartStream(OdinDataChannel channel, OdinTransportMode transport,
                                 const OdinStreamCfg* mode = nullptr,
                                 uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Get sensor capabilities (cmd_id=0x08) - Synchronous
   * @param capabilities Output: list of sensor capabilities
   * @param channels Channels to query (empty = query all)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                         const std::vector<OdinDataChannel>& channels,
                                         uint32_t timeout_ms) = 0;

  /**
   * @brief Close a data stream (cmd_id=0x05) - Synchronous
   * @param channel Data channel type
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult CloseStream(OdinDataChannel channel, uint32_t timeout_ms) = 0;

  /**
   * @brief Query per-channel subscription state (cmd_id=0x06)
   * @param channel  Channel to query
   * @param info     Output channel config info (TLV fields)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                                      uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Set filter parameters (cmd_id=0x32)
   * @param params List of TLV filter params (at least 1)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetFilterParam(const OdinLidarFilterParams& params,
                                    uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query LiDAR filter parameters (cmd_id=0x33)
   * @param params Output: populated with current values
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetFilterParam(OdinLidarFilterParams& params,
                                    uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Restore Default Config (cmd_id=0x09)
  // =========================================================================

  /**
   * @brief Restore factory default configuration (cmd_id=0x09)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult RestoreDefaultConfig(uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Time Synchronization (0x20-0x2F)
  // =========================================================================

  /**
   * @brief Set default time sync mode (cmd_id=0x20, persisted, requires reboot)
   * @param mode Sync mode (kSysTick or kPtp)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetDefaultSyncMode(OdinTimeSyncMode mode,
                                        uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query persisted default sync mode (cmd_id=0x21)
   * @param mode Output: current persisted sync mode
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetDefaultSyncMode(OdinTimeSyncMode& mode,
                                        uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query PTP sync status (cmd_id=0x22, real-time snapshot)
   * @param status Output PTP sync status
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetPtpSyncStatus(OdinPtpSyncStatus& status,
                                      uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Standalone Adaptive Param (0x34/0x35)
  // =========================================================================

  /**
   * @brief Set adaptive mode parameters (cmd_id=0x34)
   * @param param Adaptive parameters (cdf_percentile, outdoor_threshold)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetAdaptiveParam(const OdinAdaptiveFilterParam& param,
                                      uint32_t timeout_ms = 1000) = 0;

  /**
   * @brief Query adaptive mode parameters (cmd_id=0x35)
   * @param param Output adaptive parameters
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetAdaptiveParam(OdinAdaptiveFilterParam& param,
                                      uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // Algorithm Commands (0x70-0x7F)
  // =========================================================================

  /**
   * @brief Reset initial pose (cmd_id=0x70)
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult ResetInitialPose(uint32_t timeout_ms = 1000) = 0;

  // =========================================================================
  // SLAM-Odom Synchronization
  // =========================================================================

  /**
   * @brief Enable/disable SLAM-Odom synchronization for this device
   * @param enabled Whether to enable sync
   * @param max_frame_lag Maximum frame count difference before discarding unmatched data
   */
  virtual void EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag = 10) = 0;

  // =========================================================================
  // Network Configuration
  // =========================================================================

  /**
   * @brief Get device network attribute configuration.
   *
   * Retrieves current network settings (IP, gateway, netmask, DHCP, PTP).
   * The underlying transport/protocol is implementation-defined and MUST NOT
   * be assumed by callers (it may be HTTP today, USB control protocol later).
   *
   * @param attr Output: network attribute structure
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Set device network attribute configuration.
   *
   * Configures network settings (IP, gateway, netmask, DHCP, PTP).
   * Changes take effect after device reboot. The underlying transport /
   * protocol is implementation-defined.
   *
   * @param attr Network attribute to set
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms = 3000) = 0;

  /**
   * @brief Reboot the device.
   *
   * Requests the device to restart. Pending network setting changes take
   * effect after reboot. The underlying transport / protocol is
   * implementation-defined.
   *
   * @param timeout_ms Timeout in milliseconds
   * @return Operation result
   */
  virtual OdinResult Reboot(uint32_t timeout_ms = 3000) = 0;

  // =========================================================================
  // Connection Status Callbacks
  // =========================================================================

  /**
   * @brief Callback type for heartbeat success (keep-alive) notification
   * @param handle Device handle
   */
  using HeartbeatSuccessCallback = std::function<void(OdinDeviceHandle handle)>;

  /**
   * @brief Register callback for successful heartbeat round-trip.
   *
   * This is used as a keep-alive signal: the SDK routes it to the hotplug
   * listener so a connected device that stops answering discovery probes
   * (e.g. because it is streaming) is not incorrectly marked offline.
   *
   * @param cb Callback function invoked on every successful heartbeat
   */
  virtual void SetHeartbeatSuccessCallback(HeartbeatSuccessCallback cb) = 0;
};

}  // namespace sdk
}  // namespace odin
