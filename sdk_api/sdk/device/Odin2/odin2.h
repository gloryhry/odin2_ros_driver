#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "IDevice.h"
#include "ICommand.hpp"
#include "ICapture.hpp"
#include "OdinFrameAssembler.hpp"
#include "SlamOdomSynchronizer.h"

namespace odin {
namespace sdk {

/**
 * @brief Odin2 device implementation (Ethernet)
 *
 * Refactored device class that separates command handling and data capture via the
 * ICommand and ICapture interfaces. Odin2Device is only responsible for business
 * logic orchestration.
 */
class Odin2Device : public IDevice {
 public:
  explicit Odin2Device(OdinDeviceHandle handle);
  ~Odin2Device() override;

  // IDevice interface implementation
  bool Connect(const DiscoveredDevice& discovered_device) override;
  void Disconnect() override;
  bool IsConnected() const override;

  OdinDeviceHandle GetHandle() const override;
  std::string GetSerialNumber() const override;
  std::string GetModel() const override;
  MTConnectionType GetConnectionType() const override;
  const DiscoveredDevice& GetDiscoveredDevice() const override;

  OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) override;
  OdinResult SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms = 1000) override;

  // Callback registration - forwarded to capture_
  void RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) override;
  void RegisterSlamCallback(OdinSlamCallback cb, void* user_data) override;
  void RegisterCameraCallback(OdinCameraCallback cb, void* user_data) override;
  void RegisterCameraCallback2(OdinCameraCallback cb, void* user_data) override;
  void RegisterImuCallback(OdinImuCallback cb, void* user_data) override;
  void RegisterOdomCallback(OdinOdomCallback cb, void* user_data) override;
  // Telemetry (data_type=0x42, opt-in continuous stream)
  OdinResult StartTelemetry() override;
  void RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) override;
  void RegisterTfCallback(OdinTfCallback cb, void* user_data) override;

  // Data channel management
  void StopDataChannels() override;

  OdinResult SendFile(OdinFileType type, const std::string& file_path,
                      OdinUpgradeProgressCallback cb) override;
  OdinResult GetFile(OdinFileType type, const std::string& save_path,
                     OdinUpgradeProgressCallback cb,
                     uint32_t timeout_ms = 5000) override;
  OdinResult SendAlgoData(const uint8_t* data, size_t len) override;
  OdinResult GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                 const std::vector<OdinDataChannel>& channels,
                                 uint32_t timeout_ms) override;
  OdinResult StartStream(OdinDataChannel channel, OdinTransportMode transport,
                         const OdinStreamCfg* mode = nullptr,
                         uint32_t timeout_ms = 1000) override;
  OdinResult CloseStream(OdinDataChannel channel, uint32_t timeout_ms) override;
  OdinResult GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                              uint32_t timeout_ms = 1000) override;
  OdinResult SetFilterParam(const OdinLidarFilterParams& params,
                            uint32_t timeout_ms = 1000) override;
  OdinResult GetFilterParam(OdinLidarFilterParams& params,
                            uint32_t timeout_ms = 1000) override;

  // New protocol commands
  OdinResult RestoreDefaultConfig(uint32_t timeout_ms = 1000) override;
  OdinResult SetDefaultSyncMode(OdinTimeSyncMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetDefaultSyncMode(OdinTimeSyncMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetPtpSyncStatus(OdinPtpSyncStatus& status, uint32_t timeout_ms = 1000) override;
  OdinResult SetAdaptiveParam(const OdinAdaptiveFilterParam& param, uint32_t timeout_ms = 1000) override;
  OdinResult GetAdaptiveParam(OdinAdaptiveFilterParam& param, uint32_t timeout_ms = 1000) override;
  OdinResult ResetInitialPose(uint32_t timeout_ms = 1000) override;

  void EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag = 10) override;
  void SetHeartbeatSuccessCallback(HeartbeatSuccessCallback cb) override;
  void SetHeartbeatInterval(uint32_t interval_ms);
  void SetHeartbeatTimeout(uint32_t timeout_ms);

  // Network configuration
  OdinResult GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms = 3000) override;
  OdinResult SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms = 3000) override;
  OdinResult Reboot(uint32_t timeout_ms = 3000) override;

 private:
  // Firmware OTA helper
  OdinResult SendFileFirmwareOta(const std::string& file_path, OdinUpgradeProgressCallback cb);

  // Heartbeat management
  void StartHeartbeat();
  void StopHeartbeat();
  void HeartbeatThread();

  // Device information (must be first for initialization order)
  OdinDeviceHandle handle_;

  // Core components.  ICommand is now the unified device-control surface;
  // network attribute / reboot / file transfer all flow through it.
  std::unique_ptr<ICommand> command_;
  
  // Capture instances (one per channel)
  std::map<OdinDataChannel, std::unique_ptr<ICapture>> captures_;
  
  // Frame assemblers for protocol parsing (one per channel type)
  std::map<OdinDataChannel, std::unique_ptr<OdinFrameAssembler>> assemblers_;
  
  // User callbacks
  OdinPointCloudCallback point_cloud_cb_ = nullptr;
  void* point_cloud_user_data_ = nullptr;
  OdinSlamCallback slam_cb_ = nullptr;
  void* slam_user_data_ = nullptr;
  OdinCameraCallback camera_cb_ = nullptr;
  void* camera_user_data_ = nullptr;
  OdinCameraCallback camera2_cb_ = nullptr;
  void* camera2_user_data_ = nullptr;
  OdinImuCallback imu_cb_ = nullptr;
  void* imu_user_data_ = nullptr;
  OdinOdomCallback odom_cb_ = nullptr;
  void* odom_user_data_ = nullptr;
  // Calibration file: no longer stored as a callback member; fetched
  // synchronously via FetchCalibrationFile() -> command_->FetchCalibrationFile().
  // Telemetry callback is still stored for continuous streaming.
  OdinTelemetryCallback telemetry_cb_ = nullptr;
  void* telemetry_user_data_ = nullptr;
  OdinTfCallback tf_cb_ = nullptr;
  void* tf_user_data_ = nullptr;
  // The discovered-device descriptor is the single source of truth for
  // device identity / addressing. All accessors (GetSerialNumber, GetModel,
  // GetDiscoveredDevice, ...) read from it directly, so we no longer
  // duplicate sn / model / ip / firmware as separate members.
  DiscoveredDevice discovered_device_;

  std::atomic<bool> connected_{false};
  mutable std::mutex mutex_;

  // Heartbeat thread
  std::thread heartbeat_thread_;
  std::atomic<bool> heartbeat_running_{false};
  uint32_t heartbeat_interval_ms_ = 500;
  int heartbeat_max_failures_ = 6;
  HeartbeatSuccessCallback heartbeat_success_cb_;

  // SLAM-Odom synchronizer
  std::unique_ptr<SlamOdomSynchronizer> slam_odom_sync_;
};

}  // namespace sdk
}  // namespace odin
