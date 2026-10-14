#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "../ICommand.hpp"
#include "odin_command_channel.h"

namespace odin {
namespace sdk {

class HttpConfigBackend;
class HttpFileBackend;
class TcpTransport;

// Odin2 Command IDs (aligned with PROTOCOL.md reorganization)
enum class Odin2CmdId : uint16_t {
  kDeviceQuery = 0x01,
  kVersionQuery = 0x02,
  kSetDeviceMode = 0x03,            // SetDeviceMode  (was 0x04)
  kGetDeviceMode = 0x04,            // GetDeviceMode  (was 0x08)
  kChannelConfig = 0x05,            // SetChannelConfig (was 0x06)
  kGetChannelConfig = 0x06,         // GetChannelConfig (was 0x0A)
  kHeartbeat = 0x07,
  kCmdIdQueryCapability = 0x08,     // GetDeviceCapability (was 0x03)
  kRestoreDefaultConfig = 0x09,     // RestoreDefaultConfig (was 0x0F)
  // LiDAR group 0x30-0x4F
  kSensorMode = 0x30,               // SetLiDARMode  (was 0x05)
  kGetLiDARMode = 0x31,             // GetLiDARMode  (was 0x09)
  kSetFilterParam = 0x32,           // SetLiDARFilterParam (was 0x0B)
  kGetFilterParam = 0x33,           // GetLiDARFilterParam (was 0x0C)
  kSetAdaptiveParam = 0x34,         // SetAdaptiveParam (standalone, not via filter TLV)
  kGetAdaptiveParam = 0x35,         // GetAdaptiveParam (standalone)
  // Time sync group 0x20-0x2F
  kSetDefaultSyncMode = 0x20,       // SetDefaultSyncMode (persistent, requires reboot)
  kGetDefaultSyncMode = 0x21,       // GetDefaultSyncMode (query persisted sync_mode)
  kGetPtpSyncStatus = 0x22,         // GetPtpSyncStatus (real-time snapshot)
  // Algorithm group 0x70-0x7F
  kResetInitialPose = 0x70,         // ResetInitialPose
};

// TLV Type IDs for ChannelConfig command (aligned with protocol v0x01, fdf0eac)
enum class ChannelConfigTlvType : uint8_t {
  kSubscribe     = 0x01,  // 1B  mandatory: 1=subscribe, 0=unsubscribe
  kDstPort       = 0x02,  // 2B BE: routing only (no longer implies subscribe)
  kTransportMode = 0x03,  // 1B  0=UDP, 1=TCP (network only; USB bearer ignores)
  kResolutionId  = 0x04,  // 1B
  kFpsId         = 0x05,  // 1B
  kFormat        = 0x06,  // 1B
};

/**
 * @brief Odin protocol command channel implementation
 *
 * Implements the ICommand interface and encapsulates Odin2 protocol
 * command encoding and parsing.
 */
class OdinCommand : public ICommand {
 public:
  OdinCommand();
  ~OdinCommand() override;

  // ICommand interface implementation
  bool Connect(const DiscoveredDevice& discovered_device,
               OdinDeviceHandle handle) override;
  void Disconnect() override;
  bool IsConnected() const override;

  OdinResult GetFirmwareVersion(std::string& version, uint32_t timeout_ms = 1000) override;
  OdinResult SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                  const std::vector<OdinDataChannel>& channels,
                                  uint32_t timeout_ms = 1000) override;

  OdinResult SendStartStream(OdinDataChannel channel, OdinTransportMode transport,
                              const OdinStreamCfg* cfg = nullptr,
                              uint32_t timeout_ms = 1000) override;
  OdinResult SendCloseStream(OdinDataChannel channel, uint32_t timeout_ms = 1000) override;
  OdinResult GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                            uint32_t timeout_ms = 1000) override;
  OdinResult SetFilterParam(const OdinLidarFilterParams& params,
                            uint32_t timeout_ms = 1000) override;
  OdinResult GetFilterParam(OdinLidarFilterParams& params,
                            uint32_t timeout_ms = 1000) override;

  // Restore default config (cmd_id=0x09)
  OdinResult RestoreDefaultConfig(uint32_t timeout_ms = 1000) override;
  // Time sync commands (0x20-0x2F)
  OdinResult SetDefaultSyncMode(OdinTimeSyncMode mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetDefaultSyncMode(OdinTimeSyncMode& mode, uint32_t timeout_ms = 1000) override;
  OdinResult GetPtpSyncStatus(OdinPtpSyncStatus& status, uint32_t timeout_ms = 1000) override;
  // Standalone adaptive param commands (0x34/0x35)
  OdinResult SetAdaptiveParam(const OdinAdaptiveFilterParam& param, uint32_t timeout_ms = 1000) override;
  OdinResult GetAdaptiveParam(OdinAdaptiveFilterParam& param, uint32_t timeout_ms = 1000) override;
  // Algorithm commands (0x70-0x7F)
  OdinResult ResetInitialPose(uint32_t timeout_ms = 1000) override;

  const ITransportAddress& GetLocalEndpoint(OdinDataChannel channel) const override;

  bool IsHeartbeatSupported() const override;
  bool SendHeartbeat(uint32_t interval_ms, uint32_t timeout_ms = 1000) override;

  // Device management (HTTP backend internally).
  OdinResult GetNetworkAttribute(NetworkAttribute& attr,
                                 uint32_t timeout_ms = 3000) override;
  OdinResult SetNetworkAttribute(const NetworkAttribute& attr,
                                 uint32_t timeout_ms = 3000) override;
  OdinResult Reboot(uint32_t timeout_ms = 3000) override;

  // File transfer (HTTP/HTTPS backend internally).
  OdinResult Upload(OdinFileType type, const std::string& local_path,
                    CommandProgressCallback cb) override;
  OdinResult Download(OdinFileType type, const std::string& save_path,
                      CommandProgressCallback cb) override;
  OdinResult UploadFirmware(const std::string& firmware_path,
                            CommandProgressCallback cb,
                            int timeout_s = 300) override;

  // Algorithm downlink passthrough (data_type=0x40). The TCP server and its
  // 0x40 subscription are managed in Connect()/Disconnect(); this call only
  // frames [data, data+len) per protocol §2.4 and writes it to the module.
  OdinResult SendAlgoData(const uint8_t* data, size_t len) override;

  // Calibration file fetch (asynchronous, data_type=0x41).
  // Background thread receives the file and saves it to `save_path`;
  // `cb` reports 0..99% per chunk (multi-chunk path), 100.0f on success,
  // negative on failure.
  OdinResult FetchCalibrationFile(const std::string& save_path,
                                  CommandProgressCallback cb,
                                  uint32_t timeout_ms = 5000) override;

  // Synchronous body of FetchCalibrationFile; runs in a background thread.
  void FetchCalibrationFileThread(const std::string& save_path,
                                  CommandProgressCallback cb,
                                  uint32_t timeout_ms);

  // Signals and joins the background calib fetch thread.
  void StopCalibThread();

  // Telemetry (data_type=0x42, continuous stream)
  void StartTelemetry() override;
  void RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) override;

  // Internal capability table access (used by StartStream to resolve resolution_id/fps_id)
  void UpdateCapabilityTable(const std::vector<OdinSensorCapability>& capabilities);
  bool LookupStreamConfig(OdinDataChannel channel, const OdinStreamCfg* cfg,
                          uint8_t& resolution_id, uint8_t& fps_id);

 private:
  // TLV building helper
  std::vector<uint8_t> BuildChannelConfigPayload(OdinDataChannel channel,
                                                  OdinTransportMode transport,
                                                  uint16_t dst_port,
                                                  const OdinStreamCfg* cfg);

  // Local-side data-plane port allocator. Owned here because OdinCommand is
  // the layer that hands the chosen port to the peer in the channel-config
  // payload, so it is the single source of truth for which ports are in use.
  uint16_t AllocateDstPort();
  static bool ProbePortBindable(const std::string& ip, uint16_t port);

  // ChannelConfig (cmd_id 0x05) payload for a raw data_type byte (e.g. the
  // 0x40 downlink) that is not part of the OdinDataChannel enum.
  std::vector<uint8_t> BuildChannelConfigPayloadRaw(uint8_t data_type,
                                                    OdinTransportMode transport,
                                                    uint16_t dst_port);

  // Bring up / tear down the algorithm downlink (data_type=0x40) TCP server
  // and the module subscription. Called from Connect()/Disconnect() while
  // mutex_ is held. Failures are non-fatal to the main connection.
  void StartAlgoDownlink();
  void StopAlgoDownlink();

  // Tear down uplink TCP server for telemetry (0x42).
  void StopTelemetryServer();

  // Calibration helpers (called from FetchCalibrationFile only).
  void UnsubscribeCalibChannel();
  void CleanupCalibServer();

  // Internal TCP receive handlers for uplink channels.
  void HandleCalibData(const uint8_t* data, size_t length);
  void HandleTelemetryData(const uint8_t* data, size_t length);

  std::unique_ptr<CommandChannel> command_channel_;
  std::unique_ptr<CommandChannel> heartbeat_channel_;

  // Backend protocols composed alongside the TLV channel. Owned via
  // unique_ptr to keep their headers out of this public-ish header.
  std::unique_ptr<HttpConfigBackend> http_config_;
  std::unique_ptr<HttpFileBackend>   http_file_;

  // Algorithm downlink passthrough (data_type=0x40). The host owns a TCP
  // server (TcpTransport in server mode) that the module connects into.
  // Created in Connect(), destroyed in Disconnect(). Guarded by algo_mutex_
  // (separate from mutex_) so a large downlink write never blocks the command
  // / heartbeat path.
  std::unique_ptr<TcpTransport> algo_downlink_;
  uint16_t algo_downlink_port_ = 0;
  uint32_t algo_frame_seq_ = 0;
  bool algo_not_connected_warned_ = false;
  mutable std::mutex algo_mutex_;

  // Calibration file uplink (data_type=0x41). Host opens a TCP server on
  // demand in FetchCalibrationFile(), subscribes the module, waits for the
  // complete file (multi-chunk reassembly), then tears everything down.
  std::unique_ptr<TcpTransport> calib_server_;
  uint16_t calib_server_port_ = 0;
  mutable std::mutex calib_mutex_;
  std::condition_variable calib_cv_;
  // Partial reassembly state for the current calib file frame.
  std::vector<uint8_t> calib_reassembly_buf_;
  uint16_t calib_expected_chunks_ = 0;
  uint16_t calib_received_chunks_ = 0;
  uint32_t calib_frame_seq_ = 0;
  bool calib_file_received_ = false;   ///< Set by HandleCalibData, consumed by FetchCalibrationFileThread
  CommandProgressCallback calib_progress_cb_;  ///< Guarded by calib_mutex_; set by FetchCalibrationFile, read by HandleCalibData for incremental progress.

  // Background fetch state. FetchCalibrationFile spawns a thread; Disconnect
  // / destructor signal cancellation and join it before tearing down channels.
  std::thread calib_thread_;
  std::atomic<bool> calib_thread_cancel_{false};

  // Telemetry uplink (data_type=0x42). Host owns a TCP server; module pushes
  // self-contained JSON frames at ~1 Hz (no reassembly needed).
  std::unique_ptr<TcpTransport> telemetry_server_;
  uint16_t telemetry_server_port_ = 0;
  mutable std::mutex telemetry_mutex_;
  OdinTelemetryCallback telemetry_cb_ = nullptr;
  void* telemetry_user_data_ = nullptr;

  std::atomic<bool> connected_{false};
  bool heartbeat_supported_ = true;
  
  OdinDeviceHandle handle_ = kInvalidDeviceHandle;
  std::string device_ip_;
  std::string host_ip_;
  
  // Internal capability table
  std::map<OdinDataChannel, std::vector<OdinStreamCfg>> capability_table_;
  mutable std::mutex mutex_;
  std::mutex capability_mutex_;

  // Allocated local endpoints (data-plane destinations), keyed by channel.
  // The address is constructed in SendStartStream and removed by
  // SendCloseStream; capture queries it through GetLocalEndpoint().
  std::set<uint16_t> used_ports_;
  std::map<OdinDataChannel, NetworkAddress> channel_endpoints_;
  mutable std::mutex port_mutex_;
};

}  // namespace sdk
}  // namespace odin
