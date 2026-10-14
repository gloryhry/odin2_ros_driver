#include "odin2.h"
#include "OdinCommand.hpp"
#include "OdinCaptureBase.hpp"

#include "logger.h"

#include <Eigen/Dense>

#include <chrono>
#include <future>
#include <thread>

namespace odin {
namespace sdk {

Odin2Device::Odin2Device(OdinDeviceHandle handle)
    : handle_(handle),
      command_(std::make_unique<OdinCommand>()),
      slam_odom_sync_(std::make_unique<SlamOdomSynchronizer>()) {
  // Set SLAM transform function for coordinate transformation using Odom pose
  // Input: float format (already converted from uint16 by OdinFrameAssembler)
  // Output: float format with rotation and translation applied
  if (slam_odom_sync_) {
    slam_odom_sync_->SetSlamTransformFunction(
        [](OdinSlamPacket& packet, const OdinOdomPacket& odom_pkt) {
          // Match found - transform points to world frame: pt_world = R * pt_imu + t
          if (odom_pkt.data.empty()) {
            return;
          }
          const OdinOdomData& odom_data = odom_pkt.data[0];

          // Extract quaternion and position from odom (wire is now float SI units:
          // position in meters, orientation a unit quaternion -> no scaling).
          constexpr double kPosScale = 1.0;
          constexpr double kOrientScale = 1.0;
          double tx = odom_data.pos[0] * kPosScale;
          double ty = odom_data.pos[1] * kPosScale;
          double tz = odom_data.pos[2] * kPosScale;
          double qx = odom_data.orient[0] * kOrientScale;
          double qy = odom_data.orient[1] * kOrientScale;
          double qz = odom_data.orient[2] * kOrientScale;
          double qw = odom_data.orient[3] * kOrientScale;

          // Build rotation matrix R^{w}_{i} from quaternion using Eigen
          Eigen::Quaterniond q(qw, qx, qy, qz);
          q.normalize();
          Eigen::Matrix3d R = q.toRotationMatrix();
          Eigen::Vector3d t(tx, ty, tz);

          // Transform each point: pt_world = R * pt_imu + t
          // Input is already float format (converted by OdinFrameAssembler)
          for (auto& pt : packet.points) {
            // Convert from millimeters to meters for transformation
            Eigen::Vector3d pt_imu(static_cast<double>(pt.x) * 0.001,
                                   static_cast<double>(pt.y) * 0.001,
                                   static_cast<double>(pt.z) * 0.001);

            // Apply rotation and translation: pt_world = R * pt_imu + t
            Eigen::Vector3d pt_world = R * pt_imu + t;

            // Store back as float (in millimeters)
            pt.x = static_cast<float>(pt_world.x() / 0.001);
            pt.y = static_cast<float>(pt_world.y() / 0.001);
            pt.z = static_cast<float>(pt_world.z() / 0.001);
            // r, g, b, a remain unchanged
          }
        });
  }
}

Odin2Device::~Odin2Device() {
  Disconnect();
}

bool Odin2Device::Connect(const DiscoveredDevice& discovered_device) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_.load()) return true;

  // Cache the descriptor and forward it down. The device layer never
  // inspects transport-specific fields (IP, host_ip, USB path, ...);
  // each component pulls the bits it needs from the descriptor itself.
  discovered_device_ = discovered_device;

  // The command layer is now the unified device-control surface; it pulls
  // out whatever transport-specific bits it needs from the descriptor and
  // brings up its own internal sub-protocols (TLV / HTTP / file ...).
  if (!command_->Connect(discovered_device_, handle_)) {
    LOG_ERROR("Odin2Device::Connect: Command connect failed\n");
    return false;
  }

  connected_.store(true);
  LOG_INFO("Odin2Device connected: %s (SN: %s, Model: %s)\n",
           discovered_device_.network.ip_address.c_str(),
           discovered_device_.sn.c_str(),
           discovered_device_.model.c_str());

  StartHeartbeat();
  return true;
}

void Odin2Device::Disconnect() {
  StopHeartbeat();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_.load()) return;

  StopDataChannels();
  command_->Disconnect();

  connected_.store(false);
  LOG_INFO("Odin2Device disconnected: %s\n",  
           discovered_device_.network.ip_address.c_str());
}

bool Odin2Device::IsConnected() const {
  return connected_.load();
}

OdinDeviceHandle Odin2Device::GetHandle() const {
  return handle_;
}

std::string Odin2Device::GetSerialNumber() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_device_.sn;
}

std::string Odin2Device::GetModel() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return discovered_device_.model;
}

MTConnectionType Odin2Device::GetConnectionType() const {
  return MTConnectionType::kEthernet;
}

const DiscoveredDevice& Odin2Device::GetDiscoveredDevice() const {
  return discovered_device_;
}

OdinResult Odin2Device::GetFirmwareVersion(std::string& version, uint32_t timeout_ms) {
  return command_->GetFirmwareVersion(version, timeout_ms);
}

OdinResult Odin2Device::SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms) {
  return command_->SetDeviceMode(mode, timeout_ms);
}

OdinResult Odin2Device::SendAlgoData(const uint8_t* data, size_t len) {
  if (!IsConnected() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->SendAlgoData(data, len);
}

OdinResult Odin2Device::GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms) {
  return command_->GetDeviceMode(mode, timeout_ms);
}

OdinResult Odin2Device::SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms) {
  return command_->SetSensorMode(mode, timeout_ms);
}

OdinResult Odin2Device::GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms) {
  return command_->GetSensorMode(mode, timeout_ms);
}

OdinResult Odin2Device::GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                             const std::vector<OdinDataChannel>& channels,
                                             uint32_t timeout_ms) {
  return command_->GetSensorCapability(capabilities, channels, timeout_ms);
}

void Odin2Device::EnableSlamOdomSync(bool enabled, uint32_t max_frame_lag) {
  if (slam_odom_sync_) {
    slam_odom_sync_->SetEnabled(enabled);
    slam_odom_sync_->SetMaxFrameLag(max_frame_lag);
    LOG_INFO("Odin2Device[%d]: SLAM-Odom sync %s (max_lag=%u)\n", handle_,
             enabled ? "enabled" : "disabled", max_frame_lag);
  }
}

OdinResult Odin2Device::StartStream(OdinDataChannel channel, OdinTransportMode transport,
                                     const OdinStreamCfg* mode, uint32_t timeout_ms) {
  // Validate: mode parameter only valid for Image channels
  bool is_image_channel = (channel == OdinDataChannel::kImage0 || 
                           channel == OdinDataChannel::kImage1);
  if (mode != nullptr && !is_image_channel) {
    LOG_ERROR("StartStream: mode parameter only valid for Image channels\n");
    return OdinResult::kInvalidArgument;
  }

  // Determine if H264 format is requested (affects callback routing)
  bool is_h264 = (mode != nullptr && mode->format == OdinDataFormat::kImageH264);
  // Width/height from the user-provided StreamCfg; applied as fallback in lambdas
  // (assembler emits 0 for both fields; the lambda fills them from config if still 0).
  uint16_t stream_width  = (mode != nullptr) ? mode->width  : 0;
  uint16_t stream_height = (mode != nullptr) ? mode->height : 0;

  // Step 1: Send start-stream command. The command layer owns local-side
  // endpoint allocation; the device never sees the underlying port.
  OdinResult result = command_->SendStartStream(channel, transport, mode, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("StartStream: SendStartStream failed\n");
    return result;
  }

  // Step 2: Create capture and assembler. The transport mode is forwarded
  // verbatim; classification of TCP-style vs datagram-style behaviour is
  // encapsulated inside each component.
  auto capture = std::make_unique<NetworkCapture>(transport);
  auto assembler = std::make_unique<OdinFrameAssembler>(handle_, transport);

  // Non-H264 image streams (MJPEG / NV12) share the generic image assembly
  // path; tell the assembler which format the device was configured to send
  // so emitted frames carry the correct OdinDataFormat.
  if (is_image_channel && !is_h264 && mode != nullptr) {
    assembler->SetImageFormat(mode->format);
  }

  // Set up assembler callbacks based on channel
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      assembler->SetPointCloudCallback([this](const OdinPointCloudPacket& pkt) {
        if (point_cloud_cb_) point_cloud_cb_(pkt, point_cloud_user_data_);
      });
      break;
    case OdinDataChannel::kSlamPoint:
      assembler->SetSlamCallback([this](const OdinSlamPacket& pkt) {
        // Data is already in float format (converted by OdinFrameAssembler)
        // Route through synchronizer for coordinate transformation if enabled
        bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        if (sync_enabled) {
          // Synchronizer will apply rotation/translation and call user callback
          slam_odom_sync_->ProcessSlam(pkt);
        } else {
          // No sync - call user callback directly (data already in float format)
          if (slam_cb_) {
            slam_cb_(pkt, slam_user_data_);
          }
        }
      });
      break;
    case OdinDataChannel::kImage0:
      if (is_h264) {
        assembler->SetH264Callback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera_cb_) camera_cb_(p, camera_user_data_);
        });
      } else {
        assembler->SetImageCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera_cb_) camera_cb_(p, camera_user_data_);
        });
      }
      break;
    case OdinDataChannel::kImage1:
      if (is_h264) {
        assembler->SetH264Callback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera2_cb_) camera2_cb_(p, camera2_user_data_);
        });
      } else {
        assembler->SetImageCallback([this, stream_width, stream_height](const OdinCameraPacket& pkt) {
          OdinCameraPacket p = pkt;
          if (p.width == 0) p.width = stream_width;
          if (p.height == 0) p.height = stream_height;
          if (camera2_cb_) camera2_cb_(p, camera2_user_data_);
        });
      }
      break;
    case OdinDataChannel::kImu:
      assembler->SetImuCallback([this](const OdinImuPacket& pkt) {
        if (imu_cb_) imu_cb_(pkt, imu_user_data_);
      });
      break;
    // Odom Routing rules:
    //   - LF + sync enabled  -> SlamOdomSynchronizer (SLAM/odom alignment).
    //   - LF + sync disabled -> deliver to user callback directly.
    //   - HF (always)        -> deliver to user callback directly, bypassing
    //                            the synchronizer (HF odom must not be gated
    //                            by SLAM frame timing).
    case OdinDataChannel::kOdomLF:
      assembler->SetOdomCallback([this](const OdinOdomPacket& pkt, OdomSourceType type) {
        const bool sync_enabled = slam_odom_sync_ && slam_odom_sync_->IsEnabled();
        if(sync_enabled){
          slam_odom_sync_->ProcessOdom(pkt, type);
        }
        else{
          if (odom_cb_) odom_cb_(pkt, type, odom_user_data_);
        }
      });
      break;
    case OdinDataChannel::kOdomHF:
      assembler->SetOdomCallback([this](const OdinOdomPacket& pkt, OdomSourceType type) {
        if (odom_cb_) odom_cb_(pkt, type, odom_user_data_);
      });
      break;
    case OdinDataChannel::kTf:
      assembler->SetTfCallback([this](const OdinTfPacket& pkt) {
        if (tf_cb_) tf_cb_(pkt, tf_user_data_);
      });
      break;
    default:
      LOG_ERROR("StartStream: Unknown channel 0x%02x\n", static_cast<uint8_t>(channel));
      command_->SendCloseStream(channel, timeout_ms);
      return OdinResult::kInvalidArgument;
  }

  // Create raw data callback that routes to assembler
  OdinFrameAssembler* asm_ptr = assembler.get();
  RawDataCallback raw_cb;
  switch (channel) {
    case OdinDataChannel::kRawPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessRawPointData(data, len);
      };
      break;
    case OdinDataChannel::kSlamPoint:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessSlamPointData(data, len);
      };
      break;
    case OdinDataChannel::kImage0:
    case OdinDataChannel::kImage1:
      if (is_h264) {
        raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
          asm_ptr->ProcessH264Data(data, len);
        };
      } else {
        raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
          asm_ptr->ProcessImageData(data, len);
        };
      }
      break;
    case OdinDataChannel::kImu:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessImuData(data, len);
      };
      break;
    case OdinDataChannel::kOdomLF:
    case OdinDataChannel::kOdomHF:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessOdomData(data, len);
      };
      break;
    case OdinDataChannel::kTf:
      raw_cb = [asm_ptr](const uint8_t* data, size_t len, const ITransportAddress&) {
        asm_ptr->ProcessTfData(data, len);
      };
      break;
    default:
      break;
  }

  // Start capture against the local endpoint chosen by the command layer.
  // The reference is opaque to the device (typed as ITransportAddress) and
  // is forwarded straight to capture without inspecting its fields.
  if (!capture->Start(command_->GetLocalEndpoint(channel), raw_cb)) {
    LOG_ERROR("StartStream: StartCapture failed\n");
    command_->SendCloseStream(channel, timeout_ms);
    return OdinResult::kSocketError;
  }
  
  captures_[channel] = std::move(capture);
  assemblers_[channel] = std::move(assembler);

  LOG_INFO("StartStream: channel=0x%02x, local=%s, %s ok\n",
           static_cast<uint8_t>(channel),
           command_->GetLocalEndpoint(channel).ToString().c_str(),
           transport == OdinTransportMode::kTcp ? "TCP" : "UDP");
  return OdinResult::kOk;
}

OdinResult Odin2Device::CloseStream(OdinDataChannel channel, uint32_t timeout_ms) {
  // Step 1: Stop and remove capture instance
  auto it = captures_.find(channel);
  if (it != captures_.end()) {
    it->second->Stop();
    captures_.erase(it);
  }

  // Remove assembler
  auto asm_it = assemblers_.find(channel);
  if (asm_it != assemblers_.end()) {
    assemblers_.erase(asm_it);
  }

  // Step 2: Send close stream command
  OdinResult result = command_->SendCloseStream(channel, timeout_ms);
  if (result != OdinResult::kOk) {
    LOG_ERROR("CloseStream: SendCloseStream failed\n");
    return result;
  }

  LOG_INFO("CloseStream: channel=0x%02x ok\n", static_cast<uint8_t>(channel));
  return OdinResult::kOk;
}

OdinResult Odin2Device::GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                                          uint32_t timeout_ms) {
  return command_->GetChannelConfig(channel, info, timeout_ms);
}

OdinResult Odin2Device::SetFilterParam(const OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  return command_->SetFilterParam(params, timeout_ms);
}

OdinResult Odin2Device::GetFilterParam(OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  return command_->GetFilterParam(params, timeout_ms);
}

OdinResult Odin2Device::RestoreDefaultConfig(uint32_t timeout_ms) {
  return command_->RestoreDefaultConfig(timeout_ms);
}

OdinResult Odin2Device::SetDefaultSyncMode(OdinTimeSyncMode mode, uint32_t timeout_ms) {
  return command_->SetDefaultSyncMode(mode, timeout_ms);
}

OdinResult Odin2Device::GetDefaultSyncMode(OdinTimeSyncMode& mode, uint32_t timeout_ms) {
  return command_->GetDefaultSyncMode(mode, timeout_ms);
}

OdinResult Odin2Device::GetPtpSyncStatus(OdinPtpSyncStatus& status, uint32_t timeout_ms) {
  return command_->GetPtpSyncStatus(status, timeout_ms);
}

OdinResult Odin2Device::SetAdaptiveParam(const OdinAdaptiveFilterParam& param,
                                         uint32_t timeout_ms) {
  return command_->SetAdaptiveParam(param, timeout_ms);
}

OdinResult Odin2Device::GetAdaptiveParam(OdinAdaptiveFilterParam& param,
                                         uint32_t timeout_ms) {
  return command_->GetAdaptiveParam(param, timeout_ms);
}

OdinResult Odin2Device::ResetInitialPose(uint32_t timeout_ms) {
  return command_->ResetInitialPose(timeout_ms);
}

void Odin2Device::RegisterPointCloudCallback(OdinPointCloudCallback cb, void* user_data) {
  point_cloud_cb_ = cb;
  point_cloud_user_data_ = user_data;
}

void Odin2Device::RegisterSlamCallback(OdinSlamCallback cb, void* user_data) {
  slam_cb_ = cb;
  slam_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(cb, user_data, odom_cb_, odom_user_data_);
  }
}

void Odin2Device::RegisterCameraCallback(OdinCameraCallback cb, void* user_data) {
  camera_cb_ = cb;
  camera_user_data_ = user_data;
}

void Odin2Device::RegisterCameraCallback2(OdinCameraCallback cb, void* user_data) {
  camera2_cb_ = cb;
  camera2_user_data_ = user_data;
}

void Odin2Device::RegisterImuCallback(OdinImuCallback cb, void* user_data) {
  imu_cb_ = cb;
  imu_user_data_ = user_data;
}

void Odin2Device::RegisterOdomCallback(OdinOdomCallback cb, void* user_data) {
  odom_cb_ = cb;
  odom_user_data_ = user_data;
  // Update synchronizer callbacks
  if (slam_odom_sync_) {
    slam_odom_sync_->SetCallbacks(slam_cb_, slam_user_data_, cb, user_data);
  }
}

namespace {
// Build a CommandProgressCallback that forwards progress to the public
// OdinUpgradeProgressCallback, carrying the device handle.
CommandProgressCallback MakeProgressAdapter(OdinDeviceHandle handle,
                                            OdinUpgradeProgressCallback cb) {
  if (!cb) return nullptr;
  return [cb, handle](float progress) {
    cb(handle, progress, nullptr);
  };
}
}  // namespace


void Odin2Device::RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) {
  telemetry_cb_ = cb;
  telemetry_user_data_ = user_data;
  if (command_) {
    command_->RegisterTelemetryCallback(cb, user_data);
  }
}

OdinResult Odin2Device::StartTelemetry() {
  if (!command_) return OdinResult::kNotInitialized;
  command_->StartTelemetry();
  return OdinResult::kOk;
}

void Odin2Device::RegisterTfCallback(OdinTfCallback cb, void* user_data) {
  tf_cb_ = cb;
  tf_user_data_ = user_data;
}

void Odin2Device::StopDataChannels() {
  for (auto& pair : captures_) {
    if (pair.second) {
      pair.second->Stop();
    }
  }
  captures_.clear();
  assemblers_.clear();
}

// =============================================================================
// File Transfer: delegated to ICommand (formerly IFileService)
// =============================================================================

OdinResult Odin2Device::SendFile(OdinFileType type, const std::string& file_path,
                                  OdinUpgradeProgressCallback cb) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }
  if (type == OdinFileType::kFirmware) {
    return SendFileFirmwareOta(file_path, cb);
  }

  return command_->Upload(type, file_path, MakeProgressAdapter(handle_, cb));
}

OdinResult Odin2Device::GetFile(OdinFileType type, const std::string& save_path,
                                 OdinUpgradeProgressCallback cb,
                                 uint32_t timeout_ms) {
  if (!IsConnected()) {
    return OdinResult::kNotInitialized;
  }

  if (type == OdinFileType::kCalibrationFile) {
    // Prefer the active TCP connection (0x41 channel) over HTTP — BLOCKING.
    // Use a shared_ptr<promise> so the lambda is safe even if this function
    // returns early (wait_for timeout) while the background thread is still live.
    auto done_flag = std::make_shared<std::promise<bool>>();
    std::future<bool> done_future = done_flag->get_future();
    // Adapt CommandProgressCallback: forward incremental progress to the outer
    // OdinUpgradeProgressCallback; trigger the promise only on the final value
    // (100.0f = success, < 0.0f = failure) so set_value is called exactly once.
    CommandProgressCallback calib_progress_cb =
        [done_flag, cb, this](float progress) {
          if (progress >= 100.0f) {
            if (cb) cb(handle_, 100.0f, nullptr);
            done_flag->set_value(true);
          } else if (progress < 0.0f) {
            done_flag->set_value(false);
          } else {
            // Incremental chunk progress (0x41 multi-chunk path only).
            if (cb) cb(handle_, progress, nullptr);
          }
        };

    OdinResult init_result = command_->FetchCalibrationFile(save_path, calib_progress_cb, timeout_ms);
    if (init_result == OdinResult::kOk) {
      // Block until the background thread signals or the extended deadline passes.
      constexpr uint32_t kExtraMarginMs = 2000;
      auto status = done_future.wait_for(
          std::chrono::milliseconds(timeout_ms + kExtraMarginMs));
      if (status == std::future_status::ready && done_future.get()) {
        // cb already forwarded inside calib_progress_cb at 100.0f
        return OdinResult::kOk;
      }
      LOG_WARN("Odin2Device::GetFile: 0x41 transfer failed/timed out, falling back to HTTP\n");
    } else {
      LOG_WARN("Odin2Device::GetFile: 0x41 channel init failed (err=%d), falling back to HTTP\n",
               static_cast<int>(init_result));
    }
  }

  // HTTP path: synchronous blocking (HttpFileBackend::Download returns after save).
  return command_->Download(type, save_path, MakeProgressAdapter(handle_, cb));
}

// =============================================================================
// Firmware OTA orchestration: device owns heartbeat/device-mode, service
// owns the file transfer + state polling.
// =============================================================================

OdinResult Odin2Device::SendFileFirmwareOta(const std::string& firmware_path,
                                             OdinUpgradeProgressCallback cb) {
  if (!command_) {
    return OdinResult::kNotInitialized;
  }

  LOG_INFO("SendFileFirmwareOta: Stopping heartbeat for OTA\n");
  StopHeartbeat();

  OdinResult mode_result = command_->SetDeviceMode(OdinDeviceMode::kUpgrade, 3000);
  if (mode_result != OdinResult::kOk) {
    LOG_ERROR("SendFileFirmwareOta: Failed to set upgrade mode\n");
    StartHeartbeat();
    return mode_result;
  }

  OdinResult result = command_->UploadFirmware(firmware_path,
                                                MakeProgressAdapter(handle_, cb), 300);
  if (result == OdinResult::kOk) {
    // OTA succeeded — device has rebooted with new firmware.
    // Heartbeat was intentionally stopped before OTA, so it never "failed".
    // The caller is expected to call DisconnectDevice() to clean up the stale
    // session; do not synthesize any offline/heartbeat-failure event here.
    return OdinResult::kOk;
  }

  // On non-fatal failure (e.g. HTTPS-fallback path), restart heartbeat so the
  // device session keeps running.
  StartHeartbeat();
  return result;
}

// =============================================================================
// Heartbeat
// =============================================================================

void Odin2Device::StartHeartbeat() {
  if (heartbeat_running_.load()) return;

  if (!command_->IsHeartbeatSupported()) {
    LOG_INFO("Odin2Device: Heartbeat not supported\n");
    return;
  }

  heartbeat_running_.store(true);
  heartbeat_thread_ = std::thread(&Odin2Device::HeartbeatThread, this);
  LOG_DEBUG("Odin2Device: Heartbeat thread started\n");
}

void Odin2Device::StopHeartbeat() {
  if (!heartbeat_running_.load()) return;

  heartbeat_running_.store(false);

  if (heartbeat_thread_.joinable()) {
    if (std::this_thread::get_id() == heartbeat_thread_.get_id()) {
      heartbeat_thread_.detach();
    } else {
      heartbeat_thread_.join();
    }
  }
  LOG_DEBUG("Odin2Device: Heartbeat thread stopped\n");
}

void Odin2Device::HeartbeatThread() {
  int consecutive_failures = 0;

  while (heartbeat_running_.load()) {
    uint32_t elapsed = 0;
    while (heartbeat_running_.load() && elapsed < heartbeat_interval_ms_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      elapsed += 100;
    }

    if (!heartbeat_running_.load()) break;

    if (!command_->IsHeartbeatSupported()) {
      continue;
    }

    if (command_->SendHeartbeat(heartbeat_interval_ms_, 1000)) {
      consecutive_failures = 0;
      if (heartbeat_success_cb_) {
        heartbeat_success_cb_(handle_);
      }
    } else {
      consecutive_failures++;
      LOG_WARN("Odin2Device: Heartbeat failed (%d/%d)\n", 
               consecutive_failures, heartbeat_max_failures_);

      if (consecutive_failures >= heartbeat_max_failures_) {
        LOG_ERROR("Odin2Device: Heartbeat failed %d times, device may be unreachable\n",
                  consecutive_failures);
        // Keep the heartbeat thread alive: offline detection is delegated to
        // the hotplug listener (no success callbacks -> last_seen expires).
        // If the network/device recovers, the next successful heartbeat will
        // refresh last_seen and the device stays online without a spurious
        // detach/attach cycle.
      }
    }
  }
}

void Odin2Device::SetHeartbeatSuccessCallback(HeartbeatSuccessCallback cb) {
  heartbeat_success_cb_ = cb;
}

void Odin2Device::SetHeartbeatInterval(uint32_t interval_ms) {
  heartbeat_interval_ms_ = interval_ms;
  LOG_INFO("Odin2Device: Heartbeat interval set to %u ms\n", interval_ms);
}

void Odin2Device::SetHeartbeatTimeout(uint32_t timeout_ms) {
  heartbeat_max_failures_ = static_cast<int>((timeout_ms + heartbeat_interval_ms_ - 1) / heartbeat_interval_ms_);
  LOG_INFO("Odin2Device: Heartbeat timeout set to %u ms, max failures = %d\n", 
           timeout_ms, heartbeat_max_failures_);
}

// =============================================================================
// Control-plane: delegated to ICommand (formerly IConfigService)
// =============================================================================

OdinResult Odin2Device::GetNetworkAttribute(NetworkAttribute& attr, uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->GetNetworkAttribute(attr, timeout_ms);
}

OdinResult Odin2Device::SetNetworkAttribute(const NetworkAttribute& attr, uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  return command_->SetNetworkAttribute(attr, timeout_ms);
}

OdinResult Odin2Device::Reboot(uint32_t timeout_ms) {
  if (!connected_.load() || !command_) {
    return OdinResult::kNotInitialized;
  }
  // ICommand::Reboot tears down the underlying TLV/HTTP channels itself,
  // so we mirror that at the device layer to keep IsConnected() honest.
  OdinResult r = command_->Reboot(timeout_ms);
  StopHeartbeat();
  connected_.store(false);
  return r;
}

}  // namespace sdk
}  // namespace odin
