/**
 * @file main.cpp
 * @brief Multi-device hotplug demo with frame rate statistics
 *
 * This sample demonstrates:
 * - Hotplug detection (device arrival/removal)
 * - Multi-device support
 * - Firmware version and sensor capability query
 * - Device mode query/set  (GetDeviceMode / SetDeviceMode)
 * - LiDAR mode query/set   (GetSensorMode  / SetSensorMode)
 * - Filter params query/set (GetFilterParam / SetFilterParam)
 * - Stream control with channel verification (StartStream / GetChannelConfig / CloseStream)
 * - Frame rate statistics for each data type
 * - Automatic resource cleanup on device removal
 * - Re-connection when device comes back online
 *
 * Connection sequence (per protocol spec):
 *   1. GetDeviceMode           -- confirm current device state
 *   2. SetDeviceMode(Standby)  -- enter standby for configuration
 *   3. GetSensorMode           -- read current LiDAR mode
 *   4. SetSensorMode           -- configure desired LiDAR mode
 *   5. GetSensorMode           -- verify LiDAR sensor mode took effect
 *   6. StartStream x6          -- subscribe all data channels
 *   7. GetChannelConfig        -- verify per-channel subscription
 *   8. SetDeviceMode(Normal)   -- start data streaming
 */

#include "odin_lidar_api_ext.h"
#include "logger.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace odin::sdk;

// =============================================================================
// Configuration
// =============================================================================

constexpr int kFpsUpdateIntervalMs = 1000;  // FPS update interval

// =============================================================================
// Data Structures
// =============================================================================

struct FrameStats {
  std::atomic<uint64_t> frame_count{0};
  std::chrono::steady_clock::time_point last_update;
  double fps{0.0};

  FrameStats() : last_update(std::chrono::steady_clock::now()) {}

  void Reset() {
    frame_count.store(0);
    last_update = std::chrono::steady_clock::now();
    fps = 0.0;
  }
};

struct DeviceContext {
  DiscoveredDevice device;
  OdinDeviceHandle handle{kInvalidDeviceHandle};
  bool connected{false};

  // Frame statistics for each data type
  FrameStats raw_stats;
  FrameStats slam_stats;
  FrameStats image_stats;
  FrameStats imu_stats;
  FrameStats odom_lf_stats;
  FrameStats odom_hf_stats;
  FrameStats image2_stats;

  std::chrono::steady_clock::time_point connect_time;
};

// =============================================================================
// Global State
// =============================================================================

std::atomic<bool> g_running{true};
std::mutex g_devices_mutex;
std::map<std::string, std::unique_ptr<DeviceContext>> g_devices;  // key = SN

void SignalHandler(int) { g_running.store(false); }

// =============================================================================
// Utility Functions
// =============================================================================

std::string GetTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_info;
  localtime_r(&time, &tm_info);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &tm_info);
  return buffer;
}

void UpdateFps(FrameStats& stats) {
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats.last_update);
  if (elapsed.count() >= kFpsUpdateIntervalMs) {
    uint64_t frames = stats.frame_count.exchange(0);
    stats.fps = static_cast<double>(frames) * 1000.0 / elapsed.count();
    stats.last_update = now;
  }
}

DeviceContext* FindDeviceByHandle(OdinDeviceHandle handle) {
  for (auto& kv : g_devices) {
    if (kv.second->handle == handle) {
      return kv.second.get();
    }
  }
  return nullptr;
}

// =============================================================================
// Callbacks
// =============================================================================

void PointCloudCallback(const OdinPointCloudPacket& packet, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    ctx->raw_stats.frame_count++;
  }
}

void SlamCallback(const OdinSlamPacket& packet, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    ctx->slam_stats.frame_count++;
  }
}

void CameraCallback(const OdinCameraPacket& packet, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    ctx->image_stats.frame_count++;
  }
}

void CameraCallback2(const OdinCameraPacket& packet, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    ctx->image2_stats.frame_count++;
  }
}

void ImuCallback(const OdinImuPacket& packet, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    ctx->imu_stats.frame_count++;
  }
}

void OdomCallback(const OdinOdomPacket& packet, OdomSourceType odom_type, void* client_data) {
  std::lock_guard<std::mutex> lock(g_devices_mutex);
  if (auto* ctx = static_cast<DeviceContext*>(client_data)) {
    if (odom_type == OdomSourceType::kOdom2ImuHigh) {
      ctx->odom_hf_stats.frame_count++;
    } else {
      ctx->odom_lf_stats.frame_count++;
    }
  }
}

// =============================================================================
// Device Management
// =============================================================================

// Helper function to convert channel to string
std::string ChannelToString(OdinDataChannel ch) {
  auto lookup = [](OdinDataChannel c) -> const char* {
    switch (c) {
      case OdinDataChannel::kRawPoint:      return "RawPoint";
      case OdinDataChannel::kSlamPoint:     return "SlamPoint";
      case OdinDataChannel::kImage0:        return "Image0";
      case OdinDataChannel::kImage1:        return "Image1";
      case OdinDataChannel::kImu:           return "IMU";
      case OdinDataChannel::kOdomLF:        return "OdomLF";
      case OdinDataChannel::kOdomHF:        return "OdomHF";
      case OdinDataChannel::kAlgoDownlink:  return "AlgoDownlink";
      case OdinDataChannel::kCalibFile:     return "CalibFile";
      case OdinDataChannel::kTelemetry:     return "Telemetry";
      default: return nullptr;
    }
  }(ch);

  if (lookup != nullptr) {
    return lookup;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "Unknown(0x%02X)", static_cast<uint8_t>(ch));
  return buf;
}

// Helper function to convert format to string
// For image channels, format is a bitmask — decode individual bits.
// For other channels, format is a fixed enum value.
std::string FormatToString(OdinDataFormat fmt) {
  uint8_t v = static_cast<uint8_t>(fmt);

  // Image formats use bitmask (0x01|0x02|0x04|0x08)
  if (v > 0 && v < 0x10) {
    std::string result;
    if (v & 0x01) result += (result.empty() ? "" : "|") + std::string("NV12");
    if (v & 0x02) result += (result.empty() ? "" : "|") + std::string("MJPEG");
    if (v & 0x04) result += (result.empty() ? "" : "|") + std::string("H264");
    if (v & 0x08) result += (result.empty() ? "" : "|") + std::string("H265");
    uint8_t known_mask = 0x0F;
    if (!result.empty() && (v & ~known_mask) == 0) return result;
    // Partial match or unknown bits — fall through to show raw value
  }

  switch (fmt) {
    case OdinDataFormat::kImageNv12:         return "NV12";
    case OdinDataFormat::kImageMjpeg:        return "MJPEG";
    case OdinDataFormat::kImageH264:         return "H264";
    case OdinDataFormat::kImageH265:         return "H265";
    case OdinDataFormat::kRawPointXyzic:     return "XYZIC";
    case OdinDataFormat::kSlamPointXyzrgba:  return "XYZRGBA";
    case OdinDataFormat::kImu6Axis:          return "6-Axis";
    case OdinDataFormat::kOdomStandard:      return "Standard";
    default: {
      char buf[32];
      snprintf(buf, sizeof(buf), "Unknown(0x%02X)", v);
      return buf;
    }
  }
}

void PrintSensorCapabilities(const std::vector<OdinSensorCapability>& capabilities) {
  std::cout << "\n  ╔════════════════════════════════════════════════════════╗\n";
  std::cout << "  ║              SENSOR CAPABILITIES                       ║\n";
  std::cout << "  ╠════════════════════════════════════════════════════════╣\n";

  for (const auto& cap : capabilities) {
    std::cout << "  ║  Channel: " << std::left << std::setw(12) << ChannelToString(cap.channel);
    std::cout << "                              ║\n";
    std::cout << "  ║  ┌──────────────────────────────────────────────────┐ ║\n";
    std::cout << "  ║  │  Mode   Width  Height   FPS   Format            │ ║\n";
    std::cout << "  ║  ├──────────────────────────────────────────────────┤ ║\n";

    int mode_idx = 0;
    for (const auto& cfg : cap.modes) {
      std::cout << "  ║  │  [" << std::setw(2) << mode_idx++ << "]  ";
      if (cfg.width > 0) {
        std::cout << std::setw(5) << cfg.width << "  ";
      } else {
        std::cout << "  -    ";
      }
      if (cfg.height > 0) {
        std::cout << std::setw(5) << cfg.height << "  ";
      } else {
        std::cout << "  -    ";
      }
      if (cfg.fps > 0) {
        std::cout << std::setw(4) << static_cast<unsigned>(cfg.fps) << "  ";
      } else {
        std::cout << " -   ";
      }
      std::cout << std::left << std::setw(16) << FormatToString(cfg.format);
      std::cout << " │ ║\n";
    }

    if (cap.modes.empty()) {
      std::cout << "  ║  │  (No configurations available)                   │ ║\n";
    }

    std::cout << "  ║  └──────────────────────────────────────────────────┘ ║\n";
  }

  if (capabilities.empty()) {
    std::cout << "  ║  (No sensor capabilities reported by device)          ║\n";
  }

  std::cout << "  ╚════════════════════════════════════════════════════════╝\n\n";
}

void ConnectAndRegister(DeviceContext* ctx) {
  std::cout << "[" << GetTimestamp() << "] Connecting to device: " << ctx->device.network.ip_address
            << " (SN: " << ctx->device.sn << ")" << std::endl;

  ctx->handle = ConnectDevice(ctx->device);
  if (ctx->handle == kInvalidDeviceHandle) {
    std::cerr << "[" << GetTimestamp() << "] Failed to connect to device: " << ctx->device.sn
              << std::endl;
    return;
  }

  ctx->connected = true;
  ctx->connect_time = std::chrono::steady_clock::now();

  // Register all callbacks
  RegisterPointCloudCallback(ctx->handle, PointCloudCallback, ctx);
  RegisterSlamCallback(ctx->handle, SlamCallback, ctx);
  RegisterCameraCallback(ctx->handle, CameraCallback, ctx);
  RegisterCameraCallback2(ctx->handle, CameraCallback2, ctx);
  RegisterImuCallback(ctx->handle, ImuCallback, ctx);
  RegisterOdomCallback(ctx->handle, OdomCallback, ctx);

  // Get and print firmware version
  std::string version;
  if (GetFirmwareVersion(ctx->handle, version) == OdinResult::kOk) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  FIRMWARE VERSION: " << version << "\n";
    std::cout << "========================================\n";
    std::cout << "\n";
  }

  // Query and print sensor capabilities
  std::cout << "[" << GetTimestamp() << "] Querying sensor capabilities..." << std::endl;
  std::vector<OdinSensorCapability> capabilities;
  auto cap_result = GetSensorCapability(ctx->handle, capabilities, {}, 2000);
  if (cap_result == OdinResult::kOk) {
    PrintSensorCapabilities(capabilities);
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to query capabilities, result="
              << static_cast<int>(cap_result) << std::endl;
  }
  
  // --- Step 1: Query current device mode ---
  OdinDeviceMode cur_device_mode = OdinDeviceMode::kUnknown;
  if (GetDeviceMode(ctx->handle, cur_device_mode) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Current device mode: "
              << static_cast<int>(cur_device_mode) << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to get device mode" << std::endl;
  }
  
  // --- Step 2: Enter standby mode for configuration ---
  if (SetDeviceMode(ctx->handle, OdinDeviceMode::kStandby) != OdinResult::kOk) {
    std::cerr << "[" << GetTimestamp() << "] Failed to set STANDBY mode" << std::endl;
  }
  
  // --- Step 3: Query current LiDAR mode ---
  OdinSensorMode cur_lidar_mode = OdinSensorMode::kHighPeak;
  if (GetSensorMode(ctx->handle, cur_lidar_mode) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Current LiDAR mode: "
              << static_cast<int>(cur_lidar_mode) << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to get LiDAR mode" << std::endl;
  }
  
  // --- Step 4: Configure LiDAR mode (HighPeak) ---
  constexpr OdinSensorMode kDesiredLiDARMode = OdinSensorMode::kHighPeak;
  if (SetSensorMode(ctx->handle, kDesiredLiDARMode) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] LiDAR mode set to Adaptive" << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to set LiDAR mode" << std::endl;
  }
  
  // --- Step 5: Verify LiDAR mode ---
  OdinSensorMode verified_lidar_mode = OdinSensorMode::kHighPeak;
  if (GetSensorMode(ctx->handle, verified_lidar_mode) == OdinResult::kOk) {
    const bool ok = (verified_lidar_mode == kDesiredLiDARMode);
    std::cout << "[" << GetTimestamp() << "] LiDAR mode verify: "
              << (ok ? "OK" : "MISMATCH")
              << " (mode=" << static_cast<int>(verified_lidar_mode) << ")" << std::endl;
  }
  
  // --- Step 6: Query current filter params ---
  OdinLidarFilterParams filter_params;
  if (GetFilterParam(ctx->handle, filter_params) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Current filter params:"
              << " confidence=" << (filter_params.has_confidence ? "yes" : "no")
              << "(threshold=" << static_cast<int>(filter_params.confidence.confidence_threshold) << ")"
              << " adaptive=" << (filter_params.has_adaptive ? "yes" : "no")
              << "(cdf=" << filter_params.adaptive.cdf_percentile
              << " outdoor=" << filter_params.adaptive.outdoor_threshold << "mm)"
              << " range=" << (filter_params.has_range ? "yes" : "no")
              << "(min_depth=" << filter_params.range.min_depth << "mm)"
              << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to get filter params" << std::endl;
  }

  // --- Step 7: Set confidence threshold (0 = no filtering) ---
  OdinLidarFilterParams desired_params;
  desired_params.has_confidence = true;
  desired_params.confidence = OdinConfidenceFilterParam(0);
  if (SetFilterParam(ctx->handle, desired_params) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Filter param set: confidence_threshold=0" << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp() << "] Failed to set filter param" << std::endl;
  }

  // --- Step 8: Verify filter params ---
  OdinLidarFilterParams verified_params;
  if (GetFilterParam(ctx->handle, verified_params) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Filter params verified:"
              << " confidence(threshold="
              << static_cast<int>(verified_params.confidence.confidence_threshold) << ")"
              << std::endl;
  }
  
  // --- Step 9: Start all data streams with UDP transport ---
  std::cout << "[" << GetTimestamp() << "] Starting data streams..." << std::endl;
  
  auto start_stream = [&](OdinDataChannel channel, const char* name) {
    auto result = StartStream(ctx->handle, channel, OdinTransportMode::kUdp);
    if (result == OdinResult::kOk) {
      std::cout << "  [OK] " << name << " stream started" << std::endl;
    } else {
      std::cerr << "  [FAIL] " << name << " stream failed, result="
                << static_cast<int>(result) << std::endl;
    }
  };
  
  start_stream(OdinDataChannel::kRawPoint,  "RawPoint");
  start_stream(OdinDataChannel::kSlamPoint, "SlamPoint");
  start_stream(OdinDataChannel::kImage0,    "Image0");
  start_stream(OdinDataChannel::kImage1,    "Image1");
  start_stream(OdinDataChannel::kImu,       "IMU");
  start_stream(OdinDataChannel::kOdomLF,    "OdomLF");
  start_stream(OdinDataChannel::kOdomHF,    "OdomHF");
  
  // --- Step 7: Verify per-channel subscription state ---
  auto query_channel = [&](OdinDataChannel channel, const char* name) {
    OdinChannelConfigInfo ch_info;
    if (GetChannelConfig(ctx->handle, channel, ch_info) == OdinResult::kOk) {
      std::cout << "  [" << name << "] subscribed=" << (ch_info.subscribed ? "yes" : "no")
                << " port=" << ch_info.dst_port
                << " transport=" << (ch_info.transport_mode == OdinTransportMode::kTcp ? "TCP" : "UDP")
                << std::endl;
    } else {
      std::cerr << "  [" << name << "] GetChannelConfig failed" << std::endl;
    }
  };
  query_channel(OdinDataChannel::kRawPoint,  "RawPoint");
  query_channel(OdinDataChannel::kSlamPoint, "SlamPoint");
  query_channel(OdinDataChannel::kImage0,    "Image0");
  query_channel(OdinDataChannel::kImage1,    "Image1");
  query_channel(OdinDataChannel::kImu,       "IMU");
  query_channel(OdinDataChannel::kOdomLF,    "OdomLF");
  query_channel(OdinDataChannel::kOdomHF,    "OdomHF");
  
  // --- Step 8: Switch to normal mode to begin streaming ---
  if (SetDeviceMode(ctx->handle, OdinDeviceMode::kNormal) == OdinResult::kOk) {
    std::cout << "[" << GetTimestamp() << "] Device " << ctx->device.sn
              << " set to NORMAL mode, streaming started" << std::endl;
  } else {
    std::cerr << "[" << GetTimestamp()
              << "] Failed to set NORMAL mode for device: " << ctx->device.sn << std::endl;
  }
}

void DisconnectAndRelease(DeviceContext* ctx) {
  if (!ctx->connected) return;

  std::cout << "[" << GetTimestamp() << "] Disconnecting device: " << ctx->device.network.ip_address
            << " (SN: " << ctx->device.sn << ")" << std::endl;

  if (ctx->handle != kInvalidDeviceHandle) {
    // Set to standby mode before closing streams
    SetDeviceMode(ctx->handle, OdinDeviceMode::kStandby);

    // Close all data streams
    CloseStream(ctx->handle, OdinDataChannel::kRawPoint);
    CloseStream(ctx->handle, OdinDataChannel::kSlamPoint);
    CloseStream(ctx->handle, OdinDataChannel::kImage0);
    CloseStream(ctx->handle, OdinDataChannel::kImage1);
    CloseStream(ctx->handle, OdinDataChannel::kImu);
    CloseStream(ctx->handle, OdinDataChannel::kOdomLF);
    CloseStream(ctx->handle, OdinDataChannel::kOdomHF);

    DisconnectDevice(ctx->handle);
    ctx->handle = kInvalidDeviceHandle;
  }

  // Reset all stats
  ctx->raw_stats.Reset();
  ctx->slam_stats.Reset();
  ctx->image_stats.Reset();
  ctx->imu_stats.Reset();
  ctx->odom_lf_stats.Reset();
  ctx->odom_hf_stats.Reset();
  ctx->image2_stats.Reset();

  ctx->connected = false;

  std::cout << "[" << GetTimestamp() << "] Device " << ctx->device.sn << " resources released"
            << std::endl;
}

// =============================================================================
// Hotplug Event Handlers
// =============================================================================

void OnDeviceAttach(const DiscoveredDevice& device) {
  std::string key = device.sn.empty() ? device.network.ip_address : device.sn;

  std::lock_guard<std::mutex> lock(g_devices_mutex);

  std::cout << "\n[" << GetTimestamp() << "] === DEVICE ARRIVED ===" << std::endl;
  std::cout << "  IP: " << device.network.ip_address << std::endl;
  std::cout << "  SN: " << device.sn << std::endl;
  std::cout << "  Model: " << device.model << std::endl;

  if (g_devices.find(key) != g_devices.end()) {
    // Device already exists (shouldn't happen since we erase on removal)
    return;
  }
  auto ctx = std::unique_ptr<DeviceContext>(new DeviceContext());
  ctx->device = device;
  ConnectAndRegister(ctx.get());
  g_devices[key] = std::move(ctx);

  std::cout << "[" << GetTimestamp() << "] Total devices online: " << g_devices.size() << std::endl;
}

void OnDeviceDetach(const DiscoveredDevice& device) {
  std::string key = device.sn.empty() ? device.network.ip_address : device.sn;

  std::cout << "\n[" << GetTimestamp() << "] === DEVICE REMOVED ===" << std::endl;
  std::cout << "  IP: " << device.network.ip_address << std::endl;
  std::cout << "  SN: " << device.sn << std::endl;

  // Move device out of map first, then disconnect outside lock to avoid deadlock
  // (callbacks hold g_devices_mutex, Stop() waits for callback thread to exit)
  std::unique_ptr<DeviceContext> ctx_to_disconnect;
  size_t remaining_devices = 0;
  {
    std::lock_guard<std::mutex> lock(g_devices_mutex);
    auto it = g_devices.find(key);
    if (it != g_devices.end()) {
      ctx_to_disconnect = std::move(it->second);
      g_devices.erase(it);
    }
    remaining_devices = g_devices.size();
  }
  // Now disconnect without holding the lock
  if (ctx_to_disconnect) {
    DisconnectAndRelease(ctx_to_disconnect.get());
  }

  std::cout << "[" << GetTimestamp() << "] Total devices online: " << remaining_devices
            << std::endl;
}

// =============================================================================
// Statistics Display
// =============================================================================

void PrintStatistics() {
  std::lock_guard<std::mutex> lock(g_devices_mutex);

  if (g_devices.empty()) {
    return;
  }

  // Update FPS for all devices
  for (auto& kv : g_devices) {
    auto* ctx = kv.second.get();
    if (!ctx->connected) continue;

    UpdateFps(ctx->raw_stats);
    UpdateFps(ctx->slam_stats);
    UpdateFps(ctx->image_stats);
    UpdateFps(ctx->imu_stats);
    UpdateFps(ctx->odom_lf_stats);
    UpdateFps(ctx->odom_hf_stats);
    UpdateFps(ctx->image2_stats);
  }

  // Print header
  std::cout << "\n========== Frame Rate Statistics [" << GetTimestamp() << "] ==========\n";

  for (auto& kv : g_devices) {
    auto* ctx = kv.second.get();
    if (!ctx->connected) continue;

    auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - ctx->connect_time);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Device: " << ctx->device.sn << " (" << ctx->device.network.ip_address << ")\n";
    std::cout << "  Uptime: " << uptime.count() << "s\n";
    std::cout << "  RAW:    " << std::setw(6) << ctx->raw_stats.fps << " fps | ";
    std::cout << "SLAM:   " << std::setw(6) << ctx->slam_stats.fps << " fps | ";
    std::cout << "IMAGE:  " << std::setw(6) << ctx->image_stats.fps << " fps\n";
    std::cout << "  IMU:    " << std::setw(6) << ctx->imu_stats.fps << " fps | ";
    std::cout << "ODOM_LF:" << std::setw(6) << ctx->odom_lf_stats.fps << " fps | ";
    std::cout << "ODOM_HF:" << std::setw(6) << ctx->odom_hf_stats.fps << " fps\n";
    std::cout << "  IMAGE2: " << std::setw(6) << ctx->image2_stats.fps << " fps\n";
  }

  std::cout << "============================================================\n";
}

// =============================================================================
// Main
// =============================================================================

void PrintUsage(const char* exe) {
  std::cout << "Usage: " << exe << "\n"
            << "\n"
            << "Multi-device hotplug demo with frame rate statistics.\n"
            << "The program will automatically detect devices as they come online/offline.\n"
            << "\n"
            << "Press Ctrl+C to exit.\n";
}

int main(int argc, char* argv[]) {
  if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help")) {
    PrintUsage(argv[0]);
    return 0;
  }

  // Setup signal handler
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  // Initialize logger
  log_config(LOG_LEVEL_INFO, printf);

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "\n";
  std::cout << "Multi-Device Hotplug Demo\n";
  std::cout << "Starting hotplug listener...\n";
  std::cout << "Waiting for devices to come online...\n";
  std::cout << "Press Ctrl+C to exit.\n\n";

  // Start hotplug listener
  HotplugCallbacks callbacks;
  callbacks.on_attach = OnDeviceAttach;
  callbacks.on_detach = OnDeviceDetach;
  if (!StartHotplugListener(callbacks, true)) {
    std::cerr << "Failed to start hotplug listener!" << std::endl;
    return 1;
  }

  // Main loop - print statistics periodically
  auto last_print = std::chrono::steady_clock::now();
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print);
    if (elapsed.count() >= 2) {  // Print every 2 seconds
      PrintStatistics();
      last_print = now;
    }
  }

  std::cout << "\nShutting down...\n";

  // Stop hotplug listener
  StopHotplugListener();

  // Disconnect all devices - collect first, then disconnect outside lock to avoid deadlock
  // (callbacks hold g_devices_mutex, Stop() waits for callback thread to exit)
  std::vector<std::unique_ptr<DeviceContext>> devices_to_disconnect;
  {
    std::lock_guard<std::mutex> lock(g_devices_mutex);
    for (auto& kv : g_devices) {
      devices_to_disconnect.push_back(std::move(kv.second));
    }
    g_devices.clear();
  }
  // Now disconnect without holding the lock
  for (auto& ctx : devices_to_disconnect) {
    DisconnectAndRelease(ctx.get());
  }

  std::cout << "Cleanup complete. Goodbye!\n";
  return 0;
}
