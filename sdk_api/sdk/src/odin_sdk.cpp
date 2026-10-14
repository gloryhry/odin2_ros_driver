#include "OdinSdkImpl.h"
#include "../utils/network_utils.h"
#include "../hotplugListener/HotplugListenerFactory.h"

#include <memory>
#include <mutex>

namespace {
// Global hotplug listener instance (using base class pointer for polymorphism)
std::unique_ptr<odin::sdk::IHotplugListener> g_hotplug_listener;
std::mutex g_hotplug_mutex;

// User-registered hotplug callbacks (on_attach/on_detach).
odin::sdk::HotplugCallbacks g_hotplug_callbacks;
}  // namespace

namespace odin {
namespace sdk {

// =============================================================================
// Public API Functions
// =============================================================================

OdinDeviceHandle ConnectDevice(const DiscoveredDevice &device) {
  // Register the heartbeat-success keep-alive bridge exactly once. Every
  // successful heartbeat on a connected device routes through here and
  // refreshes the hotplug listener's last_seen for that device, so connected
  // devices never get timed out by the discovery-timeout path even if they
  // stop answering discovery probes while streaming. Offline detection is
  // therefore unified to a single time-based path inside the listener.
  static std::once_flag once;
  std::call_once(once, []() {
    OdinSdkImpl::Instance().SetHeartbeatSuccessNotifyCallback(
        [](const DiscoveredDevice& dev) {
          std::lock_guard<std::mutex> lock(g_hotplug_mutex);
          if (g_hotplug_listener) {
            g_hotplug_listener->RefreshDeviceAlive(dev);
          }
        });
  });

  return OdinSdkImpl::Instance().ConnectDevice(device);
}

void DisconnectDevice(OdinDeviceHandle handle) { OdinSdkImpl::Instance().DisconnectDevice(handle); }

void RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb,
                                void *client_data) {
  OdinSdkImpl::Instance().RegisterPointCloudCallback(handle, cb, client_data);
}

void RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterSlamCallback(handle, cb, client_data);
}

void RegisterCameraCallback(OdinDeviceHandle handle, OdinCameraCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterCameraCallback(handle, cb, client_data);
}

void RegisterCameraCallback2(OdinDeviceHandle handle, OdinCameraCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterCameraCallback2(handle, cb, client_data);
}

void RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterImuCallback(handle, cb, client_data);
}

void RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterOdomCallback(handle, cb, client_data);
}

void RegisterTelemetryCallback(OdinDeviceHandle handle, OdinTelemetryCallback cb,
                               void *client_data) {
  OdinSdkImpl::Instance().RegisterTelemetryCallback(handle, cb, client_data);
}

OdinResult StartTelemetry(OdinDeviceHandle handle) {
  return OdinSdkImpl::Instance().StartTelemetry(handle);
}

void RegisterTfCallback(OdinDeviceHandle handle, OdinTfCallback cb, void *client_data) {
  OdinSdkImpl::Instance().RegisterTfCallback(handle, cb, client_data);
}

void EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled, uint32_t max_frame_lag) {
  OdinSdkImpl::Instance().EnableSlamOdomSyncForDevice(handle, enabled, max_frame_lag);
}

OdinResult SetDeviceMode(OdinDeviceHandle device, OdinDeviceMode mode,
                            uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetDeviceMode(device, mode, timeout_ms);
}

OdinResult GetFirmwareVersion(OdinDeviceHandle device, std::string &version,
                              uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetFirmwareVersion(device, version, timeout_ms);
}

OdinResult SetSensorMode(OdinDeviceHandle device, OdinSensorMode mode, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetSensorMode(device, mode, timeout_ms);
}

OdinResult SendAlgoData(OdinDeviceHandle device, const uint8_t* data, size_t len) {
  return OdinSdkImpl::Instance().SendAlgoData(device, data, len);
}

OdinResult GetDeviceMode(OdinDeviceHandle device, OdinDeviceMode& mode, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetDeviceMode(device, mode, timeout_ms);
}

OdinResult GetSensorMode(OdinDeviceHandle device, OdinSensorMode& mode, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetSensorMode(device, mode, timeout_ms);
}

OdinResult StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                       OdinTransportMode transport, const OdinStreamCfg* mode,
                       uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().StartStream(device, channel, transport, mode, timeout_ms);
}

OdinResult GetSensorCapability(OdinDeviceHandle device,
                               std::vector<OdinSensorCapability>& capabilities,
                               const std::vector<OdinDataChannel>& channels,
                               uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetSensorCapability(device, capabilities, channels, timeout_ms);
}

OdinResult CloseStream(OdinDeviceHandle device, OdinDataChannel channel, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().CloseStream(device, channel, timeout_ms);
}

OdinResult GetChannelConfig(OdinDeviceHandle device, OdinDataChannel channel,
                           OdinChannelConfigInfo& info, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetChannelConfig(device, channel, info, timeout_ms);
}

OdinResult SetFilterParam(OdinDeviceHandle device, const OdinLidarFilterParams& params,
                          uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetFilterParam(device, params, timeout_ms);
}

OdinResult GetFilterParam(OdinDeviceHandle device, OdinLidarFilterParams& params,
                          uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetFilterParam(device, params, timeout_ms);
}

OdinResult RestoreDefaultConfig(OdinDeviceHandle device, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().RestoreDefaultConfig(device, timeout_ms);
}

OdinResult SetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode mode,
                              uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetDefaultSyncMode(device, mode, timeout_ms);
}

OdinResult GetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode& mode,
                              uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetDefaultSyncMode(device, mode, timeout_ms);
}

OdinResult GetPtpSyncStatus(OdinDeviceHandle device, OdinPtpSyncStatus& status,
                            uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetPtpSyncStatus(device, status, timeout_ms);
}

OdinResult SetAdaptiveParam(OdinDeviceHandle device, const OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetAdaptiveParam(device, param, timeout_ms);
}

OdinResult GetAdaptiveParam(OdinDeviceHandle device, OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetAdaptiveParam(device, param, timeout_ms);
}

OdinResult ResetInitialPose(OdinDeviceHandle device, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().ResetInitialPose(device, timeout_ms);
}

OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetNetworkAttribute(device, attr, timeout_ms);
}

OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().SetNetworkAttribute(device, attr, timeout_ms);
}

OdinResult Reboot(OdinDeviceHandle device, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().Reboot(device, timeout_ms);
}

std::vector<std::string> GetLocalIPAddresses() { return network::GetLocalIPAddresses(); }

bool DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().DiscoverDevices(devices, timeout_ms);
}

OdinResult GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb, OdinFileType type,
                         std::string save_path, uint32_t timeout_ms) {
  return OdinSdkImpl::Instance().GetDeviceFile(device, cb, type, save_path, timeout_ms);
}

OdinResult SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                            OdinFileType type, std::string file_path) {
  return OdinSdkImpl::Instance().SendFileToDevice(device, cb, type, file_path);
}

std::string GetSdkVersion() { return ODIN_SDK_VERSION_STRING; }

// =============================================================================
// Hotplug Listener API
// =============================================================================

bool StartHotplugListener(const HotplugCallbacks &callbacks, bool enumerate_existing) {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener && g_hotplug_listener->IsRunning()) {
    return false;  // Already running
  }
  
  // Store callbacks for heartbeat failure notification
  // Note: The heartbeat failure callback is registered in ConnectDevice() using std::once_flag
  g_hotplug_callbacks = callbacks;
  
  g_hotplug_listener = HotplugListenerFactory::CreateDefault();
  return g_hotplug_listener->Start(callbacks, enumerate_existing);
}

void StopHotplugListener() {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->Stop();
    g_hotplug_listener.reset();
  }
}

bool IsHotplugListenerRunning() {
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  return g_hotplug_listener && g_hotplug_listener->IsRunning();
}

void SetHotplugPollingInterval(uint32_t interval_ms) {
  // Hotplug discovery cadence only. Heartbeat tuning is a per-device protocol
  // detail owned by the device layer and is intentionally NOT coupled to this.
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->SetPollingInterval(interval_ms);
  }
}

void SetHotplugOfflineTimeout(uint32_t timeout_ms) {
  // Hotplug offline threshold only; see note above.
  std::lock_guard<std::mutex> lock(g_hotplug_mutex);
  if (g_hotplug_listener) {
    g_hotplug_listener->SetOfflineTimeout(timeout_ms);
  }
}

}  // namespace sdk
}  // namespace odin
