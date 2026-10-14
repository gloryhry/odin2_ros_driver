#include "odin2_ros_driver/sdk.hpp"

#include <logger/logger.h>

#include <cstdio>
#include <map>
#include <stdexcept>

namespace odin_driver {
namespace {
std::mutex progress_mutex;
std::map<sdk::OdinDeviceHandle, std::function<void(float)>> progress_callbacks;
void progress(sdk::OdinDeviceHandle handle, float value, void*) {
  std::function<void(float)> cb;
  {
    std::lock_guard<std::mutex> lock(progress_mutex);
    auto it = progress_callbacks.find(handle);
    if (it != progress_callbacks.end())
      cb = it->second;
  }
  if (cb)
    cb(value);
}
class LiveSdk final : public SdkFacade {
  struct Context {
    PacketCallback callback;
  };
  std::map<sdk::OdinDeviceHandle, std::unique_ptr<Context>> contexts_;
  std::mutex mutex_;
  template <class T>
  static void emit(const T& p, void* user, Channel c) {
    auto* ctx = static_cast<Context*>(user);
    try {
      if (ctx && ctx->callback)
        ctx->callback(c, Packet(p));
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM_THROTTLE(5, "SDK callback copy failed: " << e.what());
    } catch (...) {
      ROS_ERROR_THROTTLE(5, "SDK callback failed with an unknown exception");
    }
  }

 public:
  LiveSdk() {
    // The vendor logger's function pointer is null until explicitly configured.
    // Initialize before any API call, including failed connection/discovery logs.
    static std::once_flag logger_once;
    std::call_once(logger_once, [] {
      if (log_config(LOG_LEVEL_WARN, std::printf) != 0)
        throw std::runtime_error("invalid SDK LOG_LEVEL environment variable");
    });
  }
  ~LiveSdk() override {
    // Managers must stop before the facade; this is also safe for partial startup.
    while (!contexts_.empty())
      disconnect(contexts_.begin()->first);
  }
  std::vector<sdk::DiscoveredDevice> discover(uint32_t ms) override {
    std::vector<sdk::DiscoveredDevice> list;
    sdk::DiscoverDevices(list, ms);
    return list;
  }
  sdk::OdinDeviceHandle connect(const sdk::DiscoveredDevice& d) override {
    return sdk::ConnectDevice(d);
  }
  void disconnect(sdk::OdinDeviceHandle h) override {
    if (!h)
      return;
    sdk::DisconnectDevice(h);  // joins SDK receive threads before freeing callback contexts
    std::lock_guard<std::mutex> lock(mutex_);
    contexts_.erase(h);
  }
  void callbacks(sdk::OdinDeviceHandle h, PacketCallback cb) override {
    if (!h)
      throw std::invalid_argument("SDK callbacks require an explicit device handle");
    auto context = std::make_unique<Context>();
    context->callback = std::move(cb);
    auto* p = context.get();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      contexts_[h] = std::move(context);
    }
    sdk::RegisterPointCloudCallback(
        h, [](const auto& v, void* u) { emit(v, u, Channel::kRawPoint); }, p);
    sdk::RegisterSlamCallback(
        h, [](const auto& v, void* u) { emit(v, u, Channel::kSlamPoint); }, p);
    sdk::RegisterCameraCallback(h, [](const auto& v, void* u) { emit(v, u, Channel::kImage0); }, p);
    sdk::RegisterCameraCallback2(
        h, [](const auto& v, void* u) { emit(v, u, Channel::kImage1); }, p);
    sdk::RegisterImuCallback(h, [](const auto& v, void* u) { emit(v, u, Channel::kImu); }, p);
    sdk::RegisterOdomCallback(
        h,
        [](const auto& v, sdk::OdomSourceType type, void* u) {
          emit(v, u,
               type == sdk::OdomSourceType::kOdom2ImuHigh ? Channel::kOdomHF : Channel::kOdomLF);
        },
        p);
    sdk::RegisterTfCallback(h, [](const auto& v, void* u) { emit(v, u, Channel::kTf); }, p);
    sdk::RegisterTelemetryCallback(
        h, [](const auto& v, void* u) { emit(v, u, Channel::kTelemetry); }, p);
  }
  Reply start(sdk::OdinDeviceHandle h, Channel c, const StreamSettings& s,
              uint32_t timeout_ms) override {
    Reply r;
    if (!h) {
      r.code = sdk::OdinResult::kNotInitialized;
      return r;
    }
    r.code = c == Channel::kTelemetry
                 ? sdk::StartTelemetry(h)
                 : sdk::StartStream(h, c, s.transport, isCamera(c) ? &s.mode : nullptr, timeout_ms);
    return r;
  }
  Reply close(sdk::OdinDeviceHandle h, Channel c) override {
    Reply r;
    if (!h) {
      r.code = sdk::OdinResult::kNotInitialized;
      return r;
    }
    r.code = sdk::CloseStream(h, c);
    return r;
  }
  void sync(sdk::OdinDeviceHandle h, bool on, uint32_t lag) override {
    if (h)
      sdk::EnableSlamOdomSyncForDevice(h, on, lag);
  }
  Reply execute(sdk::OdinDeviceHandle h, const Command& c) override {
    Reply r;
    if (!h) {
      r.code = sdk::OdinResult::kNotInitialized;
      r.message = "explicit device handle is required; SDK default device is never used";
      return r;
    }
    auto setResult = [&](sdk::OdinResult v) {
      if (r.ok())
        r.code = v;
    };
    switch (c.kind) {
      case CommandKind::Info:
        setResult(sdk::GetFirmwareVersion(h, r.firmware, c.timeout_ms));
        setResult(sdk::GetNetworkAttribute(h, r.network, c.timeout_ms));
        setResult(sdk::GetSensorCapability(h, r.capabilities, {}, c.timeout_ms));
        break;
      case CommandKind::Capabilities:
        r.code = sdk::GetSensorCapability(h, r.capabilities, {}, c.timeout_ms);
        break;
      case CommandKind::ChannelInfo:
        r.code = sdk::GetChannelConfig(h, c.channel, r.channel, c.timeout_ms);
        break;
      case CommandKind::GetDeviceMode: {
        sdk::OdinDeviceMode m;
        r.code = sdk::GetDeviceMode(h, m, c.timeout_ms);
        if (r.ok())
          r.mode = static_cast<uint8_t>(m);
        break;
      }
      case CommandKind::SetDeviceMode:
        r.code = sdk::SetDeviceMode(h, static_cast<sdk::OdinDeviceMode>(c.mode), c.timeout_ms);
        break;
      case CommandKind::GetSensorMode: {
        sdk::OdinSensorMode m;
        r.code = sdk::GetSensorMode(h, m, c.timeout_ms);
        if (r.ok())
          r.mode = static_cast<uint8_t>(m);
        break;
      }
      case CommandKind::SetSensorMode:
        r.code = sdk::SetSensorMode(h, static_cast<sdk::OdinSensorMode>(c.mode), c.timeout_ms);
        break;
      case CommandKind::GetFilters:
        r.code = sdk::GetFilterParam(h, r.filters, c.timeout_ms);
        break;
      case CommandKind::SetFilters:
        r.code = sdk::SetFilterParam(h, c.filters, c.timeout_ms);
        break;
      case CommandKind::GetAdaptive:
        r.code = sdk::GetAdaptiveParam(h, r.adaptive, c.timeout_ms);
        break;
      case CommandKind::SetAdaptive:
        r.code = sdk::SetAdaptiveParam(h, c.adaptive, c.timeout_ms);
        break;
      case CommandKind::GetSyncMode: {
        sdk::OdinTimeSyncMode m;
        r.code = sdk::GetDefaultSyncMode(h, m, c.timeout_ms);
        if (r.ok())
          r.mode = static_cast<uint8_t>(m);
        break;
      }
      case CommandKind::SetSyncMode:
        r.code =
            sdk::SetDefaultSyncMode(h, static_cast<sdk::OdinTimeSyncMode>(c.mode), c.timeout_ms);
        break;
      case CommandKind::Ptp:
        r.code = sdk::GetPtpSyncStatus(h, r.ptp, c.timeout_ms);
        break;
      case CommandKind::GetNetwork:
        r.code = sdk::GetNetworkAttribute(h, r.network, c.timeout_ms);
        break;
      case CommandKind::SetNetwork:
        r.code = sdk::SetNetworkAttribute(h, c.network, c.timeout_ms);
        break;
      case CommandKind::ResetPose:
        r.code = sdk::ResetInitialPose(h, c.timeout_ms);
        break;
      case CommandKind::Restore:
        r.code = sdk::RestoreDefaultConfig(h, c.timeout_ms);
        break;
      case CommandKind::Reboot:
        r.code = sdk::Reboot(h, c.timeout_ms);
        break;
      case CommandKind::Algo:
        r.code = sdk::SendAlgoData(h, c.bytes.data(), c.bytes.size());
        break;
      case CommandKind::File: {
        {
          std::lock_guard<std::mutex> lock(progress_mutex);
          progress_callbacks[h] = c.progress;
        }
        try {
          r.code = c.upload ? sdk::SendFileToDevice(h, progress, c.file_type, c.path)
                            : sdk::GetDeviceFile(h, progress, c.file_type, c.path, c.timeout_ms);
        } catch (...) {
          std::lock_guard<std::mutex> lock(progress_mutex);
          progress_callbacks.erase(h);
          throw;
        }
        {
          std::lock_guard<std::mutex> lock(progress_mutex);
          progress_callbacks.erase(h);
        }
        break;
      }
    }
    if (!r.ok())
      r.message = "SDK result " + std::to_string(static_cast<int>(r.code));
    return r;
  }
};
}  // namespace
std::shared_ptr<SdkFacade> makeLiveSdk() {
  return std::make_shared<LiveSdk>();
}
}  // namespace odin_driver
