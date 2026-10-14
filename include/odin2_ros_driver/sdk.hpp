#pragma once
#include <functional>
#include <memory>

#include "core.hpp"

namespace odin_driver {
enum class CommandKind {
  Info,
  Capabilities,
  ChannelInfo,
  GetDeviceMode,
  SetDeviceMode,
  GetSensorMode,
  SetSensorMode,
  GetFilters,
  SetFilters,
  GetAdaptive,
  SetAdaptive,
  GetSyncMode,
  SetSyncMode,
  Ptp,
  GetNetwork,
  SetNetwork,
  ResetPose,
  Restore,
  Reboot,
  Algo,
  File
};
struct Command {
  CommandKind kind = CommandKind::Info;
  Channel channel = Channel::kRawPoint;
  uint8_t mode = 0;
  sdk::OdinLidarFilterParams filters;
  sdk::OdinAdaptiveFilterParam adaptive;
  sdk::NetworkAttribute network;
  std::vector<uint8_t> bytes;
  bool upload = false;
  sdk::OdinFileType file_type = sdk::OdinFileType::kCalibrationFile;
  std::string path;
  uint32_t timeout_ms = 1000;
  std::function<void(float)> progress;
};
struct Reply {
  sdk::OdinResult code = sdk::OdinResult::kOk;
  std::string message, firmware;
  uint8_t mode = 0;
  sdk::OdinLidarFilterParams filters;
  sdk::OdinAdaptiveFilterParam adaptive;
  sdk::NetworkAttribute network;
  sdk::OdinPtpSyncStatus ptp;
  sdk::OdinChannelConfigInfo channel;
  std::vector<sdk::OdinSensorCapability> capabilities;
  bool ok() const {
    return code == sdk::OdinResult::kOk;
  }
};
using PacketCallback = std::function<void(Channel, Packet)>;
class SdkFacade {
 public:
  virtual ~SdkFacade() = default;
  virtual std::vector<sdk::DiscoveredDevice> discover(uint32_t timeout_ms) = 0;
  virtual sdk::OdinDeviceHandle connect(const sdk::DiscoveredDevice&) = 0;
  virtual void disconnect(sdk::OdinDeviceHandle) = 0;
  virtual void callbacks(sdk::OdinDeviceHandle, PacketCallback) = 0;
  virtual Reply start(sdk::OdinDeviceHandle, Channel, const StreamSettings&,
                      uint32_t timeout_ms) = 0;
  virtual Reply close(sdk::OdinDeviceHandle, Channel) = 0;
  virtual void sync(sdk::OdinDeviceHandle, bool, uint32_t) = 0;
  virtual Reply execute(sdk::OdinDeviceHandle, const Command&) = 0;
};
std::shared_ptr<SdkFacade> makeLiveSdk();
}  // namespace odin_driver
