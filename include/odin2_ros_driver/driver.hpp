#pragma once
#include <memory>

#include "decoder.hpp"
#include "sdk.hpp"
namespace odin_driver {
class DeviceDriver {
 public:
  DeviceDriver(ros::NodeHandle node, DeviceSettings settings, std::shared_ptr<SdkFacade> sdk);
  ~DeviceDriver();
  void start();
  void stop();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
int runDriver(int argc, char** argv, std::shared_ptr<SdkFacade> sdk);
}  // namespace odin_driver
