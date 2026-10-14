#pragma once

#include <atomic>
#include <functional>
#include <memory>

#include "../ICapture.hpp"
#include "ITransport.hpp"
#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

/**
 * @brief Network-based data capture implementation
 * 
 * Implements ICapture for network transports (UDP/TCP).
 * Only handles transport layer - protocol parsing is done by upper layers.
 * 
 * This class is protocol-agnostic and can be used for any network-based
 * data capture. Raw data is passed to the callback without any parsing.
 */
class NetworkCapture : public ICapture {
 public:
  /**
   * @brief Constructor
   * @param transport Transport mode driving capture instantiation
   *                  (e.g. UDP / TCP; future variants extend this enum).
   */
  explicit NetworkCapture(OdinTransportMode transport = OdinTransportMode::kUdp);
  ~NetworkCapture() override;

  // ICapture interface
  bool Start(const ITransportAddress& address, RawDataCallback callback) override;
  void Stop() override;
  bool IsRunning() const override;

 private:
  std::unique_ptr<ITransport> transport_;
  std::atomic<bool> running_{false};
  RawDataCallback callback_;
  OdinTransportMode transport_mode_;
};

}  // namespace sdk
}  // namespace odin
