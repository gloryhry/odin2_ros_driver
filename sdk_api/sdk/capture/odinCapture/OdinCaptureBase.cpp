#include "OdinCaptureBase.hpp"

#include "TcpTransport.hpp"
#include "UdpTransport.hpp"
#include "logger.h"

namespace odin {
namespace sdk {

NetworkCapture::NetworkCapture(OdinTransportMode transport)
    : transport_mode_(transport) {}

NetworkCapture::~NetworkCapture() {
  Stop();
}

bool NetworkCapture::Start(const ITransportAddress& address, RawDataCallback callback) {
  if (running_) {
    LOG_WARN("NetworkCapture::Start: Already running\n");
    return true;
  }

  if (!callback) {
    LOG_ERROR("NetworkCapture::Start: callback is null\n");
    return false;
  }

  callback_ = callback;

  // Create transport based on the requested transport mode. New variants
  // (e.g. KCP, USB-bulk) should add a case here without leaking the choice
  // back up to the device layer.
  switch (transport_mode_) {
    case OdinTransportMode::kTcp:
      transport_.reset(new TcpTransport());
      break;
    case OdinTransportMode::kUdp:
    default:
      transport_.reset(new UdpTransport());
      break;
  }

  if (!transport_->Open(address)) {
    LOG_ERROR("NetworkCapture::Start: Open failed\n");
    return false;
  }

  // Set receive callback - pass raw data directly to user callback
  transport_->SetReceiveCallback(
      [this](const uint8_t* data, size_t len, const ITransportAddress& addr) {
        if (callback_) {
          callback_(data, len, addr);
        }
      });

  if (!transport_->StartReceiving()) {
    LOG_ERROR("NetworkCapture::Start: StartReceiving failed\n");
    transport_->Close();
    return false;
  }

  running_ = true;
  const char* mode_name =
      (transport_mode_ == OdinTransportMode::kTcp) ? "TCP" : "UDP";
  LOG_INFO("NetworkCapture::Start: %s, %s\n",
           address.ToString().c_str(), mode_name);
  return true;
}

void NetworkCapture::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;

  if (transport_) {
    transport_->StopReceiving();
    transport_->Close();
    transport_.reset();
  }

  callback_ = nullptr;
  LOG_DEBUG("NetworkCapture::Stop\n");
}

bool NetworkCapture::IsRunning() const {
  return running_;
}

}  // namespace sdk
}  // namespace odin
