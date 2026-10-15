#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "OdinProtocol.hpp"
#include "odin2_ros_driver/sdk.hpp"

namespace {
using namespace odin_driver;

int listener(uint16_t port) {
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) || listen(fd, 4)) {
    close(fd);
    throw std::runtime_error("cannot open mock firmware port");
  }
  return fd;
}

// Uses the real SDK command/data sockets. Like firmware 3.1.3, TCP subscribe
// connects immediately and returns failure if the host is not listening.
class Firmware {
 public:
  std::atomic<bool> reject_camera{false};
  std::atomic<uint16_t> camera_port{0};
  std::atomic<unsigned> camera_connections{0};
  std::atomic<unsigned> adaptive_sets{0}, adaptive_gets{0}, aggregate_commands{0};
  std::atomic<uint16_t> cdf_percentile{7000}, outdoor_threshold{3000};
  Firmware() : command_(listener(60001)), heartbeat_(listener(60002)) {
    command_thread_ = std::thread([this] { serve(command_); });
    heartbeat_thread_ = std::thread([this] { serve(heartbeat_); });
  }
  ~Firmware() {
    stopping_ = true;
    command_thread_.join();
    heartbeat_thread_.join();
    close(command_);
    close(heartbeat_);
  }

 private:
  int command_, heartbeat_;
  std::atomic<bool> stopping_{false};
  std::thread command_thread_, heartbeat_thread_;

  bool readable(int fd) {
    pollfd p{fd, POLLIN, 0};
    return poll(&p, 1, 20) > 0;
  }
  void serve(int server) {
    while (!stopping_) {
      if (!readable(server))
        continue;
      int fd = accept(server, nullptr, nullptr);
      if (fd < 0)
        continue;
      sdk::OdinProtocolV1 protocol;
      std::vector<uint8_t> input;
      while (!stopping_) {
        if (!readable(fd))
          continue;
        uint8_t bytes[4096];
        auto size = recv(fd, bytes, sizeof(bytes), 0);
        if (size <= 0)
          break;
        input.insert(input.end(), bytes, bytes + size);
        while (!input.empty()) {
          std::unique_ptr<sdk::IPacket> request;
          int used = protocol.Parse(input.data(), input.size(), request);
          if (used <= 0)
            break;
          input.erase(input.begin(), input.begin() + used);
          std::vector<uint8_t> reply{0};
          if (request->GetCommandId() == 5)
            reply[0] = subscribe(request->GetPayload(), request->GetPayloadSize());
          if (request->GetCommandId() == 0x34) {
            ++adaptive_sets;
            const auto* p = request->GetPayload();
            if (request->GetPayloadSize() != 4)
              reply[0] = 2;
            else {
              cdf_percentile = (uint16_t(p[0]) << 8) | p[1];
              outdoor_threshold = (uint16_t(p[2]) << 8) | p[3];
            }
          }
          if (request->GetCommandId() == 0x35) {
            ++adaptive_gets;
            const uint16_t cdf = cdf_percentile, threshold = outdoor_threshold;
            reply = {0, static_cast<uint8_t>(cdf >> 8), static_cast<uint8_t>(cdf),
                     static_cast<uint8_t>(threshold >> 8), static_cast<uint8_t>(threshold)};
          }
          if (request->GetCommandId() == 0x32 || request->GetCommandId() == 0x33)
            ++aggregate_commands;
          sdk::OdinPacket ack(request->GetCommandId(), request->GetSequence(), 1, 1, reply.data(),
                              reply.size());
          std::vector<uint8_t> output;
          protocol.Pack(ack, output);
          send(fd, output.data(), output.size(), MSG_NOSIGNAL);
        }
      }
      close(fd);
    }
  }
  uint8_t subscribe(const uint8_t* data, size_t size) {
    if (!size)
      return 2;
    uint8_t channel = data[0], enabled = 0, transport = 0;
    uint16_t port = 0;
    for (size_t i = 1; i + 2 <= size;) {
      uint8_t type = data[i++], length = data[i++];
      if (i + length > size)
        return 2;
      if (type == 1 && length == 1)
        enabled = data[i];
      if (type == 2 && length == 2)
        port = (uint16_t(data[i]) << 8) | data[i + 1];
      if (type == 3 && length == 1)
        transport = data[i];
      i += length;
    }
    if (!enabled)
      return 0;
    if (!port)
      return 2;
    if (transport != 1)
      return 0;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    bool connected = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
    close(fd);
    if (channel == 16 || channel == 17) {
      camera_port = port;
      if (connected)
        ++camera_connections;
      if (reject_camera)
        return 1;
    }
    return connected ? 0 : 1;
  }
};

sdk::DiscoveredDevice descriptor() {
  sdk::DiscoveredDevice device;
  device.sn = "TCP-regression";
  device.model = "ODIN2";
  device.network.ip_address = "127.0.0.1";
  device.host_ip = "127.0.0.1";
  return device;
}

TEST(SdkStream, TcpListenerExistsBeforeDeviceAcknowledgesSubscription) {
  Firmware firmware;
  auto sdk = makeLiveSdk();
  auto handle = sdk->connect(descriptor());
  ASSERT_NE(handle, 0u);
  auto settings = defaults().streams.at(Channel::kImage0);
  settings.enabled = true;
  auto reply = sdk->start(handle, Channel::kImage0, settings, 1000);
  EXPECT_TRUE(reply.ok()) << reply.message;
  EXPECT_EQ(firmware.camera_connections, 1u);
  sdk->close(handle, Channel::kImage0);
  sdk->disconnect(handle);
}

TEST(SdkStream, RejectedSubscriptionClosesListenerAndAllowsRetry) {
  Firmware firmware;
  auto sdk = makeLiveSdk();
  auto handle = sdk->connect(descriptor());
  ASSERT_NE(handle, 0u);
  auto settings = defaults().streams.at(Channel::kImage0);
  settings.enabled = true;
  firmware.reject_camera = true;
  EXPECT_FALSE(sdk->start(handle, Channel::kImage0, settings, 1000).ok());
  EXPECT_EQ(firmware.camera_connections, 1u);
  int released = -1;
  EXPECT_NO_THROW(released = listener(firmware.camera_port));
  if (released >= 0)
    close(released);
  firmware.reject_camera = false;
  EXPECT_TRUE(sdk->start(handle, Channel::kImage0, settings, 1000).ok());
  EXPECT_EQ(firmware.camera_connections, 2u);
  sdk->close(handle, Channel::kImage0);
  sdk->disconnect(handle);
}

TEST(SdkStream, DedicatedAdaptiveCommandsRoundTripBigEndianParameters) {
  Firmware firmware;
  auto sdk = makeLiveSdk();
  auto handle = sdk->connect(descriptor());
  ASSERT_NE(handle, 0u);
  Command c;
  c.kind = CommandKind::SetAdaptive;
  c.adaptive = {0x2345, 0xABCD};
  EXPECT_TRUE(sdk->execute(handle, c).ok());
  EXPECT_EQ(firmware.adaptive_sets, 1u);
  EXPECT_EQ(firmware.cdf_percentile, 0x2345u);
  EXPECT_EQ(firmware.outdoor_threshold, 0xABCDu);
  c.kind = CommandKind::GetAdaptive;
  const auto reply = sdk->execute(handle, c);
  EXPECT_TRUE(reply.ok());
  EXPECT_EQ(reply.adaptive.cdf_percentile, 0x2345u);
  EXPECT_EQ(reply.adaptive.outdoor_threshold, 0xABCDu);
  EXPECT_EQ(firmware.adaptive_gets, 1u);
  EXPECT_EQ(firmware.aggregate_commands, 0u);
  sdk->disconnect(handle);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
