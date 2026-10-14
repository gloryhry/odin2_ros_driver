#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "odin_lidar_api.h"
#include "../discovery/IDiscovery/IDiscovery.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace odin {
namespace sdk {

class IPacket;

namespace network {

// =============================================================================
// Socket Handle Types
// =============================================================================

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

// =============================================================================
// Winsock Management (Windows only, no-op on other platforms)
// =============================================================================

bool AcquireWinsock();
void ReleaseWinsock();

// =============================================================================
// Socket Operations
// =============================================================================

void CloseSocket(SocketHandle sock);

// Create a UDP broadcast socket bound to specified interface
// Returns kInvalidSocket on failure
// If host_port is 0, OS will choose an available port
SocketHandle CreateBroadcastSocket(const std::string& host_ip, uint16_t host_port = 0);

// Create a UDP broadcast socket bound to INADDR_ANY (so it can receive
// limited-broadcast replies whose destination is 255.255.255.255) while
// pinning outbound datagrams to the network interface that owns host_ip via
// SO_BINDTODEVICE. This is required when the target device is reachable only
// through a non-default-route interface. On Linux, SO_BINDTODEVICE needs the
// CAP_NET_RAW capability (run as root or grant cap_net_raw). If the interface
// cannot be resolved or the option cannot be set, the socket still works but
// falls back to default-route egress. Returns kInvalidSocket on failure.
SocketHandle CreateBroadcastSocketForInterface(const std::string& host_ip, uint16_t host_port = 0);

// =============================================================================
// IP Address Utilities
// =============================================================================

std::vector<std::string> GetLocalIPAddresses();

// Return the network interface name (e.g. "eth0") that owns host_ip, or an
// empty string if it cannot be resolved. Always empty on Windows.
std::string GetInterfaceNameForIp(const std::string& host_ip);

bool IpStringToBytes(const std::string& ip, uint8_t* bytes_out);

std::string BytesToIpString(const uint8_t* bytes);

// =============================================================================
// Discovery Protocol Helpers
// =============================================================================

constexpr uint16_t kDefaultCommandPort = 60001;
constexpr uint16_t kCmdIdDeviceQuery = 0x01;

bool BuildDiscoveryPacket(const std::string& host_ip, uint16_t seq,
                          std::vector<uint8_t>& frame_out);

bool ParseDiscoveryResponse(const uint8_t* data, size_t length, DiscoveredDevice& device_out);

// Send discovery broadcast on socket, returns true on success
// Uses provided discovery or default if nullptr
// context: Transport-specific parameters (will be passed to discovery)
bool SendDiscoveryBroadcast(SocketHandle sock, uint16_t target_port,
                            const IDiscovery::Context& context,
                            std::shared_ptr<IDiscovery> discovery = nullptr);

// =============================================================================
// Device Discovery
// =============================================================================

// Discover devices on a specific network interface
// Uses provided discovery or default if nullptr
bool DiscoverDevicesOnInterface(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                                uint16_t host_port, uint16_t target_port,
                                const std::string& host_ip,
                                std::shared_ptr<IDiscovery> discovery = nullptr);

// Discover all network devices across all interfaces
// Uses provided discovery or default if nullptr
bool DiscoverNetworkDevices(std::vector<DiscoveredDevice>& devices, uint32_t timeout_ms,
                            std::shared_ptr<IDiscovery> discovery = nullptr);

}  // namespace network
}  // namespace sdk
}  // namespace odin
