#include "odin_lidar_api_ext.h"
#include "logger.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace odin::sdk;

namespace {

constexpr int kDefaultDiscoveryTimeoutMs = 3000;
constexpr int kDefaultIntervalMs = 200;
constexpr size_t kDefaultPayloadSize = 64;

std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

void PrintUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [options]\n"
            << "\n"
            << "Loop-send fixed algorithm downlink data (SendAlgoData, 0x40) to\n"
            << "an ODIN device. Use this to verify the 0x40 downlink path works.\n"
            << "Press Ctrl+C to stop.\n"
            << "\n"
            << "Options:\n"
            << "  --device <ip>     Device IP address (default: first discovered)\n"
            << "  --timeout <ms>    Discovery timeout in ms (default: 3000)\n"
            << "  --interval <ms>   Send interval in ms (default: 200)\n"
            << "  --size <bytes>    Payload size per send (default: 64)\n"
            << "  --help            Show this help message\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  setvbuf(stdout, NULL, _IOLBF, 0);  // use line buffer
  log_config(LOG_LEVEL_DEBUG, printf);

  std::string device_ip;
  int discovery_timeout_ms = kDefaultDiscoveryTimeoutMs;
  int interval_ms = kDefaultIntervalMs;
  size_t payload_size = kDefaultPayloadSize;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else if (arg == "--device" && i + 1 < argc) {
      device_ip = argv[++i];
    } else if (arg == "--timeout" && i + 1 < argc) {
      discovery_timeout_ms = std::atoi(argv[++i]);
    } else if (arg == "--interval" && i + 1 < argc) {
      interval_ms = std::atoi(argv[++i]);
    } else if (arg == "--size" && i + 1 < argc) {
      payload_size = static_cast<size_t>(std::strtoul(argv[++i], nullptr, 10));
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage(argv[0]);
      return 1;
    }
  }

  if (payload_size == 0) {
    std::cerr << "Error: --size must be > 0.\n";
    return 1;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "ODIN SendAlgoData (0x40 downlink) Test Tool\n";
  std::cout << "Payload size: " << payload_size << " bytes, interval: " << interval_ms
            << " ms\n";

  // Build a fixed, recognizable payload pattern: [0xA0 0xA1 0xA2 ...] wrapping.
  std::vector<uint8_t> payload(payload_size);
  for (size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(0xA0 + (i & 0x0F));
  }

  // Discover devices
  std::cout << "Discovering devices (timeout: " << discovery_timeout_ms << "ms)...\n";
  std::vector<DiscoveredDevice> devices;
  if (!DiscoverDevices(devices, discovery_timeout_ms)) {
    std::cerr << "Device discovery failed.\n";
    return 1;
  }
  if (devices.empty()) {
    std::cerr << "No devices found.\n";
    return 1;
  }

  std::cout << "Found " << devices.size() << " device(s):\n";
  for (size_t i = 0; i < devices.size(); ++i) {
    std::cout << "  [" << i << "] " << devices[i].network.ip_address
              << " (SN: " << devices[i].sn << ")\n";
  }

  // Select device
  DiscoveredDevice* target_device = nullptr;
  if (!device_ip.empty()) {
    for (auto& dev : devices) {
      if (dev.network.ip_address == device_ip) {
        target_device = &dev;
        break;
      }
    }
    if (!target_device) {
      std::cerr << "Device with IP " << device_ip << " not found.\n";
      return 1;
    }
  } else {
    target_device = &devices[0];
  }

  std::cout << "Using device: " << target_device->network.ip_address
            << " (SN: " << target_device->sn << ")\n";

  // Connect to device (the 0x40 downlink server is brought up automatically
  // inside ConnectDevice; the module connects into it asynchronously).
  OdinDeviceHandle handle = ConnectDevice(*target_device);
  if (handle == kInvalidDeviceHandle) {
    std::cerr << "Failed to connect to device.\n";
    return 1;
  }
  std::cout << "Connected to device.\n";

  std::string version;
  if (GetFirmwareVersion(handle, version) == OdinResult::kOk) {
    std::cout << "Firmware version: " << version << "\n";
  }

  std::cout << "Start looping SendAlgoData. Press Ctrl+C to stop.\n";

  uint64_t total = 0;
  uint64_t ok_count = 0;
  uint64_t not_ready_count = 0;
  uint64_t err_count = 0;

  while (g_running.load()) {
    // Stamp a rolling counter into the first 4 bytes so the receiver can tell
    // successive messages apart while the rest of the payload stays fixed.
    if (payload.size() >= 4) {
      payload[0] = static_cast<uint8_t>(total & 0xFF);
      payload[1] = static_cast<uint8_t>((total >> 8) & 0xFF);
      payload[2] = static_cast<uint8_t>((total >> 16) & 0xFF);
      payload[3] = static_cast<uint8_t>((total >> 24) & 0xFF);
    }

    OdinResult r = SendAlgoData(handle, payload.data(), payload.size());
    ++total;
    if (r == OdinResult::kOk) {
      ++ok_count;
      std::cout << "[#" << total << "] sent " << payload.size() << " bytes OK\n";
    } else if (r == OdinResult::kNotInitialized) {
      ++not_ready_count;
      // Module has not connected to the 0x40 downlink server yet.
      if (not_ready_count == 1 || (not_ready_count % 25) == 0) {
        std::cout << "[#" << total
                  << "] module not connected to 0x40 downlink yet; waiting...\n";
      }
    } else {
      ++err_count;
      std::cerr << "[#" << total
                << "] SendAlgoData failed, code=" << static_cast<int>(r) << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
  }

  std::cout << "\nStopping. Disconnecting...\n";
  DisconnectDevice(handle);

  std::cout << "========================================\n";
  std::cout << "Total attempts: " << total << "\n";
  std::cout << "  OK:           " << ok_count << "\n";
  std::cout << "  Not connected:" << not_ready_count << "\n";
  std::cout << "  Errors:       " << err_count << "\n";
  std::cout << "========================================\n";
  return 0;
}
