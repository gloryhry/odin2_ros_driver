#include "odin_lidar_api.h"
#include "logger.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace odin::sdk;

namespace {

constexpr int kDefaultDiscoveryTimeoutMs = 3000;
constexpr int kDefaultDurationSec = 5;

std::atomic<bool> g_running{true};
std::atomic<int> g_telemetry_count{0};
std::mutex g_print_mutex;

void SignalHandler(int) { g_running.store(false); }

void TelemetryCallback(const OdinTelemetryPacket &packet, void *client_data) {
  (void)client_data;

  std::lock_guard<std::mutex> lock(g_print_mutex);
  std::cout << "\n[Telemetry] frame=" << packet.frame_count
            << " timestamp=" << packet.timestamp << "\n"
            << "  uptime_s     : " << packet.data.uptime_s << "\n"
            << "  cpu_pct      : " << packet.data.cpu_pct << "%\n"
            << "  mem_used_kb  : " << packet.data.mem_used_kb << "\n"
            << "  mem_total_kb : " << packet.data.mem_total_kb << "\n"
            << "  mem_use_rate : " << packet.data.mem_use_rate << "%\n"
            << "  load_1       : " << packet.data.load_1 << "\n"
            << "  load_5       : " << packet.data.load_5 << "\n"
            << "  load_15      : " << packet.data.load_15 << "\n"
            << std::flush;

  g_telemetry_count.fetch_add(1);
}

void PrintUsage(const char *program_name) {
  std::cout << "Usage: " << program_name << " [options]\n"
            << "\n"
            << "Discover ODIN device(s), connect, and print telemetry data (0x42).\n"
            << "\n"
            << "Options:\n"
            << "  --device <ip>     Device IP address (default: first discovered)\n"
            << "  --duration <sec>  How long to listen for telemetry (default: 5)\n"
            << "  --timeout <ms>    Discovery timeout in ms (default: 3000)\n"
            << "  --help            Show this help message\n"
            << "\n"
            << "Examples:\n"
            << "  " << program_name << "\n"
            << "  " << program_name << " --device 192.168.1.200 --duration 10\n";
}

}  // namespace

int main(int argc, char *argv[]) {
  setvbuf(stdout, NULL, _IOLBF, 0);
  log_config(LOG_LEVEL_DEBUG, printf);

  std::string device_ip;
  int discovery_timeout_ms = kDefaultDiscoveryTimeoutMs;
  int duration_sec = kDefaultDurationSec;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else if (arg == "--device" && i + 1 < argc) {
      device_ip = argv[++i];
    } else if (arg == "--duration" && i + 1 < argc) {
      duration_sec = std::atoi(argv[++i]);
      if (duration_sec < 1) duration_sec = 1;
    } else if (arg == "--timeout" && i + 1 < argc) {
      discovery_timeout_ms = std::atoi(argv[++i]);
    }
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  std::cout << "========================================\n";
  std::cout << "ODIN Telemetry Tool\n";
  std::cout << "========================================\n";

  std::cout << "\n";
  std::cout << "========================================\n";
  std::cout << "  SDK VERSION: " << GetSdkVersion() << "\n";
  std::cout << "========================================\n";
  std::cout << "\n";

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
  DiscoveredDevice *target_device = nullptr;
  if (!device_ip.empty()) {
    for (auto &dev : devices) {
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

  // Connect to device
  OdinDeviceHandle handle = ConnectDevice(*target_device);
  if (handle == kInvalidDeviceHandle) {
    std::cerr << "Failed to connect to device.\n";
    return 1;
  }
  std::cout << "Connected to device.\n";

  // Get firmware version
  std::string version;
  if (GetFirmwareVersion(handle, version) == OdinResult::kOk) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "  FIRMWARE VERSION: " << version << "\n";
    std::cout << "========================================\n";
    std::cout << "\n";
  }

  // Start telemetry subscription (0x42 channel). This is opt-in: the
  // device will not push telemetry unless explicitly requested.
  std::cout << "Starting telemetry subscription (0x42 channel)...\n";
  if (StartTelemetry(handle) != OdinResult::kOk) {
    std::cerr << "Failed to start telemetry.\n";
    DisconnectDevice(handle);
    return 1;
  }

  // Register telemetry callback to receive parsed data.
  RegisterTelemetryCallback(handle, TelemetryCallback, nullptr);
  std::cout << "Telemetry callback registered. Listening for " << duration_sec
            << " second(s)...\n";

  // Wait for telemetry frames
  auto start_time = std::chrono::steady_clock::now();
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= duration_sec) {
      break;
    }
  }

  // Cleanup
  DisconnectDevice(handle);

  std::cout << "\n========================================\n";
  std::cout << "Received " << g_telemetry_count.load() << " telemetry frame(s).\n";
  std::cout << "========================================\n";

  return 0;
}
