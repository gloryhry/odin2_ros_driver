#include "OdinCommand.hpp"

#include "cJSON.h"
#include "ITransport.hpp"
#include "TcpTransport.hpp"
#include "OdinProtocol.hpp"
#include "payload_builder.h"
#include "internal/http_config_backend.hpp"
#include "internal/http_file_backend.hpp"
#include "logger.h"

#include <chrono>
#include <fstream>
#include <random>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace odin {
namespace sdk {

namespace {

#ifdef _WIN32
std::atomic<int> g_winsock_ref_count{0};

bool AcquireWinsock() {
  if (g_winsock_ref_count.fetch_add(1) == 0) {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      g_winsock_ref_count.fetch_sub(1);
      return false;
    }
  }
  return true;
}

void ReleaseWinsock() {
  if (g_winsock_ref_count.fetch_sub(1) == 1) {
    WSACleanup();
  }
}
#else
bool AcquireWinsock() { return true; }
void ReleaseWinsock() {}
#endif

}  // namespace

OdinCommand::OdinCommand()
    : http_config_(new HttpConfigBackend()),
      http_file_(new HttpFileBackend()) {}

OdinCommand::~OdinCommand() {
  Disconnect();
}

bool OdinCommand::Connect(const DiscoveredDevice& discovered_device,
                           OdinDeviceHandle handle) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_.load()) return true;

  // Extract transport-specific fields from the descriptor. The device layer
  // stays unaware of these; future variants (USB, etc.) would consume their
  // own fields off DiscoveredDevice here.
  device_ip_ = discovered_device.network.ip_address;
  if (device_ip_.empty()) {
    LOG_ERROR("OdinCommand::Connect: empty device IP\n");
    return false;
  }
  host_ip_ = discovered_device.host_ip.empty() ? "0.0.0.0"
                                                : discovered_device.host_ip;
  NetworkAddress local_addr;
  local_addr.ip = host_ip_;
  local_addr.port = 0;
  handle_ = handle;

  if (!AcquireWinsock()) {
    LOG_ERROR("OdinCommand: AcquireWinsock failed\n");
    return false;
  }

  // Step 1: Try heartbeat channel (TCP 60002)
  heartbeat_supported_ = true;
  
  std::unique_ptr<ITransport> hb_transport(new TcpTransport());
  NetworkAddress hb_remote_addr;
  hb_remote_addr.ip = device_ip_;
  hb_remote_addr.port = 60002;
  hb_transport->SetRemoteTarget(hb_remote_addr);

  if (!hb_transport->Open(local_addr)) {
    LOG_WARN("OdinCommand: Heartbeat transport Open failed (port 60002)\n");
    heartbeat_supported_ = false;
  } else {
    std::unique_ptr<IProtocol> hb_protocol(new OdinProtocolV1());
    heartbeat_channel_.reset(new CommandChannel(std::move(hb_protocol), std::move(hb_transport)));

    CommandChannelConfig hb_cfg;
    hb_cfg.default_timeout_ms = 2000;
    hb_cfg.device_handle = handle_;

    if (!heartbeat_channel_->Start(hb_cfg)) {
      LOG_WARN("OdinCommand: Heartbeat CommandChannel start failed\n");
      heartbeat_channel_.reset();
      heartbeat_supported_ = false;
    } else {
      // Send first heartbeat to verify
      std::vector<uint8_t> hb_payload;
      OdinCommandSyncResponse hb_response;
      if (!heartbeat_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kHeartbeat), 
                                         hb_payload, hb_response, 2000)) {
        LOG_WARN("OdinCommand: First heartbeat failed\n");
        heartbeat_channel_->Stop();
        heartbeat_channel_.reset();
        heartbeat_supported_ = false;
      } else {
        LOG_INFO("OdinCommand: Heartbeat channel established (port 60002)\n");
      }
    }
  }

  if (!heartbeat_supported_) {
    LOG_INFO("OdinCommand: Device does not support heartbeat\n");
  }

  // Step 2: Command channel (TCP 60001)
  std::unique_ptr<ITransport> cmd_transport(new TcpTransport());
  NetworkAddress cmd_remote_addr;
  cmd_remote_addr.ip = device_ip_;
  cmd_remote_addr.port = 60001;
  cmd_transport->SetRemoteTarget(cmd_remote_addr);

  if (!cmd_transport->Open(local_addr)) {
    LOG_ERROR("OdinCommand: Command transport Open failed (port 60001)\n");
    if (heartbeat_channel_) {
      heartbeat_channel_->Stop();
      heartbeat_channel_.reset();
    }
    ReleaseWinsock();
    return false;
  }

  std::unique_ptr<IProtocol> cmd_protocol(new OdinProtocolV1());
  command_channel_.reset(new CommandChannel(std::move(cmd_protocol), std::move(cmd_transport)));

  CommandChannelConfig cmd_cfg;
  cmd_cfg.default_timeout_ms = 3000;
  cmd_cfg.device_handle = handle_;

  if (!command_channel_->Start(cmd_cfg)) {
    LOG_ERROR("OdinCommand: Command CommandChannel start failed\n");
    command_channel_.reset();
    if (heartbeat_channel_) {
      heartbeat_channel_->Stop();
      heartbeat_channel_.reset();
    }
    ReleaseWinsock();
    return false;
  }

  // Bring up HTTP-backed sub-protocols. Failure here is non-fatal: if the
  // device's web server is unreachable we still keep the TLV channel up so
  // streaming/control works; only management calls will fail later.
  if (http_config_) http_config_->Connect(discovered_device);
  if (http_file_)   http_file_->Connect(discovered_device);

  // Bring up the algorithm downlink (data_type=0x40) server and subscribe the
  // module. Non-fatal: a failure here never blocks the main connection.
  StartAlgoDownlink();

  connected_.store(true);
  LOG_INFO("OdinCommand: Connected to %s\n", device_ip_.c_str());
  return true;
}

void OdinCommand::Disconnect() {
  // Stop any active background calibration fetch before tearing down the
  // command channel; otherwise the thread could use a stale channel.
  StopCalibThread();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_.load()) return;

  if (command_channel_) {
    command_channel_->Stop();
    command_channel_.reset();
  }

  if (heartbeat_channel_) {
    heartbeat_channel_->Stop();
    heartbeat_channel_.reset();
  }

  StopAlgoDownlink();
  // Note: no StopCalibServer here — the calib server is brought up and
  // torn down on demand inside FetchCalibrationFile().
  StopTelemetryServer();

  if (http_file_)   http_file_->Disconnect();
  if (http_config_) http_config_->Disconnect();

  ReleaseWinsock();
  connected_.store(false);
  LOG_INFO("OdinCommand: Disconnected\n");
}

bool OdinCommand::IsConnected() const {
  return connected_.load();
}

OdinResult OdinCommand::GetFirmwareVersion(std::string& version, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kVersionQuery), 
                                  payload, response, timeout_ms)) {
    return OdinResult::kTimeout;
  }

  if (response.result != OdinResult::kOk) {
    return response.result;
  }

  if (response.response.payload.size() >= 4) {
    uint8_t major = response.response.payload[1];
    uint8_t minor = response.response.payload[2];
    uint8_t patch = response.response.payload[3];
    version = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
  } else {
    version = "unknown";
  }

  return OdinResult::kOk;
}

OdinResult OdinCommand::SetDeviceMode(OdinDeviceMode mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(mode));

  LOG_DEBUG("OdinCommand::SetDeviceMode: mode=%d\n", static_cast<int>(mode));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kSetDeviceMode), 
                                  payload, response, timeout_ms)) {
    return OdinResult::kTimeout;
  }

  return response.result;
}

OdinResult OdinCommand::GetDeviceMode(OdinDeviceMode& mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kGetDeviceMode),
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetDeviceMode: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.result != OdinResult::kOk) {
    return response.result;
  }

  if (response.response.payload.size() >= 2) {
    uint8_t raw_mode = response.response.payload[1];
    if (raw_mode > static_cast<uint8_t>(OdinDeviceMode::kUpgrade)) {
      LOG_ERROR("OdinCommand::GetDeviceMode: invalid mode value %d\n", raw_mode);
      return OdinResult::kUnknownError;
    }
    mode = static_cast<OdinDeviceMode>(raw_mode);
  } else {
    LOG_ERROR("OdinCommand::GetDeviceMode: response too short\n");
    return OdinResult::kCommunicationError;
  }

  return OdinResult::kOk;
}

OdinResult OdinCommand::SetSensorMode(OdinSensorMode mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.push_back(0x00);
  payload.push_back(static_cast<uint8_t>(mode));

  LOG_DEBUG("OdinCommand::SetSensorMode: mode=%d\n", static_cast<int>(mode));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kSensorMode), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SetSensorMode: timeout\n");
    return OdinResult::kTimeout;
  }
  return response.result;
}

OdinResult OdinCommand::GetSensorMode(OdinSensorMode& mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;

  LOG_DEBUG("OdinCommand::GetSensorMode\n");

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kGetLiDARMode),
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetSensorMode: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.result != OdinResult::kOk) {
    return response.result;
  }

  if (response.response.payload.size() <= 1) {
    LOG_ERROR("OdinCommand::GetSensorMode: response too short\n");
    return OdinResult::kCommunicationError;
  }
  if (static_cast<OdinSensorMode>(response.response.payload[1]) > OdinSensorMode::kAdaptive) {
    LOG_ERROR("OdinCommand::GetSensorMode: invalid mode value %d\n", response.response.payload[1]);
  }
  mode = static_cast<OdinSensorMode>(response.response.payload[1]);
  
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetSensorCapability(std::vector<OdinSensorCapability>& capabilities,
                                             const std::vector<OdinDataChannel>& channels,
                                             uint32_t timeout_ms) {
  capabilities.clear();

  std::vector<uint8_t> payload;
  for (auto ch : channels) {
    payload.push_back(static_cast<uint8_t>(ch));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kCmdIdQueryCapability), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetSensorCapability: timeout\n");
    return OdinResult::kTimeout;
  }

  const auto& resp_payload = response.response.payload;
  if (resp_payload.empty()) {
    LOG_ERROR("OdinCommand::GetSensorCapability: empty response\n");
    return OdinResult::kUnknownError;
  }

  // JSON format
  if (resp_payload[0] == '{') {
    std::string json_str(resp_payload.begin(), resp_payload.end());
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
      LOG_ERROR("OdinCommand::GetSensorCapability: JSON parse failed\n");
      return OdinResult::kUnknownError;
    }

    cJSON* channels_arr = cJSON_GetObjectItem(root, "channels");
    if (channels_arr && cJSON_IsArray(channels_arr)) {
      int arr_size = cJSON_GetArraySize(channels_arr);
      for (int i = 0; i < arr_size; ++i) {
        cJSON* ch_obj = cJSON_GetArrayItem(channels_arr, i);
        if (!ch_obj || !cJSON_IsObject(ch_obj)) continue;

        OdinSensorCapability cap;

        cJSON* data_type = cJSON_GetObjectItem(ch_obj, "data_type");
        if (data_type && cJSON_IsNumber(data_type)) {
          cap.channel = static_cast<OdinDataChannel>(data_type->valueint);
        }

        cJSON* format = cJSON_GetObjectItem(ch_obj, "format");
        OdinDataFormat channel_format = OdinDataFormat::kUnknown;
        if (format && cJSON_IsNumber(format)) {
          channel_format = static_cast<OdinDataFormat>(format->valueint);
        }

        cJSON* resolutions = cJSON_GetObjectItem(ch_obj, "resolutions");
        if (resolutions && cJSON_IsArray(resolutions)) {
          int res_count = cJSON_GetArraySize(resolutions);
          for (int j = 0; j < res_count; ++j) {
            cJSON* res_obj = cJSON_GetArrayItem(resolutions, j);
            if (!res_obj || !cJSON_IsObject(res_obj)) continue;

            OdinStreamCfg cfg;
            cfg.format = channel_format;

            cJSON* width = cJSON_GetObjectItem(res_obj, "width");
            if (width && cJSON_IsNumber(width)) {
              cfg.width = static_cast<uint16_t>(width->valueint);
            }

            cJSON* height = cJSON_GetObjectItem(res_obj, "height");
            if (height && cJSON_IsNumber(height)) {
              cfg.height = static_cast<uint16_t>(height->valueint);
            }

            cJSON* fps_arr = cJSON_GetObjectItem(res_obj, "fps");
            if (fps_arr && cJSON_IsArray(fps_arr)) {
              int fps_count = cJSON_GetArraySize(fps_arr);
              for (int k = 0; k < fps_count; ++k) {
                cJSON* fps_val = cJSON_GetArrayItem(fps_arr, k);
                if (fps_val && cJSON_IsNumber(fps_val)) {
                  OdinStreamCfg cfg_copy = cfg;
                  cfg_copy.fps = static_cast<uint16_t>(fps_val->valueint);
                  cap.modes.push_back(cfg_copy);
                }
              }
            } else {
              cap.modes.push_back(cfg);
            }
          }
        }

        if (channels.empty()) {
          capabilities.push_back(cap);
        } else {
          for (auto ch : channels) {
            if (ch == cap.channel) {
              capabilities.push_back(cap);
              break;
            }
          }
        }
      }
    }

    cJSON_Delete(root);
    UpdateCapabilityTable(capabilities);
    return OdinResult::kOk;
  }

  // Binary TLV format fallback
  if (resp_payload.size() < 2) {
    LOG_ERROR("OdinCommand::GetSensorCapability: response too short\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = resp_payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetSensorCapability: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  uint8_t channel_count = resp_payload[1];
  size_t offset = 2;

  for (uint8_t i = 0; i < channel_count && offset < resp_payload.size(); i++) {
    if (offset + 2 > resp_payload.size()) break;

    OdinSensorCapability cap;
    cap.channel = static_cast<OdinDataChannel>(resp_payload[offset++]);
    uint8_t mode_count = resp_payload[offset++];

    for (uint8_t j = 0; j < mode_count && offset + 6 <= resp_payload.size(); j++) {
      OdinStreamCfg cfg;
      cfg.width = (static_cast<uint16_t>(resp_payload[offset]) << 8) |
                  static_cast<uint16_t>(resp_payload[offset + 1]);
      cfg.height = (static_cast<uint16_t>(resp_payload[offset + 2]) << 8) |
                   static_cast<uint16_t>(resp_payload[offset + 3]);
      cfg.fps = (static_cast<uint16_t>(resp_payload[offset + 4]));
      cfg.format = static_cast<OdinDataFormat>(resp_payload[offset + 5]);
      offset += 6;
      cap.modes.push_back(cfg);
    }

    if (channels.empty()) {
      capabilities.push_back(cap);
    } else {
      for (auto ch : channels) {
        if (ch == cap.channel) {
          capabilities.push_back(cap);
          break;
        }
      }
    }
  }

  UpdateCapabilityTable(capabilities);
  return OdinResult::kOk;
}

void OdinCommand::UpdateCapabilityTable(const std::vector<OdinSensorCapability>& capabilities) {
  std::lock_guard<std::mutex> lock(capability_mutex_);
  for (const auto& cap : capabilities) {
    capability_table_[cap.channel] = cap.modes;
  }
}

bool OdinCommand::LookupStreamConfig(OdinDataChannel channel, const OdinStreamCfg* cfg,
                                      uint8_t& resolution_id, uint8_t& fps_id) {
  resolution_id = 0xFF;
  fps_id = 0xFF;
  
  if (!cfg || (cfg->width == 0 && cfg->height == 0)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(capability_mutex_);
  auto it = capability_table_.find(channel);
  if (it == capability_table_.end()) {
    return false;
  }

  const auto& configs = it->second;
  for (size_t i = 0; i < configs.size(); ++i) {
    if (configs[i].width == cfg->width && configs[i].height == cfg->height) {
      resolution_id = static_cast<uint8_t>(i);
      if (configs[i].fps == cfg->fps || cfg->fps == 0) {
        fps_id = 0;
      }
      return true;
    }
  }
  return false;
}

std::vector<uint8_t> OdinCommand::BuildChannelConfigPayload(OdinDataChannel channel,
                                                             OdinTransportMode transport,
                                                             uint16_t dst_port,
                                                             const OdinStreamCfg* cfg) {
  // Resolve resolution/fps IDs from capability table before building payload.
  uint8_t resolution_id = 0xFF;
  uint8_t fps_id = 0xFF;
  if (cfg && (cfg->width > 0 || cfg->height > 0)) {
    LookupStreamConfig(channel, cfg, resolution_id, fps_id);
  }

  PayloadBuilder builder;
  builder.WriteU8(static_cast<uint8_t>(channel))
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe), uint8_t(1))
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kDstPort), dst_port)
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kTransportMode),
                 static_cast<uint8_t>(transport))
         .RequireTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe));

  // Optional mode TLVs (only when resolved from capability table)
  if (resolution_id != 0xFF) {
    builder.AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kResolutionId), resolution_id);
  }
  if (fps_id != 0xFF) {
    builder.AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kFpsId), fps_id);
  }
  if (cfg && cfg->format != OdinDataFormat::kUnknown) {
    builder.AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kFormat),
                   static_cast<uint8_t>(cfg->format));
  }

  return builder.Build();
}

bool OdinCommand::ProbePortBindable(const std::string& ip, uint16_t port) {
  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return false;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (ip.empty() || ip == "0.0.0.0") {
    addr.sin_addr.s_addr = INADDR_ANY;
  } else {
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
  }

  int result = ::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
#ifdef _WIN32
  closesocket(sock);
#else
  ::close(sock);
#endif
  return result == 0;
}

uint16_t OdinCommand::AllocateDstPort() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<uint16_t> dist(10000, 60000);

  std::lock_guard<std::mutex> lock(port_mutex_);
  for (int i = 0; i < 1000; ++i) {
    uint16_t port = dist(gen);
    if (used_ports_.count(port) == 0 && ProbePortBindable(host_ip_, port)) {
      used_ports_.insert(port);
      return port;
    }
  }
  return 0;
}

OdinResult OdinCommand::SendStartStream(OdinDataChannel channel, OdinTransportMode transport,
                                         const OdinStreamCfg* cfg, uint32_t timeout_ms) {
  uint16_t dst_port = AllocateDstPort();
  if (dst_port == 0) {
    LOG_ERROR("OdinCommand::SendStartStream: failed to allocate dst_port\n");
    return OdinResult::kUnknownError;
  }

  auto payload = BuildChannelConfigPayload(channel, transport, dst_port, cfg);

  const char* transport_name = (transport == OdinTransportMode::kTcp) ? "TCP" : "UDP";
  LOG_DEBUG("OdinCommand::SendStartStream: channel=0x%02x, port=%u, %s\n",
            static_cast<uint8_t>(channel), dst_port, transport_name);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SendStartStream: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SendStartStream: empty response\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SendStartStream: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  // Record the local-side endpoint so capture can retrieve it via
  // GetLocalEndpoint() without ever surfacing the port to the device layer.
  {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    NetworkAddress endpoint;
    endpoint.ip = host_ip_;
    endpoint.port = dst_port;
    channel_endpoints_[channel] = endpoint;
  }
  return OdinResult::kOk;
}

const ITransportAddress& OdinCommand::GetLocalEndpoint(OdinDataChannel channel) const {
  // Singleton sentinel returned when the channel is not currently streaming.
  // The empty NetworkAddress is intentionally invalid (port 0) so that
  // misuse fails fast at the capture layer.
  static const NetworkAddress kInvalidEndpoint{};

  std::lock_guard<std::mutex> lock(port_mutex_);
  auto it = channel_endpoints_.find(channel);
  if (it == channel_endpoints_.end()) {
    LOG_WARN("OdinCommand::GetLocalEndpoint: channel 0x%02x not started\n",
             static_cast<uint8_t>(channel));
    return kInvalidEndpoint;
  }
  return it->second;
}

OdinResult OdinCommand::SendCloseStream(OdinDataChannel channel, uint32_t timeout_ms) {
  // Per protocol v0x01: unsubscribe uses subscribe=0 TLV (mandatory).
  // dst_port is irrelevant for unsubscription.
  PayloadBuilder builder;
  builder.WriteU8(static_cast<uint8_t>(channel))
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe), uint8_t(0))
         .RequireTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe));
  auto payload = builder.Build();

  LOG_DEBUG("OdinCommand::SendCloseStream: channel=0x%02x\n", static_cast<uint8_t>(channel));

  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig), 
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SendCloseStream: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SendCloseStream: empty response\n");
    return OdinResult::kUnknownError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SendCloseStream: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  // Release the per-channel local endpoint and free its port reservation.
  {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    auto it = channel_endpoints_.find(channel);
    if (it != channel_endpoints_.end()) {
      used_ports_.erase(it->second.port);
      channel_endpoints_.erase(it);
    }
  }
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetChannelConfig(OdinDataChannel channel, OdinChannelConfigInfo& info,
                                            uint32_t timeout_ms) {
  // REQ: [data_type: 1B] (per protocol v0x02, per-channel query)
  std::vector<uint8_t> req_payload;
  req_payload.push_back(static_cast<uint8_t>(channel));

  LOG_DEBUG("OdinCommand::GetChannelConfig: channel=0x%02X\n", static_cast<uint8_t>(channel));
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kGetChannelConfig),
                                  req_payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetChannelConfig: timeout\n");
    return OdinResult::kTimeout;
  }

  // ACK: [ret_code: 1B] [data_type: 1B] [TLV ...]
  if (response.response.payload.size() < 2) {
    LOG_ERROR("OdinCommand::GetChannelConfig: response too short (%zu bytes)\n",
              response.response.payload.size());
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetChannelConfig: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  // Parse TLV region (skip ret_code + data_type echo)
  const uint8_t* tlv_data = response.response.payload.data() + 2;
  size_t tlv_len = response.response.payload.size() - 2;
  TlvParser parser = TlvParser::Parse(tlv_data, tlv_len);

  info.subscribed = (parser.GetU8(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe), 0) != 0);
  info.dst_port = parser.GetU16BE(static_cast<uint8_t>(ChannelConfigTlvType::kDstPort), 0);
  info.transport_mode = static_cast<OdinTransportMode>(
      parser.GetU8(static_cast<uint8_t>(ChannelConfigTlvType::kTransportMode),
                   static_cast<uint8_t>(OdinTransportMode::kUdp)));
  info.resolution_id = parser.GetU8(static_cast<uint8_t>(ChannelConfigTlvType::kResolutionId), 0xFF);
  info.fps_id = parser.GetU8(static_cast<uint8_t>(ChannelConfigTlvType::kFpsId), 0xFF);
  info.format = static_cast<OdinDataFormat>(
      parser.GetU8(static_cast<uint8_t>(ChannelConfigTlvType::kFormat),
                   static_cast<uint8_t>(OdinDataFormat::kUnknown)));

  return OdinResult::kOk;
}

OdinResult OdinCommand::SetFilterParam(const OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  if (!params.has_confidence && !params.has_adaptive && !params.has_range) {
    LOG_ERROR("OdinCommand::SetFilterParam: no filter selected\n");
    return OdinResult::kInvalidArgument;
  }

  PayloadBuilder builder;

  // Confidence filter block: outer TLV [kConfidence][len][inner sub-TLVs]
  if (params.has_confidence) {
    PayloadBuilder inner;
    inner.AddTlv(0x01, params.confidence.confidence_threshold);  // uint8_t
    auto sub = inner.Build();
    builder.AddRawTlv(static_cast<uint8_t>(OdinLidarFilterId::kConfidence),
                      sub.data(), sub.size());
  }

  // Adaptive filter block
  if (params.has_adaptive) {
    PayloadBuilder inner;
    inner.AddTlv(0x01, params.adaptive.cdf_percentile)    // uint16_t BE
         .AddTlv(0x02, params.adaptive.outdoor_threshold); // uint16_t BE
    auto sub = inner.Build();
    builder.AddRawTlv(static_cast<uint8_t>(OdinLidarFilterId::kAdaptive),
                      sub.data(), sub.size());
  }

  // Range filter block
  if (params.has_range) {
    PayloadBuilder inner;
    inner.AddTlv(0x01, params.range.min_depth);  // uint16_t BE
    auto sub = inner.Build();
    builder.AddRawTlv(static_cast<uint8_t>(OdinLidarFilterId::kRange),
                      sub.data(), sub.size());
  }

  auto payload = builder.Build();
  LOG_DEBUG("OdinCommand::SetFilterParam: confidence=%d adaptive=%d range=%d\n",
            params.has_confidence, params.has_adaptive, params.has_range);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kSetFilterParam),
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SetFilterParam: timeout\n");
    return OdinResult::kTimeout;
  }
  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SetFilterParam: empty response\n");
    return OdinResult::kCommunicationError;
  }
  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SetFilterParam: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetFilterParam(OdinLidarFilterParams& params,
                                       uint32_t timeout_ms) {
  // REQ payload: empty = query all filter types
  std::vector<uint8_t> payload;

  LOG_DEBUG("OdinCommand::GetFilterParam: query all\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kGetFilterParam),
                                  payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetFilterParam: timeout\n");
    return OdinResult::kTimeout;
  }
  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::GetFilterParam: empty response\n");
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetFilterParam: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  // ACK: [ret_code:1B] [filter_id:1B][outer_len:1B][sub-TLV block:outer_len B] ...
  params = OdinLidarFilterParams();
  const auto& pl = response.response.payload;
  size_t offset = 1;  // skip ret_code
  while (offset + 2 <= pl.size()) {
    uint8_t filter_id  = pl[offset];
    uint8_t outer_len  = pl[offset + 1];
    offset += 2;
    if (offset + outer_len > pl.size()) {
      LOG_ERROR("OdinCommand::GetFilterParam: truncated at offset %zu\n", offset);
      return OdinResult::kCommunicationError;
    }
    TlvParser inner = TlvParser::Parse(pl.data() + offset, outer_len);
    if (filter_id == static_cast<uint8_t>(OdinLidarFilterId::kConfidence)) {
      params.has_confidence = true;
      params.confidence.confidence_threshold =
          inner.GetU8(0x01, params.confidence.confidence_threshold);
    } else if (filter_id == static_cast<uint8_t>(OdinLidarFilterId::kAdaptive)) {
      params.has_adaptive = true;
      params.adaptive.cdf_percentile    = inner.GetU16BE(0x01, params.adaptive.cdf_percentile);
      params.adaptive.outdoor_threshold = inner.GetU16BE(0x02, params.adaptive.outdoor_threshold);
    } else if (filter_id == static_cast<uint8_t>(OdinLidarFilterId::kRange)) {
      params.has_range = true;
      params.range.min_depth = inner.GetU16BE(0x01, params.range.min_depth);
    }
    offset += outer_len;
  }
  return OdinResult::kOk;
}

// =============================================================================
// Restore default config (cmd_id=0x09)
// =============================================================================

OdinResult OdinCommand::RestoreDefaultConfig(uint32_t timeout_ms) {
  std::vector<uint8_t> payload;  // no data

  LOG_DEBUG("OdinCommand::RestoreDefaultConfig\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kRestoreDefaultConfig),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::RestoreDefaultConfig: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::RestoreDefaultConfig: empty response\n");
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::RestoreDefaultConfig: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }
  return OdinResult::kOk;
}

// =============================================================================
// Time sync commands (0x20-0x2F)
// =============================================================================

OdinResult OdinCommand::SetDefaultSyncMode(OdinTimeSyncMode mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;
  payload.push_back(static_cast<uint8_t>(mode));

  LOG_DEBUG("OdinCommand::SetDefaultSyncMode: mode=%d\n", static_cast<int>(mode));
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kSetDefaultSyncMode),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SetDefaultSyncMode: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SetDefaultSyncMode: empty response\n");
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code == 2) {
    LOG_ERROR("OdinCommand::SetDefaultSyncMode: invalid sync mode\n");
    return OdinResult::kInvalidArgument;
  }
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SetDefaultSyncMode: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetDefaultSyncMode(OdinTimeSyncMode& mode, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;  // no data

  LOG_DEBUG("OdinCommand::GetDefaultSyncMode\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kGetDefaultSyncMode),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetDefaultSyncMode: timeout\n");
    return OdinResult::kTimeout;
  }

  // ACK: [ret_code:1B][sync_mode:1B] = 2B
  const auto& pl = response.response.payload;
  if (pl.size() < 2) {
    LOG_ERROR("OdinCommand::GetDefaultSyncMode: response too short (%zu bytes)\n", pl.size());
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = pl[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetDefaultSyncMode: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  mode = static_cast<OdinTimeSyncMode>(pl[1]);
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetPtpSyncStatus(OdinPtpSyncStatus& status, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;  // no data

  LOG_DEBUG("OdinCommand::GetPtpSyncStatus\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kGetPtpSyncStatus),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetPtpSyncStatus: timeout\n");
    return OdinResult::kTimeout;
  }

  // ACK: [ret_code:1B][sync_state:1B][servo_state:1B][offset_ns:8B][path_delay_ns:8B][freq_ppb:4B] = 23B
  const auto& pl = response.response.payload;
  if (pl.size() < 23) {
    LOG_ERROR("OdinCommand::GetPtpSyncStatus: response too short (%zu bytes)\n", pl.size());
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = pl[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetPtpSyncStatus: error code %d (ptp_daemon may not be running)\n", ret_code);
    return OdinResult::kUnknownError;
  }

  status.sync_state    = pl[1];
  status.servo_state   = pl[2];
  // Parse int64_t BE: offset_ns
  status.offset_ns     = static_cast<int64_t>(
      (static_cast<uint64_t>(pl[3])  << 56) | (static_cast<uint64_t>(pl[4])  << 48) |
      (static_cast<uint64_t>(pl[5])  << 40) | (static_cast<uint64_t>(pl[6])  << 32) |
      (static_cast<uint64_t>(pl[7])  << 24) | (static_cast<uint64_t>(pl[8])  << 16) |
      (static_cast<uint64_t>(pl[9])  << 8)  | (static_cast<uint64_t>(pl[10])));
  // Parse int64_t BE: path_delay_ns
  status.path_delay_ns = static_cast<int64_t>(
      (static_cast<uint64_t>(pl[11]) << 56) | (static_cast<uint64_t>(pl[12]) << 48) |
      (static_cast<uint64_t>(pl[13]) << 40) | (static_cast<uint64_t>(pl[14]) << 32) |
      (static_cast<uint64_t>(pl[15]) << 24) | (static_cast<uint64_t>(pl[16]) << 16) |
      (static_cast<uint64_t>(pl[17]) << 8)  | (static_cast<uint64_t>(pl[18])));
  // Parse int32_t BE: freq_ppb
  status.freq_ppb      = static_cast<int32_t>(
      (static_cast<uint32_t>(pl[19]) << 24) | (static_cast<uint32_t>(pl[20]) << 16) |
      (static_cast<uint32_t>(pl[21]) << 8)  | (static_cast<uint32_t>(pl[22])));

  return OdinResult::kOk;
}

// =============================================================================
// Standalone adaptive param commands (0x34/0x35)
// =============================================================================

OdinResult OdinCommand::SetAdaptiveParam(const OdinAdaptiveFilterParam& param,
                                         uint32_t timeout_ms) {
  // REQ: [cdf_percentile: 2B BE][outdoor_threshold: 2B BE]
  PayloadBuilder builder;
  builder.WriteU16BE(param.cdf_percentile)
         .WriteU16BE(param.outdoor_threshold);
  auto payload = builder.Build();

  LOG_DEBUG("OdinCommand::SetAdaptiveParam: cdf=%u outdoor=%u\n",
            param.cdf_percentile, param.outdoor_threshold);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kSetAdaptiveParam),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::SetAdaptiveParam: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::SetAdaptiveParam: empty response\n");
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code == 2) {
    LOG_ERROR("OdinCommand::SetAdaptiveParam: invalid parameter\n");
    return OdinResult::kInvalidArgument;
  }
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::SetAdaptiveParam: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }
  return OdinResult::kOk;
}

OdinResult OdinCommand::GetAdaptiveParam(OdinAdaptiveFilterParam& param, uint32_t timeout_ms) {
  std::vector<uint8_t> payload;  // no data

  LOG_DEBUG("OdinCommand::GetAdaptiveParam\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kGetAdaptiveParam),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::GetAdaptiveParam: timeout\n");
    return OdinResult::kTimeout;
  }

  // ACK: [ret_code:1B][cdf_percentile:2B BE][outdoor_threshold:2B BE] = 5B
  const auto& pl = response.response.payload;
  if (pl.size() < 5) {
    LOG_ERROR("OdinCommand::GetAdaptiveParam: response too short (%zu bytes)\n", pl.size());
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = pl[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::GetAdaptiveParam: error code %d\n", ret_code);
    return OdinResult::kUnknownError;
  }

  param.cdf_percentile    = (static_cast<uint16_t>(pl[1]) << 8) | static_cast<uint16_t>(pl[2]);
  param.outdoor_threshold = (static_cast<uint16_t>(pl[3]) << 8) | static_cast<uint16_t>(pl[4]);
  return OdinResult::kOk;
}

// =============================================================================
// Algorithm commands (0x70-0x7F)
// =============================================================================

OdinResult OdinCommand::ResetInitialPose(uint32_t timeout_ms) {
  std::vector<uint8_t> payload;  // no data

  LOG_DEBUG("OdinCommand::ResetInitialPose\n");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!command_channel_) return OdinResult::kNotInitialized;

  OdinCommandSyncResponse response;
  if (!command_channel_->SendSync(
          static_cast<uint16_t>(Odin2CmdId::kResetInitialPose),
          payload, response, timeout_ms)) {
    LOG_ERROR("OdinCommand::ResetInitialPose: timeout\n");
    return OdinResult::kTimeout;
  }

  if (response.response.payload.empty()) {
    LOG_ERROR("OdinCommand::ResetInitialPose: empty response\n");
    return OdinResult::kCommunicationError;
  }

  uint8_t ret_code = response.response.payload[0];
  if (ret_code != 0) {
    LOG_ERROR("OdinCommand::ResetInitialPose: error code %d (algo process may not be running)\n", ret_code);
    return OdinResult::kUnknownError;
  }
  return OdinResult::kOk;
}

bool OdinCommand::IsHeartbeatSupported() const {
  return heartbeat_supported_;
}

// =============================================================================
// Device management (formerly IConfigService) - delegates to HttpConfigBackend.
// =============================================================================

OdinResult OdinCommand::GetNetworkAttribute(NetworkAttribute& attr,
                                            uint32_t timeout_ms) {
  if (!http_config_) return OdinResult::kNotInitialized;
  return http_config_->GetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinCommand::SetNetworkAttribute(const NetworkAttribute& attr,
                                            uint32_t timeout_ms) {
  if (!http_config_) return OdinResult::kNotInitialized;
  return http_config_->SetNetworkAttribute(attr, timeout_ms);
}

OdinResult OdinCommand::Reboot(uint32_t timeout_ms) {
  if (!http_config_) return OdinResult::kNotInitialized;

  // The device drops every existing connection on reboot, so tear our own
  // channels down first - callers must Connect() again after reboot.
  OdinResult r = http_config_->Reboot(timeout_ms);
  Disconnect();
  return r;
}

// =============================================================================
// File transfer (formerly IFileService) - delegates to HttpFileBackend.
// =============================================================================

OdinResult OdinCommand::Upload(OdinFileType type,
                               const std::string& local_path,
                               CommandProgressCallback cb) {
  if (!http_file_) return OdinResult::kNotInitialized;
  return http_file_->Upload(type, local_path, std::move(cb));
}

OdinResult OdinCommand::Download(OdinFileType type,
                                 const std::string& save_path,
                                 CommandProgressCallback cb) {
  if (!http_file_) return OdinResult::kNotInitialized;
  return http_file_->Download(type, save_path, std::move(cb));
}

OdinResult OdinCommand::UploadFirmware(const std::string& firmware_path,
                                       CommandProgressCallback cb,
                                       int timeout_s) {
  if (!http_file_) return OdinResult::kNotInitialized;
  return http_file_->UploadFirmware(firmware_path, std::move(cb), timeout_s);
}

// =============================================================================
// Algorithm downlink passthrough (host -> device, data_type=0x40).
//
// Spec source of truth: docs/protocol/PROTOCOL.md sections 1.3 (0x40
// ChannelConfig subscription) and 2.4 (v2 data frame framing). The host is the
// TCP server: it listens on an allocated port, subscribes the module to it via
// ChannelConfig, and the module connects in asynchronously. SendAlgoData()
// fragments the opaque payload into 32B-header v2 frames and writes them on the
// accepted connection. Lifecycle is owned by Connect()/Disconnect().
// =============================================================================

namespace {
// Per protocol §2.4: single-chunk payload <= 60000 bytes (< 64KB, leaving TCP/IP
// header headroom); a reassembled logical message is conservatively <= 256KB.
constexpr size_t kAlgoMaxChunkPayload = 60000;
constexpr size_t kAlgoMaxMessageSize = 256 * 1024;
}  // namespace

std::vector<uint8_t> OdinCommand::BuildChannelConfigPayloadRaw(uint8_t data_type,
                                                               OdinTransportMode transport,
                                                               uint16_t dst_port) {
  PayloadBuilder builder;
  builder.WriteU8(data_type)
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe), uint8_t(1))
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kDstPort), dst_port)
         .AddTlv(static_cast<uint8_t>(ChannelConfigTlvType::kTransportMode),
                 static_cast<uint8_t>(transport))
         .RequireTlv(static_cast<uint8_t>(ChannelConfigTlvType::kSubscribe));
  return builder.Build();
}

void OdinCommand::StartAlgoDownlink() {
  // Caller (Connect) holds mutex_. The command channel must already be up.
  uint16_t port = AllocateDstPort();
  if (port == 0) {
    LOG_WARN("OdinCommand: algo downlink port allocation failed; 0x40 disabled\n");
    return;
  }

  std::unique_ptr<TcpTransport> server(new TcpTransport());
  // Server mode: bind+listen on host_ip:port. No remote target is set, so
  // Open() falls through to OpenAsServer(); StartReceiving() spawns the accept
  // loop that flips IsConnected() once the module connects in.
  NetworkAddress local_addr(host_ip_, port);
  if (!server->Open(local_addr) || !server->StartReceiving()) {
    LOG_WARN("OdinCommand: algo downlink server open failed on port %u; "
             "0x40 disabled\n", port);
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
    return;
  }

  {
    std::lock_guard<std::mutex> algo_lock(algo_mutex_);
    algo_downlink_ = std::move(server);
    algo_downlink_port_ = port;
    algo_frame_seq_ = 0;
    algo_not_connected_warned_ = false;
  }

  // Subscribe the module to the downlink (ChannelConfig, data_type=0x40, TCP).
  // Keep the server listening even if the device rejects/ignores it, so the
  // module can still connect in later.
  if (command_channel_) {
    auto payload = BuildChannelConfigPayloadRaw(0x40, OdinTransportMode::kTcp, port);
    OdinCommandSyncResponse response;
    if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig),
                                    payload, response, 1000)) {
      LOG_WARN("OdinCommand: algo downlink 0x40 subscribe timed out "
               "(firmware may not support it)\n");
    } else if (response.response.payload.empty() || response.response.payload[0] != 0) {
      LOG_WARN("OdinCommand: algo downlink 0x40 subscribe rejected "
               "(firmware may not support it)\n");
    } else {
      LOG_INFO("OdinCommand: algo downlink 0x40 subscribed, port=%u\n", port);
    }
  }
}

void OdinCommand::StopAlgoDownlink() {
  // Caller (Disconnect) holds mutex_.
  std::unique_ptr<TcpTransport> to_close;
  uint16_t port = 0;
  {
    std::lock_guard<std::mutex> algo_lock(algo_mutex_);
    to_close = std::move(algo_downlink_);
    port = algo_downlink_port_;
    algo_downlink_port_ = 0;
    algo_not_connected_warned_ = false;
  }
  // Close outside algo_mutex_: TcpTransport::Close() joins its accept/receive
  // threads, which must not contend with a concurrent SendAlgoData().
  if (to_close) to_close->Close();
  if (port != 0) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
  }
}

OdinResult OdinCommand::SendAlgoData(const uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) {
    return OdinResult::kInvalidArgument;
  }
  if (len > kAlgoMaxMessageSize) {
    LOG_ERROR("OdinCommand::SendAlgoData: message too large (%zu > %zu)\n",
              len, kAlgoMaxMessageSize);
    return OdinResult::kInvalidArgument;
  }

  std::lock_guard<std::mutex> algo_lock(algo_mutex_);
  if (!algo_downlink_) {
    return OdinResult::kNotInitialized;
  }
  if (!algo_downlink_->IsConnected()) {
    if (!algo_not_connected_warned_) {
      LOG_WARN("OdinCommand::SendAlgoData: module not connected to the 0x40 "
               "downlink server yet; data dropped until it connects\n");
      algo_not_connected_warned_ = true;
    }
    return OdinResult::kNotInitialized;
  }
  algo_not_connected_warned_ = false;

  // Write the full buffer over a possibly-partial TCP send().
  auto send_all = [this](const uint8_t* p, size_t n) -> bool {
    size_t off = 0;
    while (off < n) {
      int sent = algo_downlink_->Send(p + off, n - off);
      if (sent <= 0) return false;
      off += static_cast<size_t>(sent);
    }
    return true;
  };

  const uint32_t frame_seq = algo_frame_seq_++;
  const uint16_t chunk_count =
      static_cast<uint16_t>((len + kAlgoMaxChunkPayload - 1) / kAlgoMaxChunkPayload);
  const uint64_t timestamp_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  size_t offset = 0;
  for (uint16_t i = 0; i < chunk_count; ++i) {
    const size_t chunk_len = (len - offset < kAlgoMaxChunkPayload)
                                 ? (len - offset)
                                 : kAlgoMaxChunkPayload;

    OdinDataFrameHeader header{};
    header.sof = OdinConst::kDataSof;
    header.version = OdinConst::kDataVersion;
    header.data_type = 0x40;
    header.flags = (i + 1 == chunk_count) ? 0x01 : 0x00;
    header.frame_seq = frame_seq;
    header.chunk_index = i;
    header.chunk_count = chunk_count;
    header.element_count = static_cast<uint16_t>(chunk_len);
    header.payload_len = static_cast<uint16_t>(chunk_len);
    header.timestamp = timestamp_ns;
    header.interval_ns = 0;
    header.crc32 = 0;

    if (!send_all(reinterpret_cast<const uint8_t*>(&header), sizeof(header)) ||
        !send_all(data + offset, chunk_len)) {
      LOG_ERROR("OdinCommand::SendAlgoData: write failed at chunk %u/%u\n",
                i, chunk_count);
      return OdinResult::kSocketError;
    }
    offset += chunk_len;
  }
  return OdinResult::kOk;
}

// =============================================================================
// Calibration file fetch (synchronous, data_type=0x41).
//
// Spec: docs/protocol/PROTOCOL.md section 1.3 (0x41).
// One-shot blocking call: allocates a TCP port, starts a server, subscribes
// the module via ChannelConfig, waits for the complete file (multi-chunk
// reassembly handled by HandleCalibData), then unsubscribes and tears down.
// =============================================================================

OdinResult OdinCommand::FetchCalibrationFile(const std::string& save_path,
                                             CommandProgressCallback cb,
                                             uint32_t timeout_ms) {
  // Ensure only one background fetch runs at a time.
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    if (calib_thread_.joinable()) {
      LOG_WARN("OdinCommand::FetchCalibrationFile: previous fetch still running\n");
      return OdinResult::kInvalidArgument;
    }
    calib_thread_cancel_.store(false);
    calib_progress_cb_ = cb;
  }

  calib_thread_ = std::thread([this, save_path, cb, timeout_ms]() {
    FetchCalibrationFileThread(save_path, cb, timeout_ms);
  });
  return OdinResult::kOk;
}

void OdinCommand::FetchCalibrationFileThread(const std::string& save_path,
                                             CommandProgressCallback cb,
                                             uint32_t timeout_ms) {
  auto Finalize = [&cb](bool success) {
    if (cb) cb(success ? 100.0f : -1.0f);
  };

  // Step 1: Allocate port and start TCP server.
  uint16_t port = AllocateDstPort();
  if (port == 0) {
    LOG_WARN("OdinCommand::FetchCalibrationFileThread: port allocation failed\n");
    Finalize(false);
    return;
  }

  std::unique_ptr<TcpTransport> server(new TcpTransport());
  NetworkAddress local_addr(host_ip_, port);
  if (!server->Open(local_addr) || !server->StartReceiving()) {
    LOG_WARN("OdinCommand::FetchCalibrationFileThread: server open failed on port %u\n", port);
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
    Finalize(false);
    return;
  }

  // Register receive callback for chunk reassembly.
  server->SetReceiveCallback(
      [this](const uint8_t* data, size_t length, const ITransportAddress&) {
        HandleCalibData(data, length);
      });

  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    calib_server_ = std::move(server);
    calib_server_port_ = port;
    calib_reassembly_buf_.clear();
    calib_expected_chunks_ = 0;
    calib_received_chunks_ = 0;
    calib_frame_seq_ = 0;
    calib_file_received_ = false;
  }

  // Step 2: Subscribe the module to 0x41 (TCP).
  {
    auto payload = BuildChannelConfigPayloadRaw(0x41, OdinTransportMode::kTcp, port);
    OdinCommandSyncResponse response;
    std::lock_guard<std::mutex> cmd_lock(mutex_);
    if (!command_channel_) {
      LOG_WARN("OdinCommand::FetchCalibrationFileThread: not connected\n");
      CleanupCalibServer();
      Finalize(false);
      return;
    }
    if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig),
                                    payload, response, 1000)) {
      LOG_WARN("OdinCommand::FetchCalibrationFileThread: subscribe timed out\n");
      CleanupCalibServer();
      Finalize(false);
      return;
    }
    if (response.response.payload.empty() || response.response.payload[0] != 0) {
      LOG_WARN("OdinCommand::FetchCalibrationFileThread: subscribe rejected (ret=%u)\n",
               response.response.payload.empty() ? 255 : response.response.payload[0]);
      CleanupCalibServer();
      Finalize(false);
      return;
    }
  }

  // Step 3: Block until complete file arrives, timeout, or cancellation.
  {
    std::unique_lock<std::mutex> lock(calib_mutex_);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    constexpr uint32_t kCancelPollMs = 100;

    while (!calib_file_received_) {
      auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        LOG_WARN("OdinCommand::FetchCalibrationFileThread: timeout after %u ms\n", timeout_ms);
        lock.unlock();
        UnsubscribeCalibChannel();
        CleanupCalibServer();
        Finalize(false);
        return;
      }

      if (calib_thread_cancel_.load()) {
        LOG_WARN("OdinCommand::FetchCalibrationFileThread: cancelled\n");
        lock.unlock();
        UnsubscribeCalibChannel();
        CleanupCalibServer();
        Finalize(false);
        return;
      }

      auto next_deadline = now + std::chrono::milliseconds(kCancelPollMs);
      if (next_deadline > deadline) next_deadline = deadline;
      calib_cv_.wait_until(lock, next_deadline,
                           [this] { return calib_file_received_; });
    }
  }

  // Step 4: Save assembled payload to file and tear down.
  {
    std::vector<uint8_t> buf_copy;
    {
      std::lock_guard<std::mutex> lock(calib_mutex_);
      buf_copy = calib_reassembly_buf_;  // copy under lock, release before file I/O
    }
    std::ofstream file(save_path, std::ios::binary);
    if (!file.is_open()) {
      LOG_WARN("OdinCommand::FetchCalibrationFileThread: failed to open %s\n", save_path.c_str());
      UnsubscribeCalibChannel();
      CleanupCalibServer();
      Finalize(false);
      return;
    }
    file.write(reinterpret_cast<const char*>(buf_copy.data()), buf_copy.size());
    file.close();
    LOG_DEBUG("OdinCommand::FetchCalibrationFileThread: saved %zu bytes to %s\n",
             buf_copy.size(), save_path.c_str());
  }

  UnsubscribeCalibChannel();
  CleanupCalibServer();
  Finalize(true);
}

void OdinCommand::StopCalibThread() {
  calib_thread_cancel_.store(true);
  calib_cv_.notify_all();
  if (calib_thread_.joinable()) {
    calib_thread_.join();
  }
}

void OdinCommand::UnsubscribeCalibChannel() {
  auto payload = BuildChannelConfigPayloadRaw(0x41, OdinTransportMode::kTcp, 0);
  OdinCommandSyncResponse response;
  std::lock_guard<std::mutex> cmd_lock(mutex_);
  if (command_channel_) {
    command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig),
                               payload, response, 1000);
  }
}

void OdinCommand::CleanupCalibServer() {
  std::unique_ptr<TcpTransport> to_close;
  uint16_t port = 0;
  {
    std::lock_guard<std::mutex> lock(calib_mutex_);
    to_close = std::move(calib_server_);
    port = calib_server_port_;
    calib_server_port_ = 0;
    calib_reassembly_buf_.clear();
    calib_expected_chunks_ = 0;
    calib_received_chunks_ = 0;
    calib_progress_cb_ = nullptr;  // release callback reference
  }
  if (to_close) to_close->Close();
  if (port != 0) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
  }
}

namespace {
// v2 data frame header size (32 bytes, per protocol §2.1)
constexpr size_t kV2HeaderSize = sizeof(OdinDataFrameHeader);
}  // namespace

void OdinCommand::HandleCalibData(const uint8_t* data, size_t length) {
  // TCP stream may deliver partial frames; buffer and parse incrementally.
  // For simplicity, we accumulate and process complete v2 frames.
  std::lock_guard<std::mutex> calib_lock(calib_mutex_);
  // Append incoming bytes to a temporary stream buffer.
  // We reuse calib_reassembly_buf_ for partial chunk accumulation, so we
  // maintain a local static-ish stream buffer via a simple approach:
  // parse header, wait for full payload, process frame.
  //
  // Because TcpTransport delivers data from a single TCP connection in order,
  // and the module sends frames back-to-back, we can use a straightforward
  // state machine. For robustness we handle partial reads.

  // Static buffer is not thread-safe but HandleCalibData is only called from
  // the single TcpTransport receive thread, so it is safe.
  static thread_local std::vector<uint8_t> stream_buf;
  stream_buf.insert(stream_buf.end(), data, data + length);

  while (stream_buf.size() >= kV2HeaderSize) {
    const OdinDataFrameHeader* hdr =
        reinterpret_cast<const OdinDataFrameHeader*>(stream_buf.data());

    if (hdr->sof != OdinConst::kDataSof || hdr->version != OdinConst::kDataVersion) {
      LOG_WARN("OdinCommand::HandleCalibData: bad header (sof=0x%02X ver=%u), dropping %zu bytes\n",
               hdr->sof, hdr->version, stream_buf.size());
      stream_buf.clear();
      return;
    }

    size_t frame_size = kV2HeaderSize + hdr->payload_len;
    if (stream_buf.size() < frame_size) {
      break;  // Wait for more data
    }

    // Extract payload
    const uint8_t* payload = stream_buf.data() + kV2HeaderSize;
    uint16_t payload_len = hdr->payload_len;
    bool last_chunk = (hdr->flags & 0x01) != 0;

    // Reassembly logic
    if (hdr->chunk_count == 1 && last_chunk) {
      // Single-chunk frame: store payload and signal.
      calib_reassembly_buf_.assign(payload, payload + payload_len);
      calib_expected_chunks_ = 0;
      calib_received_chunks_ = 0;
      calib_file_received_ = true;
      calib_cv_.notify_one();
    } else {
      // Multi-chunk reassembly
      if (hdr->chunk_index == 0) {
        // New frame: reset reassembly state
        calib_reassembly_buf_.clear();
        calib_expected_chunks_ = hdr->chunk_count;
        calib_received_chunks_ = 0;
        calib_frame_seq_ = hdr->frame_seq;
      }
      // Append chunk payload
      calib_reassembly_buf_.insert(calib_reassembly_buf_.end(),
                                   payload, payload + payload_len);
      calib_received_chunks_++;

      // Report incremental progress once per complete chunk (0x41 multi-chunk path).
      // stream_buf guarantees the full frame (header + payload) is present before
      // this point, so progress is not counted on partial TCP deliveries.
      if (calib_progress_cb_ && calib_expected_chunks_ > 0) {
        float p = static_cast<float>(calib_received_chunks_) /
                  static_cast<float>(calib_expected_chunks_) * 99.0f;
        calib_progress_cb_(p);  // called under calib_mutex_; safe (no re-entrant lock)
      }

      if (last_chunk && calib_received_chunks_ == calib_expected_chunks_) {
        // Complete frame assembled — signal FetchCalibrationFile.
        calib_file_received_ = true;
        calib_cv_.notify_one();
        calib_expected_chunks_ = 0;
        calib_received_chunks_ = 0;
      }
    }

    // Remove processed frame from stream buffer
    stream_buf.erase(stream_buf.begin(), stream_buf.begin() + frame_size);
  }
}

// =============================================================================
// Telemetry uplink (device -> host, data_type=0x42).
//
// Spec: docs/protocol/PROTOCOL.md section 1.3. Host opens a TCP server,
// subscribes the module via ChannelConfig (data_type=0x42, TCP), the module
// connects in and pushes JSON frames at ~1 Hz. Each frame is self-contained
// (no chunk reassembly needed for single-chunk frames).
// =============================================================================

void OdinCommand::StartTelemetry() {
  // Caller (Connect) holds mutex_.
  uint16_t port = AllocateDstPort();
  if (port == 0) {
    LOG_WARN("OdinCommand: telemetry server port allocation failed; 0x42 disabled\n");
    return;
  }

  std::unique_ptr<TcpTransport> server(new TcpTransport());
  NetworkAddress local_addr(host_ip_, port);
  if (!server->Open(local_addr) || !server->StartReceiving()) {
    LOG_WARN("OdinCommand: telemetry server open failed on port %u; 0x42 disabled\n", port);
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
    return;
  }

  // Register TCP receive callback. Each v2 frame is self-contained.
  server->SetReceiveCallback(
      [this](const uint8_t* data, size_t length, const ITransportAddress&) {
        HandleTelemetryData(data, length);
      });

  {
    std::lock_guard<std::mutex> telem_lock(telemetry_mutex_);
    telemetry_server_ = std::move(server);
    telemetry_server_port_ = port;
  }

  // Subscribe the module to the telemetry uplink.
  if (command_channel_) {
    auto payload = BuildChannelConfigPayloadRaw(0x42, OdinTransportMode::kTcp, port);
    // Log outgoing subscribe request payload.
    {
      std::string hex;
      for (size_t i = 0; i < payload.size(); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", payload[i]);
        hex += buf;
      }
    }
    OdinCommandSyncResponse response;
    if (!command_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kChannelConfig),
                                    payload, response, 1000)) {
      LOG_WARN("OdinCommand: telemetry 0x42 subscribe timed out (firmware may not support it)\n");
    } else {
      // Log the raw response for diagnostics.
      std::string hex;
      for (size_t i = 0; i < response.response.payload.size(); ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", response.response.payload[i]);
        hex += buf;
      }

      if (response.response.payload.empty() || response.response.payload[0] != 0) {
        LOG_WARN("OdinCommand: telemetry 0x42 subscribe rejected (firmware may not support it)\n");
      } else {
        LOG_DEBUG("OdinCommand: telemetry 0x42 subscribed, port=%u\n", port);
      }
    }
  }
}

void OdinCommand::StopTelemetryServer() {
  // Caller (Disconnect) holds mutex_.
  std::unique_ptr<TcpTransport> to_close;
  uint16_t port = 0;
  {
    std::lock_guard<std::mutex> telem_lock(telemetry_mutex_);
    to_close = std::move(telemetry_server_);
    port = telemetry_server_port_;
    telemetry_server_port_ = 0;
  }
  if (to_close) to_close->Close();
  if (port != 0) {
    std::lock_guard<std::mutex> port_lock(port_mutex_);
    used_ports_.erase(port);
  }
}

void OdinCommand::RegisterTelemetryCallback(OdinTelemetryCallback cb, void* user_data) {
  std::lock_guard<std::mutex> telem_lock(telemetry_mutex_);
  telemetry_cb_ = cb;
  telemetry_user_data_ = user_data;
}

void OdinCommand::HandleTelemetryData(const uint8_t* data, size_t length) {
  // TCP stream parser: buffer and extract complete v2 frames.
  // Called from the single TcpTransport receive thread.
  static thread_local std::vector<uint8_t> stream_buf;
  stream_buf.insert(stream_buf.end(), data, data + length);

  while (stream_buf.size() >= kV2HeaderSize) {
    const OdinDataFrameHeader* hdr =
        reinterpret_cast<const OdinDataFrameHeader*>(stream_buf.data());

    if (hdr->sof != OdinConst::kDataSof || hdr->version != OdinConst::kDataVersion) {
      LOG_WARN("OdinCommand::HandleTelemetryData: bad header (sof=0x%02X ver=%u), dropping %zu bytes\n",
               hdr->sof, hdr->version, stream_buf.size());
      stream_buf.clear();
      return;
    }

    size_t frame_size = kV2HeaderSize + hdr->payload_len;
    if (stream_buf.size() < frame_size) {
      break;  // Wait for more data
    }

    // Extract payload and deliver
    const uint8_t* payload = stream_buf.data() + kV2HeaderSize;
    uint16_t payload_len = hdr->payload_len;

    // Parse JSON into structured OdinTelemetryData.
    // The device sends a null-terminated JSON string; ensure null-termination
    // for cJSON even if the TCP stream delivers it without the trailing '\0'.
    std::string json_str(reinterpret_cast<const char*>(payload), payload_len);
    OdinTelemetryData telem;
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (root) {
      cJSON* item;
      // Top-level
      if ((item = cJSON_GetObjectItem(root, "uptime_sec")) && cJSON_IsNumber(item))
        telem.uptime_s = static_cast<uint32_t>(item->valueint);

      // Nested: "cpu"."total"
      cJSON* cpu = cJSON_GetObjectItem(root, "cpu");
      if (cpu) {
        if ((item = cJSON_GetObjectItem(cpu, "total")) && cJSON_IsNumber(item))
          telem.cpu_pct = static_cast<uint32_t>(item->valueint);
      }

      // Nested: "memory"
      cJSON* mem = cJSON_GetObjectItem(root, "memory");
      if (mem) {
        if ((item = cJSON_GetObjectItem(mem, "used_kb"))  && cJSON_IsNumber(item))
          telem.mem_used_kb = static_cast<uint32_t>(item->valueint);
        if ((item = cJSON_GetObjectItem(mem, "total_kb")) && cJSON_IsNumber(item))
          telem.mem_total_kb = static_cast<uint32_t>(item->valueint);
        if ((item = cJSON_GetObjectItem(mem, "use_rate")) && cJSON_IsNumber(item))
          telem.mem_use_rate = static_cast<uint32_t>(item->valueint);
      }

      // Nested: "load_avg"
      cJSON* load = cJSON_GetObjectItem(root, "load_avg");
      if (load) {
        if ((item = cJSON_GetObjectItem(load, "1min"))  && cJSON_IsNumber(item))
          telem.load_1 = static_cast<float>(item->valuedouble);
        if ((item = cJSON_GetObjectItem(load, "5min"))  && cJSON_IsNumber(item))
          telem.load_5 = static_cast<float>(item->valuedouble);
        if ((item = cJSON_GetObjectItem(load, "15min")) && cJSON_IsNumber(item))
          telem.load_15 = static_cast<float>(item->valuedouble);
      }

      cJSON_Delete(root);
    } else {
      LOG_WARN("OdinCommand::HandleTelemetryData: JSON parse failed (%u bytes)\n", payload_len);
    }

    std::lock_guard<std::mutex> telem_lock(telemetry_mutex_);
    OdinTelemetryCallback cb = telemetry_cb_;
    void* ud = telemetry_user_data_;
    if (cb) {
      OdinTelemetryPacket pkt;
      pkt.frame_count = hdr->frame_seq;
      pkt.timestamp = hdr->timestamp;
      pkt.data = telem;
      cb(pkt, ud);
    }

    stream_buf.erase(stream_buf.begin(), stream_buf.begin() + frame_size);
  }
}

bool OdinCommand::SendHeartbeat(uint32_t interval_ms, uint32_t timeout_ms) {
  std::vector<uint8_t> payload(2);
  uint16_t interval = static_cast<uint16_t>(interval_ms);
  payload[0] = static_cast<uint8_t>(interval & 0xFF);
  payload[1] = static_cast<uint8_t>((interval >> 8) & 0xFF);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!heartbeat_channel_) {
    return false;
  }

  OdinCommandSyncResponse response;
  if (!heartbeat_channel_->SendSync(static_cast<uint16_t>(Odin2CmdId::kHeartbeat), 
                                    payload, response, timeout_ms)) {
    return false;
  }

  if (response.result != OdinResult::kOk) {
    return false;
  }

  const auto& ack_payload = response.response.payload;
  if (ack_payload.size() >= 5) {
    uint8_t ret_code = ack_payload[0];
    if (ret_code != 0) {
      return false;
    }
  }

  return true;
}

}  // namespace sdk
}  // namespace odin
