#include "http_file_backend.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <thread>

#include "http_client.h"
#include "logger.h"

namespace odin {
namespace sdk {

namespace {

constexpr int kHttpProbeTimeoutMs = 2000;
constexpr int kHttpHealthTimeoutMs = 3000;

// -----------------------------------------------------------------------------
// Thread-local bridge: http::handle_opts uses plain C function pointers, while
// the service exposes std::function progress callbacks. We stash the active
// std::function in TLS so the C trampoline can reach it for the duration of
// a single transfer call.
// -----------------------------------------------------------------------------
thread_local HttpFileBackend::ProgressCallback g_progress_cb;
thread_local std::string g_resolved_download_path;

int ProgressBytesBridge(uint64_t current, uint64_t total) {
  if (g_progress_cb && total > 0) {
    g_progress_cb(static_cast<float>(current) * 100.0f /
                  static_cast<float>(total));
  }
  return 0;
}

int CaptureResolvedPath(const char* resolved, const char* /*ignored*/) {
  if (resolved) g_resolved_download_path = resolved;
  return 0;
}

// Maps an OTA state name + device-reported sub-progress to overall progress
// in [0..100]. Returns -1 for the FAILED terminal state.
float MapOtaStateToProgress(const std::string& state, int device_progress) {
  if (state == "UPLOADING")      return 50.0f * device_progress / 100.0f;
  if (state == "VERIFYING")      return 50.0f + 10.0f * device_progress / 100.0f;
  if (state == "INSTALLING_MCU") return 60.0f + 10.0f * device_progress / 100.0f;
  if (state == "INSTALLING_SOC") return 70.0f + 10.0f * device_progress / 100.0f;
  if (state == "REBOOTING")      return 80.0f + 15.0f * device_progress / 100.0f;
  if (state == "POST_VERIFY")    return 95.0f + 5.0f * device_progress / 100.0f;
  if (state == "DONE")           return 100.0f;
  if (state == "FAILED")         return -1.0f;
  return 0.0f;
}

bool IsOtaInstallingState(const std::string& state) {
  return state == "VERIFYING" || state == "INSTALLING_MCU" ||
         state == "INSTALLING_SOC" || state == "REBOOTING" ||
         state == "POST_VERIFY";
}

// RAII guard to clear the thread-local progress slot.
struct TlsProgressGuard {
  explicit TlsProgressGuard(HttpFileBackend::ProgressCallback cb) {
    g_progress_cb = std::move(cb);
    g_resolved_download_path.clear();
  }
  ~TlsProgressGuard() {
    g_progress_cb = nullptr;
  }
};

}  // namespace

HttpFileBackend::HttpFileBackend() = default;
HttpFileBackend::~HttpFileBackend() { Disconnect(); }

bool HttpFileBackend::Connect(const DiscoveredDevice& device) {
  const std::string& ip = device.network.ip_address;
  if (ip.empty()) {
    LOG_ERROR("HttpFileBackend::Connect: device has no IP\n");
    return false;
  }
  device_ip_ = ip;
  connected_.store(true);
  LOG_INFO("HttpFileBackend: bound to %s (http=%u, https_file=%u)\n",
           device_ip_.c_str(), http_port_, https_file_port_);
  return true;
}

void HttpFileBackend::Disconnect() {
  connected_.store(false);
  device_ip_.clear();
}

std::string HttpFileBackend::FileTypeToRemoteName(OdinFileType type,
                                                     bool for_upload) {
  switch (type) {
    case OdinFileType::kCalibrationFile: return "calibration";
    case OdinFileType::kRelocationMap:   return "relocation_map";
    case OdinFileType::kDevLogFile:      return for_upload ? "" : "logs";
    case OdinFileType::kFirmware:        return "firmware";
    default:                             return "";
  }
}

bool HttpFileBackend::ProbeHttpBaseUrl(std::string& base_url) const {
  std::string url = "http://" + device_ip_ + ":" + std::to_string(http_port_);
  http::Client probe(url);
  if (!probe.health_check(kHttpProbeTimeoutMs)) return false;
  base_url = url;
  return true;
}

std::string HttpFileBackend::BuildHttpsUrl() const {
  if (https_file_port_ == 0) return "";
  return "https://" + device_ip_ + ":" + std::to_string(https_file_port_);
}

// =============================================================================
// HTTP byte-level dispatch (port 8080-style API)
// =============================================================================

bool HttpFileBackend::HttpDownload(const std::string& base_url,
                                   const std::string& remote_name,
                                   const std::string& local_path,
                                   ProgressCallback cb,
                                   std::string& err) {
  namespace fs = std::filesystem;
  std::string output_dir = fs::path(local_path).parent_path().string();
  if (output_dir.empty()) output_dir = ".";

  http::Client client(base_url);
  http::handle_opts opts{};
  if (cb) opts.progress_download_callback = ProgressBytesBridge;
  opts.after_download_hook = CaptureResolvedPath;
  client.set_handle_opts(opts);

  TlsProgressGuard tls_guard(cb);

  bool ok = false;
  if (remote_name == "logs") {
    ok = client.download_log(output_dir);
  } else if (remote_name == "calibration") {
    ok = client.download_calib(output_dir);
  } else if (remote_name == "relocation_map") {
    ok = client.download_map(output_dir);
  } else {
    err = "Unknown remote file: " + remote_name;
    return false;
  }
  if (!ok) {
    err = "HTTP download failed: " + remote_name;
    return false;
  }

  // Rename downloaded file to the caller-expected path if they differ.
  if (!g_resolved_download_path.empty() &&
      g_resolved_download_path != local_path) {
    std::error_code ec;
    fs::rename(g_resolved_download_path, local_path, ec);
    if (ec) {
      err = "rename failed: " + ec.message();
      return false;
    }
  }
  return true;
}

bool HttpFileBackend::HttpUpload(const std::string& base_url,
                                 const std::string& local_path,
                                 const std::string& remote_name,
                                 ProgressCallback cb,
                                 std::string& err) {
  http::Client client(base_url);
  http::handle_opts opts{};
  if (cb) opts.progress_upload_callback = ProgressBytesBridge;
  client.set_handle_opts(opts);

  TlsProgressGuard tls_guard(cb);

  bool ok = false;
  if (remote_name == "calibration") {
    ok = client.upload_calib(local_path);
  } else {
    err = "Unsupported HTTP upload type: " + remote_name;
    return false;
  }
  if (!ok) err = "HTTP upload failed: " + remote_name;
  return ok;
}

// =============================================================================
// HTTPS byte-level dispatch (legacy URL-arg API)
// =============================================================================

bool HttpFileBackend::HttpsDownload(const std::string& base_url,
                                    const std::string& remote_name,
                                    const std::string& local_path,
                                    ProgressCallback cb,
                                    std::string& err) {
  namespace fs = std::filesystem;
  std::string output_dir = fs::path(local_path).parent_path().string();
  if (output_dir.empty()) output_dir = ".";

  http::Client client;
  http::handle_opts opts{};
  if (cb) opts.progress_download_callback = ProgressBytesBridge;
  opts.after_download_hook = CaptureResolvedPath;
  client.set_handle_opts(opts);

  TlsProgressGuard tls_guard(cb);

  bool ok = false;
  if (remote_name == "logs") {
    ok = client.download_device_logs(base_url, output_dir);
  } else if (remote_name == "calibration") {
    ok = client.download_calibration(base_url, output_dir);
  } else if (remote_name == "relocation_map") {
    ok = client.download_relocation_map(base_url, output_dir);
  } else {
    err = "Unknown remote file: " + remote_name;
    return false;
  }
  if (!ok) {
    err = "HTTPS download failed: " + remote_name;
    return false;
  }

  if (!g_resolved_download_path.empty() &&
      g_resolved_download_path != local_path) {
    std::error_code ec;
    fs::rename(g_resolved_download_path, local_path, ec);
    if (ec) {
      err = "rename failed: " + ec.message();
      return false;
    }
  }
  return true;
}

bool HttpFileBackend::HttpsUpload(const std::string& base_url,
                                  const std::string& local_path,
                                  const std::string& remote_name,
                                  ProgressCallback cb,
                                  std::string& err) {
  http::Client client;
  http::handle_opts opts{};
  if (cb) opts.progress_upload_callback = ProgressBytesBridge;
  client.set_handle_opts(opts);

  TlsProgressGuard tls_guard(cb);

  bool ok = false;
  if (remote_name == "firmware") {
    ok = client.upload_firmware_and_OTA(base_url, local_path);
  } else {
    err = "Unsupported HTTPS upload type: " + remote_name;
    return false;
  }
  if (!ok) err = "HTTPS upload failed: " + remote_name;
  return ok;
}

// =============================================================================
// Public API
// =============================================================================

OdinResult HttpFileBackend::Upload(OdinFileType type,
                                   const std::string& local_path,
                                   ProgressCallback cb) {
  if (!connected_.load()) return OdinResult::kNotInitialized;
  if (type == OdinFileType::kFirmware) return UploadFirmware(local_path, cb);

  std::string remote = FileTypeToRemoteName(type, /*for_upload=*/true);
  if (remote.empty()) {
    LOG_ERROR("HttpFileBackend::Upload: unsupported type %d\n",
              static_cast<int>(type));
    return OdinResult::kInvalidArgument;
  }

  std::string err;
  std::string http_url;
  if (ProbeHttpBaseUrl(http_url)) {
    if (HttpUpload(http_url, local_path, remote, cb, err)) return OdinResult::kOk;
  } else {
    std::string https_url = BuildHttpsUrl();
    if (https_url.empty()) {
      LOG_ERROR("HttpFileBackend::Upload: HTTPS fallback file port not set\n");
      return OdinResult::kCommunicationError;
    }
    if (HttpsUpload(https_url, local_path, remote, cb, err)) return OdinResult::kOk;
  }
  LOG_ERROR("HttpFileBackend::Upload(%s): %s\n", remote.c_str(), err.c_str());
  return OdinResult::kCommunicationError;
}

OdinResult HttpFileBackend::Download(OdinFileType type,
                                     const std::string& save_path,
                                     ProgressCallback cb) {
  if (!connected_.load()) return OdinResult::kNotInitialized;

  std::string remote = FileTypeToRemoteName(type, /*for_upload=*/false);
  if (remote.empty()) {
    LOG_ERROR("HttpFileBackend::Download: unsupported type %d\n",
              static_cast<int>(type));
    return OdinResult::kInvalidArgument;
  }

  std::string err;
  bool ok = false;
  std::string http_url;
  if (ProbeHttpBaseUrl(http_url)) {
    ok = HttpDownload(http_url, remote, save_path, cb, err);
  } else {
    std::string https_url = BuildHttpsUrl();
    if (https_url.empty()) {
      LOG_ERROR("HttpFileBackend::Download: HTTPS fallback file port not set\n");
      return OdinResult::kCommunicationError;
    }
    ok = HttpsDownload(https_url, remote, save_path, cb, err);
  }
  if (!ok) {
    LOG_ERROR("HttpFileBackend::Download(%s): %s\n", remote.c_str(), err.c_str());
    return OdinResult::kCommunicationError;
  }
  if (cb) cb(100.0f);
  return OdinResult::kOk;
}

OdinResult HttpFileBackend::UploadFirmware(const std::string& firmware_path,
                                           ProgressCallback cb,
                                           int timeout_s) {
  if (!connected_.load()) return OdinResult::kNotInitialized;

  // Resolve HTTP endpoint. Some legacy devices expose HTTP on the file port
  // instead of the canonical http_port_, so we probe both.
  uint16_t http_port = http_port_;
  {
    std::string probe_url = "http://" + device_ip_ + ":" + std::to_string(http_port_);
    http::Client probe(probe_url);
    if (!probe.health_check(kHttpProbeTimeoutMs) && https_file_port_ != 0 &&
        https_file_port_ != http_port_) {
      http_port = https_file_port_;
    }
  }

  std::string base_url = "http://" + device_ip_ + ":" + std::to_string(http_port);
  http::Client client(base_url);

  if (!client.health_check(kHttpHealthTimeoutMs)) {
    LOG_WARN("OdinHttpFileService::UploadFirmware: HTTP not reachable, fallback to HTTPS\n");
    std::string https_url = BuildHttpsUrl();
    if (https_url.empty()) {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: HTTPS fallback file port not set\n");
      return OdinResult::kCommunicationError;
    }
    std::string err;
    if (!HttpsUpload(https_url, firmware_path, "firmware", cb, err)) {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: %s\n", err.c_str());
      return OdinResult::kCommunicationError;
    }
    return OdinResult::kOk;
  }

  // Inspect current OTA state and reset stale terminal states.
  http::OtaStatus status;
  bool ota_in_progress = false;
  if (client.ota_status(status)) {
    if (IsOtaInstallingState(status.state)) {
      ota_in_progress = true;
      LOG_INFO("OdinHttpFileService::UploadFirmware: OTA already in progress (%s)\n",
               status.state.c_str());
    } else if (status.state == "UPLOADING" || status.state == "DONE" ||
               status.state == "FAILED") {
      if (!client.ota_reset()) {
        LOG_ERROR("OdinHttpFileService::UploadFirmware: ota_reset failed\n");
        return OdinResult::kCommunicationError;
      }
    }
  }

  if (!ota_in_progress) {
    TlsProgressGuard tls_guard(cb);
    auto upload_progress = [](uint64_t current, uint64_t total) -> int {
      if (g_progress_cb && total > 0) {
        g_progress_cb(50.0f * static_cast<float>(current) /
                      static_cast<float>(total));
      }
      return 0;
    };

    if (!client.ota_upload(firmware_path, upload_progress)) {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: ota_upload failed\n");
      return OdinResult::kCommunicationError;
    }
    if (!client.ota_trigger()) {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: ota_trigger failed\n");
      return OdinResult::kCommunicationError;
    }
  }

  // Poll OTA status until a terminal state is reached. During the reboot
  // phase the HTTP server is down for a while, so we silence HTTP error logs
  // until the device is reachable again.
  auto start_time = std::chrono::steady_clock::now();
  std::string last_state;
  bool in_reboot_phase = false;

  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time);
    if (elapsed.count() >= timeout_s) {
      if (in_reboot_phase) http::log_set_level(http::kLogInfo);
      LOG_ERROR("OdinHttpFileService::UploadFirmware: timeout\n");
      return OdinResult::kTimeout;
    }

    if (!client.ota_status(status)) {
      if (!in_reboot_phase) {
        in_reboot_phase = true;
        http::log_set_level(http::kLogNone);
        LOG_INFO("OdinHttpFileService::UploadFirmware: device rebooting...\n");
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));
      continue;
    }

    if (in_reboot_phase) {
      in_reboot_phase = false;
      http::log_set_level(http::kLogInfo);
      LOG_INFO("OdinHttpFileService::UploadFirmware: device reconnected\n");
    }

    if (status.state != last_state) {
      LOG_INFO("OdinHttpFileService::UploadFirmware: state=%s\n", status.state.c_str());
      last_state = status.state;
    }

    float progress = MapOtaStateToProgress(status.state, status.progress);
    if (progress < 0) {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: OTA failed: %s\n", status.error.c_str());
      return OdinResult::kUnknownError;
    }
    if (cb) cb(progress);

    if (status.state == "DONE") {
      if (cb) cb(100.0f);
      LOG_INFO("OdinHttpFileService::UploadFirmware: OTA completed\n");
      return OdinResult::kOk;
    }
    if (status.state == "FAILED") {
      LOG_ERROR("OdinHttpFileService::UploadFirmware: device reported failure: %s\n",
                status.error.c_str());
      return OdinResult::kUnknownError;
    }

    int sleep_ms = (status.state == "REBOOTING") ? 5000 : 2000;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  }
}

}  // namespace sdk
}  // namespace odin
