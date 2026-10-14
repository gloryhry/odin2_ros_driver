#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

/**
 * @brief HTTP/HTTPS file backend used internally by OdinCommand.
 *
 * Self-contained byte-level HTTP/HTTPS file transfer:
 *   - probe http://<ip>:<http_port> and fall back to https://<ip>:<https_file_port>
 *     for non-firmware files;
 *   - chunked OTA upload + state-machine polling for firmware;
 *   - translation of OdinFileType to remote resource name.
 *
 * Not a public service - OdinCommand composes it.
 */
class HttpFileBackend {
 public:
  using ProgressCallback = std::function<void(float)>;

  HttpFileBackend();
  ~HttpFileBackend();

  bool Connect(const DiscoveredDevice& device);
  void Disconnect();
  bool IsConnected() const { return connected_.load(); }

  OdinResult Upload(OdinFileType type, const std::string& local_path,
                    ProgressCallback cb);
  OdinResult Download(OdinFileType type, const std::string& save_path,
                      ProgressCallback cb);
  OdinResult UploadFirmware(const std::string& firmware_path,
                            ProgressCallback cb,
                            int timeout_s = 300);

  // Optional configuration.
  void SetHttpPort(uint16_t port) { http_port_ = port; }

 private:
  // Maps a semantic file type to the remote resource name.
  // Returns an empty string if the (type, direction) tuple is unsupported.
  static std::string FileTypeToRemoteName(OdinFileType type, bool for_upload);

  // Probe http://<device_ip_>:<http_port_>. Writes the canonical http base URL
  // into 'base_url' on success.
  bool ProbeHttpBaseUrl(std::string& base_url) const;

  // Returns the legacy https://<ip>:<port> URL used by the HTTPS fallback.
  // Returns empty string when https_file_port_ is unset.
  std::string BuildHttpsUrl() const;

  // Byte-level dispatch over HTTP (port 8080-style API).
  bool HttpDownload(const std::string& base_url, const std::string& remote_name,
                    const std::string& local_path,
                    ProgressCallback cb, std::string& err);
  bool HttpUpload(const std::string& base_url, const std::string& local_path,
                  const std::string& remote_name,
                  ProgressCallback cb, std::string& err);

  // Byte-level dispatch over HTTPS (legacy URL-arg API).
  bool HttpsDownload(const std::string& base_url, const std::string& remote_name,
                     const std::string& local_path,
                     ProgressCallback cb, std::string& err);
  bool HttpsUpload(const std::string& base_url, const std::string& local_path,
                   const std::string& remote_name,
                   ProgressCallback cb, std::string& err);

  std::string device_ip_;
  uint16_t http_port_ = 8080;
  uint16_t https_file_port_ = 0;  // Set by Odin2Device once stream config allocates it.
  std::atomic<bool> connected_{false};
};

}  // namespace sdk
}  // namespace odin
