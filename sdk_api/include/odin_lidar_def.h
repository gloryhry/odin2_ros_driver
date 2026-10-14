#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace odin {
namespace sdk {

// ============================================================================
// Basic Types
// ============================================================================

using OdinDeviceHandle = uint32_t;
constexpr OdinDeviceHandle kInvalidDeviceHandle = 0;

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief SDK API result codes
 */
enum class OdinResult {
  kOk = 0,              ///< Operation successful
  kNotInitialized,      ///< SDK not initialized or device not connected
  kInvalidArgument,     ///< Invalid argument
  kSocketError,         ///< Network socket error
  kTimeout,             ///< Operation timed out
  kStopped,             ///< Operation stopped
  kFirmwareIllegal,     ///< Invalid firmware file
  kCommunicationError,  ///< Communication error
  kUnknownError         ///< Unknown error
};

/**
 * @brief Device connection type
 */
enum class MTConnectionType : uint8_t {
  kEthernet = 0,   ///< Ethernet (UDP/TCP)
  kUsb = 1,        ///< USB connection
  kUnknown = 0xFF  ///< Unknown connection type
};

/**
 * @brief Device operating mode
 */
enum class OdinDeviceMode : uint8_t {
  kStandby = 0,    ///< Standby mode, stops data streaming
  kNormal = 1,     ///< Normal operating mode, starts data streaming
  kUpgrade = 2,    ///< Upgrade mode
  kUnknown = 0xFF  ///< Unknown mode
};

/**
 * @brief LiDAR sensor mode (cmd_id=0x05 SetSensorMode / 0x09 GetSensorMode)
 */
enum class OdinSensorMode : uint8_t {
  kHDR = 0,       ///< HDR mode
  kHighPeak = 1,  ///< High Peak mode
  kLowPeak = 2,   ///< Low Peak mode
  kAdaptive = 3,  ///< Adaptive mode
};

/**
 * @brief Device file type
 */
enum class OdinFileType : uint8_t {
  kRelocationMap = 0,    ///< Relocation map file
  kCalibrationFile = 1,  ///< Camera calibration file
  kDevLogFile = 2,       ///< Device log file
  kFirmware = 3,         ///< Firmware/OTA file
  kUnknown = 0xFF        ///< Unknown file type
};

enum class OdomSourceType : uint8_t {
  kOdom2ImuLow   = 0,  ///< Odom → IMU low-freq
  kOdom2ImuHigh  = 1,  ///< Odom → IMU high-freq
};

/**
 * @brief Data channel type for ChannelConfig command (0x06 SetChannelConfig)
 *
 * Segmented numbering — each segment reserves 16 slots (0x10 per group):
 *   0x00-0x0F  Point cloud  (kRawPoint=0x00, kSlamPoint=0x01)
 *   0x10-0x1F  Image        (kImage0=0x10, kImage1=0x11)
 *   0x20-0x2F  IMU          (kImu=0x20)
 *   0x30-0x3F  Odometry     (kOdomLF=0x30 low-freq, kOdomHF=0x31 high-freq, kTf=0x32)
 *   0x40-0x4F  Function     (kAlgoDownlink=0x40, kCalibFile=0x41, kTelemetry=0x42)
 */
enum class OdinDataChannel : uint8_t {
  // 0x00-0x0F: Point cloud
  kRawPoint  = 0x00,  ///< Raw point cloud (host <- device)
  kSlamPoint = 0x01,  ///< SLAM point cloud (host <- device)

  // 0x10-0x1F: Image
  kImage0 = 0x10,     ///< Left camera image/video stream (host <- device)
  kImage1 = 0x11,     ///< Right camera image/video stream (host <- device)

  // 0x20-0x2F: IMU
  kImu = 0x20,        ///< IMU 6/9-axis samples (host <- device)

  // 0x30-0x3F: Odometry / TF
  kOdomLF   = 0x30,     ///< Odometry low-freq  (odom_lf, host <- device)
  kOdomHF = 0x31,     ///< Odometry high-freq (odom_hf, host <- device)
  kTf = 0x32,         ///< TF transforms
  // 0x40-0x4F: Function channels
  kAlgoDownlink = 0x40,  ///< Algorithm downlink transparent (host -> device)
  kCalibFile    = 0x41,  ///< Calibration file download (host <- device)
  kTelemetry    = 0x42,  ///< Telemetry JSON ~1 Hz (host <- device)
};

/**
 * @brief Transport mode for data channel
 */
enum class OdinTransportMode : uint8_t {
  kUdp = 0,  ///< UDP transport (default)
  kTcp = 1,  ///< TCP transport
};

/**
 * @brief Unified data format for all sensor types
 *
 * Image formats use bitmask encoding (can be OR-combined):
 *   0x01=NV12 | 0x02=MJPEG | 0x04=H264 | 0x08=H265
 * Other sensor types use fixed segment values:
 * - RawPoint (0x10): XYZIC (uint16 compressed coords)
 * - SlamPoint (0x20): XYZRGBA (uint16 compressed coords)
 * - IMU (0x30): 6-axis
 * - Odom (0x40): Standard
 * - TF (0x50): Standard TF
 */
enum class OdinDataFormat : uint8_t {
  // Image formats (bitmask, single-bit per codec)
  kImageNv12  = 0x01,   ///< NV12 raw data passthrough (no encoding)
  kImageMjpeg = 0x02,   ///< MJPEG (JPEG compressed)
  kImageH264  = 0x04,   ///< H.264 encoded video stream
  kImageH265  = 0x08,   ///< H.265 encoded video stream

  // RawPoint formats (0x10-0x1F)
  kRawPointXyzic = 0x10,  ///< XYZ + Intensity + Confidence (uint16 compressed coords, 8B per point)

  // SlamPoint formats (0x20-0x2F)
  kSlamPointXyzrgba = 0x20,  ///< XYZ + RGBA (uint16 compressed coords, 10B per point)

  // IMU formats (0x30-0x3F)
  kImu6Axis = 0x30,     ///< Gyro + Acc (6-axis)

  // Odom formats (0x40-0x4F)
  kOdomStandard = 0x40, ///< Standard odometry

  // TF formats (0x50-0x5F)
  kTfStandard = 0x50,   ///< Standard TF transform

  kUnknown = 0xFF
};

/**
 * @brief Stream configuration - a valid combination of resolution, fps, and format
 * 
 * Fields set to 0 or kUnknown will not be sent to device (device uses default).
 * Only set the fields you want to configure.
 * Image format uses OdinDataFormat::kImageXxx series
 */
struct OdinStreamCfg {
  uint16_t width = 0;           ///< Resolution width (0 = not sent, use device default)
  uint16_t height = 0;          ///< Resolution height (0 = not sent, use device default)
  uint16_t fps = 0;             ///< Frame rate (0 = not sent, use device default)
  OdinDataFormat format = OdinDataFormat::kUnknown;  ///< Image format (kUnknown = not sent)
};

/**
 * @brief Sensor capability - supported modes for a data channel
 */
struct OdinSensorCapability {
  OdinDataChannel channel = OdinDataChannel::kRawPoint;  ///< Channel type
  std::vector<OdinStreamCfg> modes;                       ///< Supported mode list
};

// ============================================================================
// Wire-format data types (packed)
// ============================================================================

#pragma pack(push, 1)

template <typename T>
struct OdinRawPoint {
  T x = T(0);
  T y = T(0);
  T z = T(0);
  uint8_t intensity = 0;
  uint8_t confidence = 0;
};


template <typename T>
struct OdinSlamPoint {
  T x = T(0);
  T y = T(0);
  T z = T(0);
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  uint8_t a = 0;
};


/// @brief Single IMU sample on the wire (v2, 24 bytes: gyro + acc only).
/// @details Per-sample timestamp/sequence were removed in v2; the timestamp is
///          reconstructed by the assembler as `header.timestamp + j*interval_ns`
///          and delivered via `OdinImuPacket.timestamp`.
struct OdinImuSample {
  float gyro_x = 0.0f;  ///< Angular velocity X (rad/s)
  float gyro_y = 0.0f;  ///< Angular velocity Y (rad/s)
  float gyro_z = 0.0f;  ///< Angular velocity Z (rad/s)
  float acc_x = 0.0f;   ///< Linear acceleration X (m/s^2)
  float acc_y = 0.0f;   ///< Linear acceleration Y (m/s^2)
  float acc_z = 0.0f;   ///< Linear acceleration Z (m/s^2)
};
static_assert(sizeof(OdinImuSample) == 24, "OdinImuSample (v2) must be 24 bytes");

struct OdinOdomData {
  float pos[3] = {0};
  float orient[4] = {0};
  float linear_velocity[3] = {0};
  float angular_velocity[3] = {0};
  float cov[18] = {0};
  uint8_t type = 0;  ///< Odom source type (@see OdomSourceType)
};
static_assert(sizeof(OdinOdomData) == 125, "Unexpected OdinOdomData size");

#pragma pack(pop)

// ============================================================================
// Assembled Data Packets (delivered to user via callbacks)
// ============================================================================

/// @brief Assembled raw point-cloud frame delivered to the user.
///
/// Protocol-level fields (version, length, chunk_count, data_type) are handled
/// internally by the SDK and not exposed. Points are already converted from
/// wire uint16 to float (with -30000 offset applied to y/z).
/// Use points.size() for the number of points.
struct OdinPointCloudPacket {
  uint32_t frame_count = 0;      ///< Frame sequence number (for drop detection)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<OdinRawPoint<float>> points;  ///< Float point data (ready to use)
};

/// @brief Assembled SLAM point-cloud frame delivered to the user.
///
/// Separate from OdinPointCloudPacket because SLAM points carry RGBA instead
/// of intensity/confidence. Points are already converted from wire uint16
/// to float and coordinate-transformed by the SLAM-Odom synchronizer.
struct OdinSlamPacket {
  uint32_t frame_count = 0;      ///< Frame sequence number (for drop detection)
  uint64_t timestamp = 0;        ///< Frame timestamp in ns (raw hardware monotonic)
  std::vector<OdinSlamPoint<float>> points;  ///< Float SLAM point data (ready to use)
};

/// @brief Unified camera frame packet delivered to the user (wire protocol v2).
///
/// Covers all image/video channel formats on kImage0/kImage1: MJPEG, H.264,
/// and any future encoded formats. The `format` field is the sole discriminant;
/// downstream consumers dispatch on it (e.g. via switch) without casts.
///
/// Field semantics by format:
///   - All formats : format, timestamp, payload are always valid.
///   - kImageMjpeg : frame_count carries the v2 frame_seq (for drop detection).
///                   width/height from the StartStream cfg passed by the caller
///                   (0 if not configured, same convention as H264).
///                   is_keyframe = true (every JPEG frame is independently decodable).
///   - kImageH264  : frame_count carries the v2 frame_seq for drop detection.
///                   width/height from the StartStream cfg passed by the caller.
///                   is_keyframe set by SDK via NAL unit scan (IDR/SPS detection).
struct OdinCameraPacket {
  OdinDataFormat   format      = OdinDataFormat::kUnknown; ///< Codec/format tag
  uint64_t         timestamp   = 0;    ///< Frame timestamp in ns (raw hardware monotonic)
  uint32_t         frame_count = 0;    ///< Frame sequence number from device
  uint16_t         width       = 0;    ///< Frame width
  uint16_t         height      = 0;    ///< Frame height
  bool             is_keyframe = true; ///< only use in stream: true if IDR/SPS NAL present; MJPEG: always true
  std::vector<uint8_t> payload;        ///< Encoded bytes: JPEG SOI..EOI or H.264 Annex-B AU
};

/// @brief IMU data packet delivered to the user.
/// Protocol-level fields are handled internally and not exposed.
/// Use samples.size() for the number of samples.
struct OdinImuPacket {
  uint64_t timestamp = 0;       ///< Reconstructed per-sample timestamp in ns
  std::vector<OdinImuSample> samples;  ///< IMU samples (ready to use)
};

/// @brief Odometry data packet delivered to the user.
/// Protocol-level fields are handled internally and not exposed.
/// Use data.size() for the number of entries.
struct OdinOdomPacket {
  uint32_t frame_count = 0;     ///< Frame sequence number (for drop detection)
  uint64_t timestamp = 0;       ///< Frame timestamp in ns
  std::vector<OdinOdomData> data;  ///< Odom data entries (ready to use)
};

/// @brief Parsed telemetry data from the device (0x42 channel, ~1 Hz).
///
/// Fields correspond to the JSON keys defined in the protocol:
/// ```json
/// {"uptime_s":123,"cpu_pct":45,"mem_used_kb":1024,"mem_total_kb":2048,
///  "mem_use_rate":50,"load_1":0.50,"load_5":0.45,"load_15":0.42}
/// ```
/// The SDK parses the JSON internally; callers receive structured data.
struct OdinTelemetryData {
  uint32_t uptime_s      = 0;    ///< Device uptime in seconds
  uint32_t cpu_pct       = 0;    ///< CPU usage percentage (0-100)
  uint32_t mem_used_kb   = 0;    ///< Memory used in kilobytes
  uint32_t mem_total_kb  = 0;    ///< Total memory in kilobytes
  uint32_t mem_use_rate  = 0;    ///< Memory usage rate percentage (0-100)
  float    load_1        = 0.0f; ///< System load average (1 min)
  float    load_5        = 0.0f; ///< System load average (5 min)
  float    load_15       = 0.0f; ///< System load average (15 min)
};

/// @brief Telemetry packet delivered via 0x42 channel (device -> host, ~1 Hz).
/// Each frame is self-contained; no chunk reassembly is needed.
/// The SDK parses the device JSON into structured OdinTelemetryData.
struct OdinTelemetryPacket {
  uint32_t frame_count = 0;         ///< Frame sequence number (v2 frame_seq)
  uint64_t timestamp = 0;           ///< Frame timestamp in ns (raw hardware monotonic)
  OdinTelemetryData data;           ///< Parsed telemetry fields
};

/// @brief Assembled TF frame delivered to the user.
///
/// Mirrors ROS2 TransformStamped: `frame_id` is the parent frame,
/// `child_frame_id` is the child frame, both delivered as std::string
/// for direct assignment into ROS message fields.
/// Protocol-level fields (version, length, raw wire payload) are not exposed.
struct OdinTfPacket {
  uint32_t frame_count = 0;                  ///< Frame sequence number (for drop detection)
  uint64_t timestamp = 0;                    ///< Frame timestamp in ns (raw hardware monotonic)
  float translation[3] = {0.0f, 0.0f, 0.0f}; ///< Parsed translation (m)
  float rotation[4] = {0.0f, 0.0f, 0.0f, 1.0f}; ///< Parsed quaternion (x, y, z, w)
  std::string frame_id;                      ///< Parent frame id (parsed string)
  std::string child_frame_id;                ///< Child frame id (parsed string)
};

/**
 * @brief Network attribute configuration
 *
 * Used for getting/setting device network configuration via HTTP API.
 * Can be embedded in DiscoveredDevice or used standalone.
 */
struct NetworkAttribute {
  std::string ip_address;  ///< Device IP address
  std::string gateway;     ///< Gateway address
  std::string netmask;     ///< Subnet mask
  std::string mac;         ///< Device MAC address
  bool dhcp = false;       ///< DHCP enabled
  bool ptp = false;        ///< PTP enabled
};

/**
 * @brief Discovered device information
 *
 * Contains device identity and network configuration.
 * Network settings are grouped in the 'network' member for easy access.
 */
struct DiscoveredDevice {
  std::string sn;                ///< Device serial number
  std::string model;             ///< Device model name (e.g., "Odin-L1")
  std::string host_ip;           ///< Host interface IP that discovered this device

  NetworkAttribute network;      ///< Network configuration (IP, gateway, netmask, MAC, DHCP, PTP)

  MTConnectionType connection = MTConnectionType::kEthernet;  ///< Connection type
};

// ============================================================================
// Callback Type Definitions
// ============================================================================

/** @brief Raw point cloud callback type */
using OdinPointCloudCallback = void (*)(const OdinPointCloudPacket &packet, void *client_data);
/** @brief SLAM point cloud callback type */
using OdinSlamCallback = void (*)(const OdinSlamPacket &packet, void *client_data);
/** @brief Camera frame callback type (kImage0/kImage1 channel, any format: MJPEG, H.264, etc.) */
using OdinCameraCallback = void (*)(const OdinCameraPacket &packet, void *client_data);
/** @brief IMU data callback type */
using OdinImuCallback = void (*)(const OdinImuPacket &packet, void *client_data);
/** @brief Odometry data callback type (with odom source type) */
using OdinOdomCallback = void (*)(const OdinOdomPacket &packet, OdomSourceType odom_type,
                                  void *client_data);
/** @brief TF transform callback type */
using OdinTfCallback = void (*)(const OdinTfPacket &packet, void *client_data);

/** @brief Upgrade progress callback type */
using OdinUpgradeProgressCallback = void (*)(OdinDeviceHandle device, float progress,
                                             void *client_data);

/** @brief Telemetry data callback type (0x42 channel, device -> host, ~1 Hz) */
using OdinTelemetryCallback = void (*)(const OdinTelemetryPacket &packet, void *client_data);

// ============================================================================
// Hotplug Types
// ============================================================================

/** @brief Hotplug callback invoked when a device comes online */
using OnAttachCallback = std::function<void(const DiscoveredDevice &device)>;
/** @brief Hotplug callback invoked when a device goes offline */
using OnDetachCallback = std::function<void(const DiscoveredDevice &device)>;

/** @brief Hotplug callbacks structure - register both callbacks together */
struct HotplugCallbacks {
  OnAttachCallback on_attach;  ///< Called when device comes online
  OnDetachCallback on_detach;  ///< Called when device goes offline
};

}  // namespace sdk
}  // namespace odin
