/**
 * @file odin_lidar_def_ext.h
 * @brief Odin LiDAR SDK Extended Type Definitions
 *
 * Advanced / tuning data types used only by the extended API
 * (odin_lidar_api_ext.h). Most customers only need odin_lidar_def.h.
 *
 * Including this file pulls in the core definitions from odin_lidar_def.h.
 */

#pragma once

#include "odin_lidar_def.h"

namespace odin {
namespace sdk {

// ============================================================================
// Channel Configuration (cmd_id=0x0A)
// ============================================================================

/**
 * @brief Per-channel subscription state returned by GetChannelConfig (cmd_id=0x0A)
 *
 * Mirrors the TLV layout of the protocol ACK (subscribe / dst_port / transport_mode /
 * resolution_id / fps_id / format). Fields not present in the ACK retain their default values.
 */
struct OdinChannelConfigInfo {
  bool subscribed = false;                                  ///< TLV 0x01: 1=subscribed, 0=not
  uint16_t dst_port = 0;                                    ///< TLV 0x02: destination port (0 if N/A)
  OdinTransportMode transport_mode = OdinTransportMode::kUdp; ///< TLV 0x03: 0=UDP, 1=TCP
  uint8_t resolution_id = 0xFF;                             ///< TLV 0x04: capability index (0xFF=unset)
  uint8_t fps_id = 0xFF;                                    ///< TLV 0x05: fps sub-index (0xFF=unset)
  OdinDataFormat format = OdinDataFormat::kUnknown;          ///< TLV 0x06: wire format (kUnknown=unset)
};

// ============================================================================
// LiDAR Filter Parameters (0x32/0x33)
// ============================================================================

/**
 * @brief Filter type identifiers used to select which LiDAR filter to configure.
 */
enum class OdinLidarFilterId : uint8_t {
  kConfidence = 0x01,  ///< Confidence threshold filter
  kAdaptive   = 0x02,  ///< Adaptive analysis filter (indoor/outdoor detection)
  kRange      = 0x03,  ///< Distance range filter
};

/**
 * @brief Parameters for the confidence threshold filter.
 */
struct OdinConfidenceFilterParam {
  uint8_t confidence_threshold = 0;  ///< 0=disabled, 255=keep only highest-confidence points
  OdinConfidenceFilterParam() = default;
  explicit OdinConfidenceFilterParam(uint8_t t) : confidence_threshold(t) {}
};

/**
 * @brief Parameters for the adaptive analysis filter.
 *        This filter is active only when the LiDAR is running in ADAPTIVE mode.
 */
struct OdinAdaptiveFilterParam {
  uint16_t cdf_percentile    = 7000;  ///< CDF percentile threshold (1-10000, unit: 0.01%)
  uint16_t outdoor_threshold = 3000;  ///< CDF distance >= this value is classified as outdoor (mm)
  OdinAdaptiveFilterParam() = default;
  OdinAdaptiveFilterParam(uint16_t cdf, uint16_t thr)
      : cdf_percentile(cdf), outdoor_threshold(thr) {}
};

/**
 * @brief Parameters for the distance range filter.
 */
struct OdinRangeFilterParam {
  uint16_t min_depth = 150;  ///< Points closer than this distance are marked invalid (mm)
  OdinRangeFilterParam() = default;
  explicit OdinRangeFilterParam(uint16_t d) : min_depth(d) {}
};

/**
 * @brief Aggregated LiDAR filter parameter set.
 *        Set has_xxx=true to include that filter in SetFilterParam / GetFilterParam.
 */
struct OdinLidarFilterParams {
  bool has_confidence = false; OdinConfidenceFilterParam confidence;
  bool has_adaptive   = false; OdinAdaptiveFilterParam   adaptive;
  bool has_range      = false; OdinRangeFilterParam      range;
};

// ============================================================================
// Time Synchronization Types (0x20-0x2F)
// ============================================================================

/**
 * @brief Time synchronization mode (cmd_id=0x20 SetDefaultSyncMode)
 */
enum class OdinTimeSyncMode : uint8_t {
  kSysTick = 0x00,  ///< System clock (default)
  kPtp     = 0x01,  ///< PTP hardware timestamp sync
};

/**
 * @brief PTP synchronization status (cmd_id=0x22 GetPtpSyncStatus)
 */
struct OdinPtpSyncStatus {
  uint8_t sync_state    = 0;     ///< 0=searching, 1=locking, 2=locked, 3=error
  uint8_t servo_state   = 0;     ///< ptp4l raw servo state (0/1/2)
  int64_t offset_ns     = 0;     ///< Master-Slave offset (nanoseconds)
  int64_t path_delay_ns = 0;     ///< Path delay (nanoseconds)
  int32_t freq_ppb      = 0;     ///< Frequency adjustment (ppb)
};

}  // namespace sdk
}  // namespace odin
