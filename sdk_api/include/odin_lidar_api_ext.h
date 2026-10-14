/**
 * @file odin_lidar_api_ext.h
 * @brief Odin LiDAR SDK Extended / Advanced Public API
 *
 * Advanced tuning and configuration APIs for power users.
 * Includes filter parameters, image encoder tuning, PTP time sync,
 * adaptive LiDAR parameters, network configuration, pass-through channels, etc.
 *
 * Most customers only need the core API in "odin_lidar_api.h".
 * Including this file pulls in both core and extended APIs.
 *
 * All type definitions are in odin_lidar_def.h.
 */

#pragma once

#include "odin_lidar_api.h"
#include "odin_lidar_def_ext.h"

namespace odin {
namespace sdk {

// ============================================================================
// Channel Query
// ============================================================================

/**
 * @brief Query per-channel subscription state (cmd_id=0x06)
 *
 * Returns the full configuration snapshot for the specified channel,
 * including subscription status, destination port, transport mode,
 * resolution_id and fps_id (as TLV fields).
 *
 * @param device     Device handle
 * @param channel    Channel to query
 * @param info       Output channel config info (TLV fields)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetChannelConfig(OdinDeviceHandle device, OdinDataChannel channel,
                            OdinChannelConfigInfo& info, uint32_t timeout_ms = 1000);

// ============================================================================
// LiDAR Filter Parameters (0x32/0x33)
// ============================================================================

/**
 * @brief Set LiDAR filter parameters.
 *
 * Configures one or more point cloud filters. Only filters with has_xxx=true
 * in @p params are applied; others are left unchanged on the device.
 * Parameters take effect at the next frame boundary without interrupting data flow.
 *
 * @param device     Device handle
 * @param params     Filter parameter set (at least one has_xxx must be true)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult SetFilterParam(OdinDeviceHandle device, const OdinLidarFilterParams& params,
                          uint32_t timeout_ms = 1000);

/**
 * @brief Query current LiDAR filter parameters.
 *
 * Retrieves the current configuration of all supported filters.
 * On success, @p params is populated with the device's current values;
 * has_xxx is set to true for each filter type returned by the device.
 *
 * @param device     Device handle
 * @param params     Output: populated with current filter parameter values
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetFilterParam(OdinDeviceHandle device, OdinLidarFilterParams& params,
                          uint32_t timeout_ms = 1000);

// ============================================================================
// Adaptive LiDAR Parameters (0x34/0x35)
// ============================================================================

/**
 * @brief Set LiDAR adaptive mode parameters (cmd_id=0x34)
 *
 * Configures CDF percentile and outdoor threshold for adaptive LiDAR mode.
 * Only active when LiDAR mode is ADAPTIVE (mode 3).
 *
 * @param device     Device handle
 * @param param      Adaptive parameters
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult SetAdaptiveParam(OdinDeviceHandle device, const OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms = 1000);

/**
 * @brief Query LiDAR adaptive mode parameters (cmd_id=0x35)
 *
 * @param device     Device handle
 * @param param      Output: current adaptive parameters
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetAdaptiveParam(OdinDeviceHandle device, OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms = 1000);

// ============================================================================
// Time Synchronization (0x20-0x2F)
// ============================================================================

/**
 * @brief Set default time synchronization mode (cmd_id=0x20)
 *
 * The setting is persisted to the device's custom YAML configuration.
 * A device reboot is required for the change to take effect.
 *
 * @param device     Device handle
 * @param mode       Sync mode (kSysTick=system clock, kPtp=PTP hardware timestamp)
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult SetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode mode,
                              uint32_t timeout_ms = 1000);

/**
 * @brief Query persisted default sync mode (cmd_id=0x21)
 *
 * Returns the sync_mode from config YAML (default or custom).
 * This reflects the persisted configuration, NOT the real-time PTP status.
 * For real-time PTP sync state, use GetPtpSyncStatus (cmd_id=0x22).
 *
 * @param device     Device handle
 * @param mode       Output: current persisted sync mode
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode& mode,
                              uint32_t timeout_ms = 1000);

/**
 * @brief Query PTP synchronization status (cmd_id=0x22)
 *
 * Returns a real-time snapshot of the PTP sync state. Not persisted.
 *
 * @param device     Device handle
 * @param status     Output: PTP sync status fields
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult GetPtpSyncStatus(OdinDeviceHandle device, OdinPtpSyncStatus& status,
                            uint32_t timeout_ms = 1000);

// ============================================================================
// Algorithm Commands (0x70-0x7F)
// ============================================================================

/**
 * @brief Reset initial pose (cmd_id=0x70)
 *
 * Resets algorithm state (pose estimate, map, factor graph) so SLAM
 * re-initializes from scratch. Does not interrupt data streaming.
 *
 * @param device     Device handle
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult ResetInitialPose(OdinDeviceHandle device, uint32_t timeout_ms = 3000);

// ============================================================================
// Device Configuration
// ============================================================================

/**
 * @brief Restore factory default configuration (cmd_id=0x09)
 *
 * Deletes the custom configuration file on the device. A device reboot is
 * required for the default configuration to take effect.
 *
 * @param device     Device handle
 * @param timeout_ms Timeout in milliseconds (default: 1000)
 * @return Operation result
 */
OdinResult RestoreDefaultConfig(OdinDeviceHandle device, uint32_t timeout_ms = 1000);

/**
 * @brief Get device network attribute configuration
 *
 * Retrieves current network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
 *
 * @param device Device handle
 * @param attr Output: network attribute structure
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);

/**
 * @brief Set device network attribute configuration
 *
 * Configures network settings (IP, gateway, netmask, DHCP, PTP) via HTTP API.
 * Changes take effect after device reboot.
 *
 * @param device Device handle
 * @param attr Network attribute to set
 * @param timeout_ms Timeout in milliseconds (default: 3000)
 * @return Operation result
 */
OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);

// ============================================================================
// Pass-through / Advanced Channels
// ============================================================================

/**
 * @brief Send algorithm downlink passthrough data to the device (0x40).
 *
 * One-way host->device opaque algorithm bytes. The SDK acts as a TCP server:
 * the downlink listening port and its 0x40 subscription are brought up
 * automatically when the device connects, and the module connects in
 * asynchronously. There is no explicit start/stop; the link only needs to be
 * up at send time.
 *
 * The SDK does NOT interpret `data`; it fragments the buffer into protocol v2
 * frames (data_type=0x40) per docs/protocol/PROTOCOL.md section 2.4 and writes
 * them on the module connection. Semantics of the payload are an application
 * contract between the host algorithm and the device-side algorithm process.
 *
 * Fail-fast: if the module has not connected to the downlink server yet, this
 * returns kNotInitialized immediately (a one-time warning is logged); the SDK
 * does NOT buffer or retry.
 *
 * @param device  Device handle.
 * @param data    Opaque algorithm bytes (must be non-null).
 * @param len     Byte count of `data` (1 .. 256 KB).
 * @return kOk on success; kNotInitialized if the module is not connected;
 *         kInvalidArgument on null/size violations; kSocketError on write
 *         failure.
 */
OdinResult SendAlgoData(OdinDeviceHandle device, const uint8_t* data, size_t len);

}  // namespace sdk
}  // namespace odin
