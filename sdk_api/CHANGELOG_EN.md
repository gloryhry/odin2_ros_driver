# Changelog

This project follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the version numbers follow [Semantic Versioning](https://semver.org/).

V3.3.0_20260822
1. Removed kLidar2Camera, kLidar2Imu, kOdom2Map from the OdomSourceType enum.
2. Removed the GetImageEncoderParam extended API.
3. Updated GetChannelConfig comments.
4. Fixed a crash when the network camera disconnects.
5. Removed the SetImageEncoderParam extended API.
6. Removed the SendUserMessage extended API.
7. Added the sdk_api.md API reference document.

v3.2.0_20260813

[BREAKING CHANGES] Existing integration code must be updated for this release.
B1. Public header layering refactor: odin_lidar_api.h now contains the core API only;
    a new odin_lidar_api_ext.h provides the extended / tuning API. Likewise
    odin_lidar_def.h holds core type definitions and a new odin_lidar_def_ext.h
    holds extended type definitions.
    Users of basic functionality keep including odin_lidar_api.h. Users of advanced
    APIs (filters, image encoder, PTP sync, adaptive params, network config, ...)
    must switch to including odin_lidar_api_ext.h.
B2. Image callback renamed and merged: RegisterImageCallback / RegisterImageCallback2
    -> RegisterCameraCallback / RegisterCameraCallback2. Callback types
    OdinImageCallback / OdinImageCallback2 are merged into OdinCameraCallback, and
    the packet type OdinImagePacket is renamed to OdinCameraPacket.
    A single callback now delivers MJPEG / H.264 / NV12 and any other format; use
    the packet's `format` field to tell the codec apart.
B3. Device operating mode renamed: enum OdinOperatingMode -> OdinDeviceMode, and
    SetOperatingMode -> SetDeviceMode. A matching GetDeviceMode was added.
B4. SetSensorMode is now strongly typed: the mode argument changed from uint8_t to
    the new OdinSensorMode enum (kHDR=0 / kHighPeak=1 / kLowPeak=2 / kAdaptive=3).
    A matching GetSensorMode was added.
B5. IMU / Odom data structures reworked into typed packets: OdinImuPacket and
    OdinOdomPacket now carry frame_count (frame sequence), timestamp (frame time in
    ns) and a std::vector of samples. Protocol-level fields are no longer exposed.
B6. Removed the firmware_version field from the DiscoveredDevice struct. Use the
    GetFirmwareVersion API instead.
B7. Command IDs (cmd_id) were renumbered to align with protocol V2. A matching
    firmware version is required; older firmware is not compatible.
B8. GetNetworkAttribute / SetNetworkAttribute moved from the core API to the
    extended API (odin_lidar_api_ext.h).
B9. Supported image formats changed: yuyv / nv21 / rgb24 removed, h264 added.
B10. The SDK is now built as a shared library instead of a static one. Ship
    libodin_sdk.so* together with your application.

[NEW FEATURES]
1. Added SendUserMessage for application-defined downlink messages (requires
   firmware and algorithm support; non-standard feature).
2. Added SendAlgoData for algorithm data downlink (requires firmware and algorithm
   support; non-standard feature).
3. Added SetFilterParam / GetFilterParam for LiDAR point cloud filters.
4. Added the odomHf channel, separating high- and low-frequency odometry transport.
5. Added cross-subnet hotplug device discovery.
6. Added camera H.264 encoding capability reporting and output.
7. Added deb package generation for the SDK.
8. Added GetDefaultSyncMode to query the device time synchronization mode.
9. Added a mac address field to the DiscoveredDevice struct.
10. Added a warning log suggesting kernel receive-buffer tuning when UDP packet
    loss is detected.
11. Added GetChannelConfig to query the subscription state of a data stream channel.
12. Added SetAdaptiveParam / GetAdaptiveParam to read and write adaptive parameters
    used when sensorMode = adaptive.
13. Added ResetInitialPose to reset the on-module algorithm state.
14. Added RestoreDefaultConfig to restore the module's persisted configuration
    (NIC PTP state, IP, sync mode, ...).
15. Added TF transform data output: new OdinTfPacket struct, OdinTfCallback callback
    type and RegisterTfCallback registration API. Fields mirror ROS 2
    TransformStamped (frame_id / child_frame_id).
16. Added device telemetry output (channel 0x42, about 1 Hz): new OdinTelemetryPacket
    and OdinTelemetryData structs, OdinTelemetryCallback callback type, StartTelemetry
    and RegisterTelemetryCallback APIs. The SDK parses the device JSON internally and
    exposes structured fields (uptime, CPU / memory usage, system load).
17. Added GetPtpSyncStatus to query PTP synchronization status (sync state,
    master-slave offset, path delay, frequency adjustment).
18. Added SetImageEncoderParam / GetImageEncoderParam for image encoder tuning
    (JPEG quality, H.264 bitrate, H.264 QP min / max).
19. Added SetDefaultSyncMode to configure the device time synchronization mode.
20. Hotplug callback types (OnAttachCallback / OnDetachCallback / HotplugCallbacks)
    moved into the public header odin_lidar_def.h; users no longer need to include
    SDK-internal headers.

[BEHAVIOR CHANGES]
21. The default hotplug polling interval changed from 1000 ms to 500 ms
    (SetHotplugPollingInterval).

[BUG FIXES]
22. Fixed missing offline detection for network cameras that are discovered but not
    connected during hotplug events.
23. Fixed the OTA upgrade routine not returning after a successful upgrade.
24. Fixed a crash caused by network camera packet loss.
25. Fixed odom / SLAM transform synchronization: matching changed from keypoint
    matching to nearest-timestamp matching.
26. Fixed incomplete point counts caused by packet loss at the tail of a data frame
    (PadZerosForTailLoss).
27. Fixed intermittent failures of device discovery.

V2.0.3_20260518
1. Renamed the odom struct.
2. Added APIs to get and set the module's network attributes.
3. Added a device reboot API.

V2.0.2_20260513
1. Added the device map -> odom -> base_link TF tree.
2. Optimized IMU data transport.

V2.0.1_20260426
1. Added heartbeat detection (requires device support).
2. Reworked the network camera device mechanism to use heartbeats for offline
   detection.

V2.0.0_20260421
1. Added sensor capability query / configuration APIs.
2. Switched command transport from UDP to TCP.
3. Added image data transport channel configuration (UDP / TCP).
4. Optimized the transport protocol content.
5. Added StartStream / CloseStream flow-control APIs.

V1.0.2_20260326
1. Fixed flicker caused by the SLAM point cloud timestamp callback.

V1.0.1_20260311
1. Added image data validation to filter out incomplete images.
2. Enabled -O3 optimization and improved build speed.

V1.0.0_20260307
1. Fixed OTA failures.
2. Added network device hotplug support.
3. Added multi-instance support for long / short frame modes.
4. Added high-frequency odometry output.
5. Added the OdomSourceType odometry type enum.
6. Added multi-device support to the SLAM / odom synchronization API.
7. Added automatic data port adaptation (auto-discovery of free ports).
8. Added multi-NIC search to the device discovery API.

V0.3.0_20260205
1. Added calibration file retrieval.
2. Added an API to synchronize odometry and SLAM data output.

V0.2.0_20250130
1. Added ODIN2S stereo color image output.
2. Added OTA upgrade support.

V0.1.0_20251205
1. Initial release.
