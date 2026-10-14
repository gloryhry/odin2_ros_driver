# Odin SDK API 文档 / Odin SDK API Reference

> [← 返回 README](../README.md)

本文档详细描述 Odin LiDAR SDK 的公开 C++ API，包括设备发现、连接管理、数据回调注册、指令发送、流控制、文件传输、热插拔监听及高级调优接口。

This document describes the public C++ API of the Odin LiDAR SDK, covering device discovery, connection management, data callback registration, command sending, stream control, file transfer, hotplug listener, and advanced tuning APIs.

---

## 目录 / Table of Contents

- [头文件概览 / Header Overview](#头文件概览--header-overview)
- [命名空间 / Namespace](#命名空间--namespace)
- [基本类型 / Basic Types](#基本类型--basic-types)
- [枚举定义 / Enumerations](#枚举定义--enumerations)
- [数据结构 / Data Structures](#数据结构--data-structures)
- [回调类型 / Callback Types](#回调类型--callback-types)
- [核心 API / Core API](#核心-api--core-api)
  - [设备发现 / Device Discovery](#设备发现--device-discovery)
  - [连接管理 / Connection Management](#连接管理--connection-management)
  - [数据回调注册 / Data Callback Registration](#数据回调注册--data-callback-registration)
  - [流控制 / Stream Control](#流控制--stream-control)
  - [基本指令 / Basic Commands](#基本指令--basic-commands)
  - [文件传输 / File Transfer](#文件传输--file-transfer)
  - [热插拔监听 / Hotplug Listener](#热插拔监听--hotplug-listener)
- [扩展 API / Extended API](#扩展-api--extended-api)
  - [通道查询 / Channel Query](#通道查询--channel-query)
  - [点云滤波参数 / LiDAR Filter Parameters](#点云滤波参数--lidar-filter-parameters)
  - [自适应参数 / Adaptive Parameters](#自适应参数--adaptive-parameters)
  - [时间同步 / Time Synchronization](#时间同步--time-synchronization)
  - [算法指令 / Algorithm Commands](#算法指令--algorithm-commands)
  - [设备配置 / Device Configuration](#设备配置--device-configuration)
  - [透传通道 / Pass-through Channel](#透传通道--pass-through-channel)
- [典型使用流程 / Typical Usage Flow](#典型使用流程--typical-usage-flow)
- [错误码 / Result Codes](#错误码--result-codes)

---

## 头文件概览 / Header Overview

| 头文件 / Header | 说明 / Description |
|---|---|
| `odin_lidar_api.h` | 核心 API：设备发现、连接、数据回调、流控制、基本指令 / Core API: discovery, connection, callbacks, stream control, basic commands |
| `odin_lidar_def.h` | 核心类型定义：枚举、数据结构、回调类型 / Core type definitions: enums, structs, callback types |
| `odin_lidar_api_ext.h` | 扩展 API：滤波参数、PTP 同步、网络配置、透传等 / Extended API: filter params, PTP sync, network config, pass-through, etc. |
| `odin_lidar_def_ext.h` | 扩展类型定义：滤波参数结构、PTP 状态等 / Extended type definitions: filter param structs, PTP status, etc. |

> 包含 `odin_lidar_api_ext.h` 会自动引入核心 API 和扩展 API。大多数用户只需包含 `odin_lidar_api.h`。
>
> Including `odin_lidar_api_ext.h` pulls in both core and extended APIs. Most users only need `odin_lidar_api.h`.

---

## 命名空间 / Namespace

所有 API 和类型均位于 `odin::sdk` 命名空间下。

All APIs and types reside under the `odin::sdk` namespace.

```cpp
using namespace odin::sdk;
```

---

## 基本类型 / Basic Types

| 类型 / Type | 定义 / Definition | 说明 / Description |
|---|---|---|
| `OdinDeviceHandle` | `uint32_t` | 设备句柄，由 `ConnectDevice()` 返回 / Device handle returned by `ConnectDevice()` |
| `kInvalidDeviceHandle` | `0` | 无效句柄常量 / Invalid handle sentinel |

---

## 枚举定义 / Enumerations

### OdinResult

API 调用结果码 / API result codes.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kOk` | 0 | 操作成功 / Operation successful |
| `kNotInitialized` | 1 | SDK 未初始化或设备未连接 / SDK not initialized or device not connected |
| `kInvalidArgument` | 2 | 无效参数 / Invalid argument |
| `kSocketError` | 3 | 网络套接字错误 / Network socket error |
| `kTimeout` | 4 | 操作超时 / Operation timed out |
| `kStopped` | 5 | 操作已停止 / Operation stopped |
| `kFirmwareIllegal` | 6 | 无效固件文件 / Invalid firmware file |
| `kCommunicationError` | 7 | 通信错误 / Communication error |
| `kUnknownError` | 8 | 未知错误 / Unknown error |

### OdinDeviceMode

设备工作模式 / Device operating mode.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kStandby` | 0 | 待机模式，停止数据流 / Standby mode, stops data streaming |
| `kNormal` | 1 | 正常工作模式，开始数据流 / Normal operating mode, starts data streaming |
| `kUpgrade` | 2 | 升级模式 / Upgrade mode |
| `kUnknown` | 0xFF | 未知模式 / Unknown mode |

### OdinSensorMode

LiDAR 传感器模式 / LiDAR sensor mode (cmd_id=0x05/0x09).

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kHDR` | 0 | HDR 模式 / HDR mode |
| `kHighPeak` | 1 | 高反模式 / High Peak mode |
| `kLowPeak` | 2 | 低反模式 / Low Peak mode |
| `kAdaptive` | 3 | 自适应模式 / Adaptive mode |

### OdinFileType

设备文件类型 / Device file type.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kRelocationMap` | 0 | 重定位地图文件 / Relocation map file |
| `kCalibrationFile` | 1 | 相机标定文件 / Camera calibration file |
| `kDevLogFile` | 2 | 设备日志文件 / Device log file |
| `kFirmware` | 3 | 固件 / OTA 文件 / Firmware / OTA file |
| `kUnknown` | 0xFF | 未知类型 / Unknown file type |

### OdinDataChannel

数据通道类型 / Data channel type. 每段保留 16 个槽位 / Each segment reserves 16 slots (0x10 per group).

| 枚举值 / Enum Value | 值 / Value | 方向 / Direction | 说明 / Description |
|---|---|---|---|
| `kRawPoint` | 0x00 | device→host | 原始点云 / Raw point cloud |
| `kSlamPoint` | 0x01 | device→host | SLAM 点云 / SLAM point cloud |
| `kImage0` | 0x10 | device→host | 左相机图像 / Left camera image/video |
| `kImage1` | 0x11 | device→host | 右相机图像 / Right camera image/video |
| `kImu` | 0x20 | device→host | IMU 6 轴数据 / IMU 6-axis data |
| `kOdomLF` | 0x30 | device→host | 低频里程计 / Low-frequency odometry |
| `kOdomHF` | 0x31 | device→host | 高频里程计 / High-frequency odometry |
| `kTf` | 0x32 | device→host | TF 变换 / TF transforms |
| `kAlgoDownlink` | 0x40 | host→device | 算法下行透传 / Algorithm downlink pass-through |
| `kCalibFile` | 0x41 | device→host | 标定文件下载 / Calibration file download |
| `kTelemetry` | 0x42 | device→host | 遥测 JSON (~1 Hz) / Telemetry JSON |

### OdinTransportMode

传输模式 / Transport mode.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kUdp` | 0 | UDP 传输（默认）/ UDP transport (default) |
| `kTcp` | 1 | TCP 传输 / TCP transport |

### OdinDataFormat

统一数据格式 / Unified data format. 图像格式使用位掩码编码 / Image formats use bitmask encoding.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kImageNv12` | 0x01 | NV12 原始数据 / NV12 raw passthrough |
| `kImageMjpeg` | 0x02 | MJPEG 压缩 / MJPEG compressed |
| `kImageH264` | 0x04 | H.264 编码 / H.264 encoded |
| `kImageH265` | 0x08 | H.265 编码 / H.265 encoded |
| `kRawPointXyzic` | 0x10 | XYZ + 强度 + 置信度 / XYZ + Intensity + Confidence |
| `kSlamPointXyzrgba` | 0x20 | XYZ + RGBA / XYZ + RGBA |
| `kImu6Axis` | 0x30 | 陀螺仪 + 加速度计 (6 轴) / Gyro + Acc (6-axis) |
| `kOdomStandard` | 0x40 | 标准里程计 / Standard odometry |
| `kTfStandard` | 0x50 | 标准 TF 变换 / Standard TF transform |
| `kUnknown` | 0xFF | 未知格式 / Unknown format |

### OdomSourceType

里程计来源类型 / Odometry source type.

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kOdom2ImuLow` | 0 | Odom → IMU 低频 / Odom → IMU low-freq |
| `kOdom2ImuHigh` | 1 | Odom → IMU 高频 / Odom → IMU high-freq |

### OdinTimeSyncMode

时间同步模式 / Time synchronization mode (扩展 API / Extended API).

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kSysTick` | 0x00 | 系统时钟（默认）/ System clock (default) |
| `kPtp` | 0x01 | PTP 硬件时间戳同步 / PTP hardware timestamp sync |

### OdinLidarFilterId

点云滤波器类型 / LiDAR filter type identifiers (扩展 API / Extended API).

| 枚举值 / Enum Value | 值 / Value | 说明 / Description |
|---|---|---|
| `kConfidence` | 0x01 | 置信度阈值滤波 / Confidence threshold filter |
| `kAdaptive` | 0x02 | 自适应分析滤波 / Adaptive analysis filter |
| `kRange` | 0x03 | 距离范围滤波 / Distance range filter |

---

## 数据结构 / Data Structures

### OdinStreamCfg

流配置 — 分辨率、帧率和格式的有效组合 / Stream configuration — a valid combination of resolution, fps, and format.

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `width` | `uint16_t` | 0 | 分辨率宽度（0 = 不发送，使用设备默认值）/ Resolution width (0 = not sent, use device default) |
| `height` | `uint16_t` | 0 | 分辨率高度（0 = 不发送）/ Resolution height (0 = not sent) |
| `fps` | `uint16_t` | 0 | 帧率（0 = 不发送）/ Frame rate (0 = not sent) |
| `format` | `OdinDataFormat` | `kUnknown` | 图像格式（`kUnknown` = 不发送）/ Image format (`kUnknown` = not sent) |

### OdinSensorCapability

传感器能力 — 某数据通道支持的模式列表 / Sensor capability — supported modes for a data channel.

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `channel` | `OdinDataChannel` | 通道类型 / Channel type |
| `modes` | `std::vector<OdinStreamCfg>` | 支持的模式列表 / Supported mode list |

### OdinRawPoint\<T\>

原始点云单点结构（紧凑格式）/ Raw point cloud single point (packed).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `x` | `T` | X 坐标 / X coordinate |
| `y` | `T` | Y 坐标 / Y coordinate |
| `z` | `T` | Z 坐标 / Z coordinate |
| `intensity` | `uint8_t` | 反射强度 / Reflectance intensity |
| `confidence` | `uint8_t` | 置信度 / Confidence |

### OdinSlamPoint\<T\>

SLAM 点云单点结构（紧凑格式）/ SLAM point cloud single point (packed).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `x` | `T` | X 坐标 / X coordinate |
| `y` | `T` | Y 坐标 / Y coordinate |
| `z` | `T` | Z 坐标 / Z coordinate |
| `r` | `uint8_t` | 红色通道 / Red channel |
| `g` | `uint8_t` | 绿色通道 / Green channel |
| `b` | `uint8_t` | 蓝色通道 / Blue channel |
| `a` | `uint8_t` | 透明度通道 / Alpha channel |

### OdinImuSample

单条 IMU 采样（v2 协议，24 字节）/ Single IMU sample (v2 protocol, 24 bytes).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `gyro_x` / `gyro_y` / `gyro_z` | `float` | 角速度 X/Y/Z (rad/s) / Angular velocity (rad/s) |
| `acc_x` / `acc_y` / `acc_z` | `float` | 线性加速度 X/Y/Z (m/s²) / Linear acceleration (m/s²) |

### OdinOdomData

里程计数据条目 / Odometry data entry (125 bytes, packed).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `pos[3]` | `float[3]` | 位置 (x, y, z) / Position |
| `orient[4]` | `float[4]` | 姿态四元数 (x, y, z, w) / Orientation quaternion |
| `linear_velocity[3]` | `float[3]` | 线速度 / Linear velocity |
| `angular_velocity[3]` | `float[3]` | 角速度 / Angular velocity |
| `cov[18]` | `float[18]` | 协方差矩阵 / Covariance matrix |
| `type` | `uint8_t` | 里程计来源类型（见 `OdomSourceType`）/ Odom source type (see `OdomSourceType`) |

### OdinPointCloudPacket

原始点云帧（回调交付）/ Assembled raw point-cloud frame (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `frame_count` | `uint32_t` | 帧序号（用于丢帧检测）/ Frame sequence number (for drop detection) |
| `timestamp` | `uint64_t` | 帧时间戳 (ns，硬件单调时钟) / Frame timestamp in ns (hardware monotonic) |
| `points` | `std::vector<OdinRawPoint<float>>` | 浮点点云数据（已转换，可直接使用）/ Float point data (ready to use) |

### OdinSlamPacket

SLAM 点云帧（回调交付）/ Assembled SLAM point-cloud frame (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `frame_count` | `uint32_t` | 帧序号 / Frame sequence number |
| `timestamp` | `uint64_t` | 帧时间戳 (ns) / Frame timestamp in ns |
| `points` | `std::vector<OdinSlamPoint<float>>` | 浮点 SLAM 点云数据（已转换）/ Float SLAM point data (ready to use) |

### OdinCameraPacket

相机帧（回调交付，支持 MJPEG / H.264 / H.265）/ Unified camera frame (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `format` | `OdinDataFormat` | 编解码格式标签 / Codec/format tag |
| `timestamp` | `uint64_t` | 帧时间戳 (ns) / Frame timestamp in ns |
| `frame_count` | `uint32_t` | 帧序号（来自设备）/ Frame sequence number from device |
| `width` | `uint16_t` | 帧宽度 / Frame width |
| `height` | `uint16_t` | 帧高度 / Frame height |
| `is_keyframe` | `bool` | 是否关键帧（H.264: IDR/SPS 检测; MJPEG: 始终 true）/ Is keyframe (H.264: IDR/SPS; MJPEG: always true) |
| `payload` | `std::vector<uint8_t>` | 编码字节流（JPEG SOI..EOI 或 H.264 Annex-B AU）/ Encoded bytes |

### OdinImuPacket

IMU 数据包（回调交付）/ IMU data packet (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `timestamp` | `uint64_t` | 重建的每样本时间戳 (ns) / Reconstructed per-sample timestamp in ns |
| `samples` | `std::vector<OdinImuSample>` | IMU 采样数据（可直接使用）/ IMU samples (ready to use) |

### OdinOdomPacket

里程计数据包（回调交付）/ Odometry data packet (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `frame_count` | `uint32_t` | 帧序号 / Frame sequence number |
| `timestamp` | `uint64_t` | 帧时间戳 (ns) / Frame timestamp in ns |
| `data` | `std::vector<OdinOdomData>` | 里程计数据条目 / Odom data entries |

### OdinTelemetryData

遥测数据（0x42 通道，~1 Hz）/ Parsed telemetry data (0x42 channel, ~1 Hz).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `uptime_s` | `uint32_t` | 设备运行时间（秒）/ Device uptime in seconds |
| `cpu_pct` | `uint32_t` | CPU 使用率 (0-100) / CPU usage percentage |
| `mem_used_kb` | `uint32_t` | 已用内存 (KB) / Memory used in kilobytes |
| `mem_total_kb` | `uint32_t` | 总内存 (KB) / Total memory in kilobytes |
| `mem_use_rate` | `uint32_t` | 内存使用率 (0-100) / Memory usage rate percentage |
| `load_1` | `float` | 系统负载 (1 分钟) / System load average (1 min) |
| `load_5` | `float` | 系统负载 (5 分钟) / System load average (5 min) |
| `load_15` | `float` | 系统负载 (15 分钟) / System load average (15 min) |

### OdinTelemetryPacket

遥测数据包（回调交付）/ Telemetry packet (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `frame_count` | `uint32_t` | 帧序号 / Frame sequence number |
| `timestamp` | `uint64_t` | 帧时间戳 (ns) / Frame timestamp in ns |
| `data` | `OdinTelemetryData` | 解析后的遥测字段 / Parsed telemetry fields |

### OdinTfPacket

TF 变换帧（回调交付）/ Assembled TF frame (delivered via callback).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `frame_count` | `uint32_t` | 帧序号 / Frame sequence number |
| `timestamp` | `uint64_t` | 帧时间戳 (ns) / Frame timestamp in ns |
| `translation[3]` | `float[3]` | 平移 (m) / Translation in meters |
| `rotation[4]` | `float[4]` | 旋转四元数 (x, y, z, w) / Rotation quaternion |
| `frame_id` | `std::string` | 父坐标系名称 / Parent frame ID |
| `child_frame_id` | `std::string` | 子坐标系名称 / Child frame ID |

### NetworkAttribute

网络属性配置 / Network attribute configuration.

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `ip_address` | `std::string` | - | 设备 IP 地址 / Device IP address |
| `gateway` | `std::string` | - | 网关地址 / Gateway address |
| `netmask` | `std::string` | - | 子网掩码 / Subnet mask |
| `mac` | `std::string` | - | 设备 MAC 地址 / Device MAC address |
| `dhcp` | `bool` | `false` | 是否启用 DHCP / DHCP enabled |
| `ptp` | `bool` | `false` | 是否启用 PTP / PTP enabled |

### DiscoveredDevice

发现的设备信息 / Discovered device information.

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `sn` | `std::string` | 设备序列号 / Device serial number |
| `model` | `std::string` | 设备型号（如 "Odin-L1"）/ Device model name (e.g., "Odin-L1") |
| `host_ip` | `std::string` | 发现该设备的主机接口 IP / Host interface IP that discovered this device |
| `network` | `NetworkAttribute` | 网络配置 / Network configuration |
| `connection` | `MTConnectionType` | 连接类型（默认 Ethernet）/ Connection type (default: Ethernet) |

### OdinChannelConfigInfo

通道订阅状态（扩展 API，`GetChannelConfig` 返回）/ Per-channel subscription state (Extended API, returned by `GetChannelConfig`).

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `subscribed` | `bool` | `false` | 是否已订阅 / Subscribed (TLV 0x01) |
| `dst_port` | `uint16_t` | 0 | 目标端口 / Destination port (TLV 0x02) |
| `transport_mode` | `OdinTransportMode` | `kUdp` | 传输模式 / Transport mode (TLV 0x03) |
| `resolution_id` | `uint8_t` | 0xFF | 分辨率能力索引 / Capability index (TLV 0x04, 0xFF=unset) |
| `fps_id` | `uint8_t` | 0xFF | 帧率子索引 / FPS sub-index (TLV 0x05, 0xFF=unset) |
| `format` | `OdinDataFormat` | `kUnknown` | 线缆格式 / Wire format (TLV 0x06, kUnknown=unset) |

### OdinConfidenceFilterParam

置信度滤波参数 / Confidence threshold filter parameters.

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `confidence_threshold` | `uint8_t` | 0 | 0=禁用，255=仅保留最高置信度点 / 0=disabled, 255=keep only highest-confidence |

### OdinAdaptiveFilterParam

自适应滤波参数 / Adaptive analysis filter parameters. 仅在 LiDAR 模式为 ADAPTIVE 时生效 / Active only when LiDAR mode is ADAPTIVE.

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `cdf_percentile` | `uint16_t` | 7000 | CDF 百分位阈值 (1-10000, 单位: 0.01%) / CDF percentile threshold |
| `outdoor_threshold` | `uint16_t` | 3000 | CDF 距离 ≥ 此值判定为室外 (mm) / CDF distance classified as outdoor (mm) |

### OdinRangeFilterParam

距离范围滤波参数 / Distance range filter parameters.

| 字段 / Field | 类型 / Type | 默认值 / Default | 说明 / Description |
|---|---|---|---|
| `min_depth` | `uint16_t` | 150 | 小于此距离的点标记为无效 (mm) / Points closer than this are invalid (mm) |

### OdinLidarFilterParams

聚合滤波参数集 / Aggregated LiDAR filter parameter set. 设置 `has_xxx=true` 以在 `SetFilterParam` / `GetFilterParam` 中包含对应滤波器 / Set `has_xxx=true` to include that filter.

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `has_confidence` | `bool` | 是否包含置信度滤波 / Include confidence filter |
| `confidence` | `OdinConfidenceFilterParam` | 置信度滤波参数 / Confidence filter params |
| `has_adaptive` | `bool` | 是否包含自适应滤波 / Include adaptive filter |
| `adaptive` | `OdinAdaptiveFilterParam` | 自适应滤波参数 / Adaptive filter params |
| `has_range` | `bool` | 是否包含距离滤波 / Include range filter |
| `range` | `OdinRangeFilterParam` | 距离滤波参数 / Range filter params |

### OdinPtpSyncStatus

PTP 同步状态（扩展 API）/ PTP synchronization status (Extended API).

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `sync_state` | `uint8_t` | 0=搜索, 1=锁定中, 2=已锁定, 3=错误 / 0=searching, 1=locking, 2=locked, 3=error |
| `servo_state` | `uint8_t` | ptp4l 原始伺服状态 (0/1/2) / ptp4l raw servo state |
| `offset_ns` | `int64_t` | 主从偏移 (ns) / Master-Slave offset (nanoseconds) |
| `path_delay_ns` | `int64_t` | 路径延迟 (ns) / Path delay (nanoseconds) |
| `freq_ppb` | `int32_t` | 频率调整 (ppb) / Frequency adjustment (ppb) |

### HotplugCallbacks

热插拔回调结构 / Hotplug callbacks structure.

| 字段 / Field | 类型 / Type | 说明 / Description |
|---|---|---|
| `on_attach` | `OnAttachCallback` | 设备上线时调用 / Called when device comes online |
| `on_detach` | `OnDetachCallback` | 设备下线时调用 / Called when device goes offline |

---

## 回调类型 / Callback Types

| 类型别名 / Type Alias | 签名 / Signature | 说明 / Description |
|---|---|---|
| `OdinPointCloudCallback` | `void(*)(const OdinPointCloudPacket&, void*)` | 原始点云回调 / Raw point cloud callback |
| `OdinSlamCallback` | `void(*)(const OdinSlamPacket&, void*)` | SLAM 点云回调 / SLAM point cloud callback |
| `OdinCameraCallback` | `void(*)(const OdinCameraPacket&, void*)` | 相机帧回调 / Camera frame callback |
| `OdinImuCallback` | `void(*)(const OdinImuPacket&, void*)` | IMU 数据回调 / IMU data callback |
| `OdinOdomCallback` | `void(*)(const OdinOdomPacket&, OdomSourceType, void*)` | 里程计回调（含来源类型）/ Odometry callback (with source type) |
| `OdinTfCallback` | `void(*)(const OdinTfPacket&, void*)` | TF 变换回调 / TF transform callback |
| `OdinTelemetryCallback` | `void(*)(const OdinTelemetryPacket&, void*)` | 遥测数据回调 / Telemetry data callback |
| `OdinUpgradeProgressCallback` | `void(*)(OdinDeviceHandle, float, void*)` | 升级进度回调 (0-100%) / Upgrade progress callback |
| `OnAttachCallback` | `std::function<void(const DiscoveredDevice&)>` | 热插拔上线回调 / Hotplug attach callback |
| `OnDetachCallback` | `std::function<void(const DiscoveredDevice&)>` | 热插拔下线回调 / Hotplug detach callback |

> 所有数据回调中 `void* client_data` 为用户自定义数据指针，在注册回调时传入。
>
> The `void* client_data` in all data callbacks is a user-defined pointer passed during callback registration.

---

## 核心 API / Core API

> 头文件 / Header: `odin_lidar_api.h`

### 设备发现 / Device Discovery

#### `DiscoverDevices`

```cpp
bool DiscoverDevices(std::vector<DiscoveredDevice> &devices, uint32_t timeout_ms = 2000);
```

广播发现局域网内的 Odin 设备。自动枚举所有网络接口，在每个子网上进行发现。结果按 SN 去重。

Broadcasts to discover Odin devices on the LAN. Automatically enumerates all network interfaces and discovers devices on each subnet. Results are deduplicated by SN.

- **参数 / Parameters**:
  - `devices`: 输出，发现的设备列表 / Output: list of discovered devices
  - `timeout_ms`: 发现超时（毫秒，在所有接口间分配）/ Discovery timeout in ms (divided among interfaces)
- **返回值 / Returns**: `true` 如果找到至少一台设备 / `true` if at least one device was found

#### `GetSdkVersion`

```cpp
std::string GetSdkVersion();
```

获取 SDK 版本字符串 / Get SDK version string.

- **返回值 / Returns**: 版本字符串，如 `"0.3.0_20260205"` / Version string, e.g., `"0.3.0_20260205"`

---

### 连接管理 / Connection Management

#### `ConnectDevice`

```cpp
OdinDeviceHandle ConnectDevice(const DiscoveredDevice &device);
```

连接到已发现的设备。所有端口自动分配：设备命令端口固定 60001，其余端口随机分配。

Connects to a discovered device. All ports are auto-generated: device command port is fixed at 60001, other ports are randomly assigned.

- **参数 / Parameters**: `device` — 来自 `DiscoverDevices()` 的设备信息 / Device from `DiscoverDevices()`
- **返回值 / Returns**: 设备句柄，失败返回 `kInvalidDeviceHandle` / Device handle, or `kInvalidDeviceHandle` on failure

#### `DisconnectDevice`

```cpp
void DisconnectDevice(OdinDeviceHandle handle);
```

断开设备连接 / Disconnect from a device.

- **参数 / Parameters**: `handle` — 设备句柄 / Device handle

---

### 数据回调注册 / Data Callback Registration

> 所有回调注册函数要求设备已连接。`client_data` 指针会原样传递到回调函数。
>
> All callback registration functions require a connected device. The `client_data` pointer is passed through to the callback.

#### `RegisterPointCloudCallback`

```cpp
void RegisterPointCloudCallback(OdinDeviceHandle handle, OdinPointCloudCallback cb, void *client_data);
```

注册原始点云回调（data_type=0）/ Register raw point cloud callback (data_type=0).

#### `RegisterSlamCallback`

```cpp
void RegisterSlamCallback(OdinDeviceHandle handle, OdinSlamCallback cb, void *client_data);
```

注册 SLAM 点云回调（data_type=1）/ Register SLAM point cloud callback (data_type=1).

#### `RegisterCameraCallback`

```cpp
void RegisterCameraCallback(OdinDeviceHandle handle, OdinCameraCallback cb, void *client_data);
```

注册左相机帧回调（kImage0）。支持 MJPEG / H.264 / H.265 等所有格式，通过 `packet.format` 区分。

Register left camera frame callback (kImage0). Supports all formats (MJPEG / H.264 / H.265), distinguished by `packet.format`.

#### `RegisterCameraCallback2`

```cpp
void RegisterCameraCallback2(OdinDeviceHandle handle, OdinCameraCallback cb, void *client_data);
```

注册右相机帧回调（kImage1）/ Register right camera frame callback (kImage1).

#### `RegisterImuCallback`

```cpp
void RegisterImuCallback(OdinDeviceHandle handle, OdinImuCallback cb, void *client_data);
```

注册 IMU 数据回调（data_type=3）/ Register IMU data callback (data_type=3).

#### `RegisterOdomCallback`

```cpp
void RegisterOdomCallback(OdinDeviceHandle handle, OdinOdomCallback cb, void *client_data);
```

注册里程计数据回调（data_type=4）。回调签名包含 `OdomSourceType` 参数以区分低频/高频里程计。

Register odometry data callback (data_type=4). The callback signature includes `OdomSourceType` to distinguish low-freq / high-freq odom.

#### `RegisterTelemetryCallback`

```cpp
void RegisterTelemetryCallback(OdinDeviceHandle handle, OdinTelemetryCallback cb, void *client_data);
```

注册遥测数据回调（data_type=0x42）。设备以 ~1 Hz 推送 JSON 遥测帧，SDK 内部解析后通过回调交付结构化数据。

Register telemetry data callback (data_type=0x42). The device pushes JSON telemetry frames at ~1 Hz; the SDK parses JSON internally and delivers structured data via the callback.

#### `RegisterTfCallback`

```cpp
void RegisterTfCallback(OdinDeviceHandle handle, OdinTfCallback cb, void *client_data);
```

注册 TF 变换回调（data_type=0x32）。每帧自包含，包含平移、旋转和坐标系 ID。

Register TF transform callback (data_type=0x32). Each frame is self-contained with translation, rotation, and frame IDs.

#### `StartTelemetry`

```cpp
OdinResult StartTelemetry(OdinDeviceHandle handle);
```

启动遥测订阅（data_type=0x42）。建立本地 TCP 服务器并订阅 0x42 通道，设备开始以 ~1 Hz 推送遥测帧。可在调用前后注册 `RegisterTelemetryCallback` 来接收数据。

Start telemetry subscription (data_type=0x42). Brings up a local TCP server and subscribes the 0x42 channel so the device begins pushing telemetry frames at ~1 Hz. Call `RegisterTelemetryCallback` before or after to receive data.

- **返回值 / Returns**: `kOk` 成功；`kNotInitialized` 设备未连接 / `kOk` on success; `kNotInitialized` if device not connected

#### `EnableSlamOdomSyncForDevice`

```cpp
void EnableSlamOdomSyncForDevice(OdinDeviceHandle handle, bool enabled, uint32_t max_frame_lag = 10);
```

启用/禁用 SLAM 和 Odom 数据同步。启用时（默认），SDK 按 `frame_count` 匹配 SLAM 和 Odom 数据，将 SLAM 点从 IMU 坐标系变换到世界坐标系。超过 `max_frame_lag` 的未匹配数据将被丢弃。

Enable/disable SLAM and Odom data synchronization. When enabled (default), SDK matches SLAM and Odom data by `frame_count` before invoking callbacks. SLAM points are transformed from IMU frame to world frame using Odom pose. Unmatched data exceeding `max_frame_lag` is discarded.

- **参数 / Parameters**:
  - `handle`: 设备句柄 / Device handle
  - `enabled`: 是否启用同步（默认 true）/ Whether to enable sync (default: true)
  - `max_frame_lag`: 最大帧号差，超过则丢弃未匹配数据（默认 10）/ Max frame count difference before discarding (default: 10)

---

### 流控制 / Stream Control

#### `StartStream`

```cpp
OdinResult StartStream(OdinDeviceHandle device, OdinDataChannel channel,
                       OdinTransportMode transport, const OdinStreamCfg* mode = nullptr,
                       uint32_t timeout_ms = 1000);
```

启动数据流（cmd_id=0x05）。端口由 SDK 内部分配。阻塞调用，等待设备响应。

Start a data stream (cmd_id=0x05). Port is auto-allocated by the SDK. Blocking call that waits for device response.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `channel`: 数据通道类型（如 `kRawPoint`, `kImage0`, `kImu` 等）/ Data channel type
  - `transport`: 传输模式（`kUdp` 或 `kTcp`）/ Transport mode (`kUdp` or `kTcp`)
  - `mode`: 可选流配置（分辨率/帧率/格式），`nullptr` 使用设备默认 / Optional stream config, `nullptr` for device defaults
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `CloseStream`

```cpp
OdinResult CloseStream(OdinDeviceHandle device, OdinDataChannel channel, uint32_t timeout_ms = 1000);
```

关闭数据流（cmd_id=0x05）。通过发送 `dst_port=0` 停止数据流。阻塞调用。

Close a data stream (cmd_id=0x05). Stops the stream by sending `dst_port=0`. Blocking call.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `channel`: 要关闭的数据通道 / Data channel to close
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetSensorCapability`

```cpp
OdinResult GetSensorCapability(OdinDeviceHandle device,
                               std::vector<OdinSensorCapability>& capabilities,
                               const std::vector<OdinDataChannel>& channels = {},
                               uint32_t timeout_ms = 1000);
```

查询设备传感器能力（cmd_id=0x08）。返回指定通道支持的分辨率/帧率/格式组合列表。

Query device sensor capabilities (cmd_id=0x08). Returns supported resolution/fps/format combinations for specified channels.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `capabilities`: 输出，传感器能力列表 / Output: list of sensor capabilities
  - `channels`: 要查询的通道列表（空 = 查询全部）/ Channels to query (empty = all)
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `Reboot`

```cpp
OdinResult Reboot(OdinDeviceHandle device, uint32_t timeout_ms = 3000);
```

重启设备。通过网络 API 发送重启命令，设备将重新启动。网络设置更改在重启后生效。

Reboot the device. Sends reboot command via HTTP API. Network settings changes take effect after reboot.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `timeout_ms`: 超时（毫秒，默认 3000）/ Timeout in ms (default: 3000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 基本指令 / Basic Commands

#### `SetDeviceMode`

```cpp
OdinResult SetDeviceMode(OdinDeviceHandle device, OdinDeviceMode mode, uint32_t timeout_ms = 1000);
```

设置设备工作模式（cmd_id=0x02）。`kStandby` 停止数据流，`kNormal` 开始数据流。

Set device operating mode (cmd_id=0x02). `kStandby` stops streaming, `kNormal` starts streaming.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 目标模式（`kStandby`, `kNormal`, `kUpgrade`）/ Target mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetDeviceMode`

```cpp
OdinResult GetDeviceMode(OdinDeviceHandle device, OdinDeviceMode& mode, uint32_t timeout_ms = 1000);
```

查询设备当前工作模式（cmd_id=0x08）。

Query current device operating mode (cmd_id=0x08).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 输出，当前设备模式 / Output: current device mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `SetSensorMode`

```cpp
OdinResult SetSensorMode(OdinDeviceHandle device, OdinSensorMode mode, uint32_t timeout_ms = 1000);
```

设置 LiDAR 传感器模式（cmd_id=0x05）。

Set LiDAR sensor mode (cmd_id=0x05).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 传感器模式（0=HDR, 1=高反, 2=低反, 3=自适应）/ Sensor mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetSensorMode`

```cpp
OdinResult GetSensorMode(OdinDeviceHandle device, OdinSensorMode& mode, uint32_t timeout_ms = 1000);
```

查询 LiDAR 传感器模式（cmd_id=0x09）。

Query LiDAR sensor mode (cmd_id=0x09).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 输出，当前 LiDAR 模式 / Output: current LiDAR mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetFirmwareVersion`

```cpp
OdinResult GetFirmwareVersion(OdinDeviceHandle device, std::string &version, uint32_t timeout_ms = 1000);
```

查询设备固件版本（cmd_id=0x06）。

Get device firmware version (cmd_id=0x06).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `version`: 输出，固件版本字符串（如 `"1.2.3"`）/ Output: firmware version string
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 文件传输 / File Transfer

#### `GetDeviceFile`

```cpp
OdinResult GetDeviceFile(OdinDeviceHandle device, OdinUpgradeProgressCallback cb, OdinFileType type,
                         std::string save_path, uint32_t timeout_ms = 5000);
```

从设备获取文件（同步阻塞）。对于标定文件，优先使用 TCP 通道（0x41），失败时回退到 HTTP/HTTPS。阻塞直到文件保存到 `save_path` 或传输失败/超时。

Get file from device (synchronous/blocking). For calibration files, prefers the active TCP connection (0x41 channel) and falls back to HTTP/HTTPS. Blocks until the file is saved or transfer fails/times out.

- **进度回调约定 / Progress callback convention**:
  - `100.0`: 传输完成，文件已保存 / Transfer complete, file saved
  - `0..99`: 增量进度 / Incremental progress
- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `cb`: 进度回调（可为 `nullptr`）/ Progress callback (may be `nullptr`)
  - `type`: 文件类型（`kCalibrationFile`, `kRelocationMap`, `kDevLogFile`）/ File type
  - `save_path`: 本地保存路径 / Local path to save the file
  - `timeout_ms`: 0x41 通道传输超时（默认 5000 ms）/ Timeout for 0x41 channel transfer (default: 5000 ms)
- **返回值 / Returns**: `kOk` 文件已保存；否则返回错误码 / `kOk` when file saved; error code otherwise

#### `SendFileToDevice`

```cpp
OdinResult SendFileToDevice(OdinDeviceHandle device, OdinUpgradeProgressCallback cb,
                            OdinFileType type, std::string file_path);
```

向设备发送文件。对于固件升级（`kFirmware` 类型），处理完整 OTA 流程：自动选择 HTTP/HTTPS、管理 OTA 状态机、进度回调报告 0-100%。

Send file to device. For firmware upgrade (`kFirmware` type), handles the complete OTA process: auto-selects HTTP/HTTPS, manages OTA state machine, progress callback reports 0-100%.

- **固件升级进度分布 / Firmware upgrade progress breakdown**:
  - 0-50%: 固件上传 / Firmware upload
  - 50-60%: 验证 / Verification
  - 60-80%: 安装 (MCU/SOC) / Installation
  - 80-95%: 重启 / Reboot
  - 95-100%: 升级后验证 / Post-verification
  - 100%: 成功 / Success
- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `cb`: 进度回调函数 (0-100%) / Progress callback function
  - `type`: 文件类型（`kFirmware` 用于 OTA 升级）/ File type (`kFirmware` for OTA)
  - `file_path`: 本地文件路径 / Path to local file
- **返回值 / Returns**: 操作结果 / Operation result

---

### 热插拔监听 / Hotplug Listener

#### `StartHotplugListener`

```cpp
bool StartHotplugListener(const HotplugCallbacks &callbacks, bool enumerate_existing = true);
```

启动热插拔监听器，检测设备上线/下线事件。内部使用 UDP 广播发现。

Start hotplug listener to detect device arrival/removal. Uses UDP broadcast-based discovery internally.

- **参数 / Parameters**:
  - `callbacks`: 包含 `on_attach` 和 `on_detach` 回调的结构体 / Structure containing attach/detach callbacks
  - `enumerate_existing`: 若为 `true`，已连接的设备也会通过 `on_attach` 上报 / If `true`, report already-connected devices via `on_attach`
- **返回值 / Returns**: `true` 启动成功 / `true` if started successfully

#### `StopHotplugListener`

```cpp
void StopHotplugListener();
```

停止热插拔监听器 / Stop hotplug listener.

#### `IsHotplugListenerRunning`

```cpp
bool IsHotplugListenerRunning();
```

检查热插拔监听器是否正在运行 / Check if hotplug listener is running.

- **返回值 / Returns**: `true` 正在运行 / `true` if running

#### `SetHotplugPollingInterval`

```cpp
void SetHotplugPollingInterval(uint32_t interval_ms);
```

设置热插拔轮询间隔 / Set hotplug polling interval.

- **参数 / Parameters**: `interval_ms` — 轮询间隔（毫秒，默认 500）/ Interval in ms (default: 500)

#### `SetHotplugOfflineTimeout`

```cpp
void SetHotplugOfflineTimeout(uint32_t timeout_ms);
```

设置热插拔下线超时 / Set hotplug offline timeout.

- **参数 / Parameters**: `timeout_ms` — 超时时间（毫秒，默认 3000）/ Timeout in ms (default: 3000)

---

## 扩展 API / Extended API

> 头文件 / Header: `odin_lidar_api_ext.h`（自动引入核心 API / Automatically includes core API）

### 通道查询 / Channel Query

#### `GetChannelConfig`

```cpp
OdinResult GetChannelConfig(OdinDeviceHandle device, OdinDataChannel channel,
                            OdinChannelConfigInfo& info, uint32_t timeout_ms = 1000);
```

查询指定通道的订阅状态（cmd_id=0x06）。返回通道的完整配置快照，包括订阅状态、目标端口、传输模式、分辨率和帧率。

Query per-channel subscription state (cmd_id=0x06). Returns the full configuration snapshot for the specified channel.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `channel`: 要查询的通道 / Channel to query
  - `info`: 输出，通道配置信息 / Output: channel config info
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 点云滤波参数 / LiDAR Filter Parameters

#### `SetFilterParam`

```cpp
OdinResult SetFilterParam(OdinDeviceHandle device, const OdinLidarFilterParams& params,
                          uint32_t timeout_ms = 1000);
```

设置点云滤波参数。仅 `has_xxx=true` 的滤波器会被应用，其余保持设备当前值。参数在下一帧边界生效，不中断数据流。

Set LiDAR filter parameters. Only filters with `has_xxx=true` are applied; others are left unchanged. Parameters take effect at the next frame boundary without interrupting data flow.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `params`: 滤波参数集（至少一个 `has_xxx` 必须为 `true`）/ Filter parameter set (at least one `has_xxx` must be true)
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetFilterParam`

```cpp
OdinResult GetFilterParam(OdinDeviceHandle device, OdinLidarFilterParams& params,
                          uint32_t timeout_ms = 1000);
```

查询当前点云滤波参数。成功时 `params` 填充设备当前值，对应 `has_xxx` 设为 `true`。

Query current LiDAR filter parameters. On success, `params` is populated with current values and `has_xxx` is set to `true` for each returned filter.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `params`: 输出，当前滤波参数值 / Output: current filter parameter values
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 自适应参数 / Adaptive Parameters

#### `SetAdaptiveParam`

```cpp
OdinResult SetAdaptiveParam(OdinDeviceHandle device, const OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms = 1000);
```

设置 LiDAR 自适应模式参数（cmd_id=0x34）。配置 CDF 百分位和室外阈值。仅在 LiDAR 模式为 ADAPTIVE（模式 3）时生效。

Set LiDAR adaptive mode parameters (cmd_id=0x34). Configures CDF percentile and outdoor threshold. Only active when LiDAR mode is ADAPTIVE (mode 3).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `param`: 自适应参数 / Adaptive parameters
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetAdaptiveParam`

```cpp
OdinResult GetAdaptiveParam(OdinDeviceHandle device, OdinAdaptiveFilterParam& param,
                            uint32_t timeout_ms = 1000);
```

查询 LiDAR 自适应模式参数（cmd_id=0x35）。

Query LiDAR adaptive mode parameters (cmd_id=0x35).

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `param`: 输出，当前自适应参数 / Output: current adaptive parameters
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 时间同步 / Time Synchronization

#### `SetDefaultSyncMode`

```cpp
OdinResult SetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode mode,
                              uint32_t timeout_ms = 1000);
```

设置默认时间同步模式（cmd_id=0x20）。设置持久化到设备自定义 YAML 配置，需重启设备生效。

Set default time synchronization mode (cmd_id=0x20). The setting is persisted to the device's custom YAML configuration. A reboot is required for the change to take effect.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 同步模式（`kSysTick`=系统时钟, `kPtp`=PTP 硬件时间戳）/ Sync mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetDefaultSyncMode`

```cpp
OdinResult GetDefaultSyncMode(OdinDeviceHandle device, OdinTimeSyncMode& mode,
                              uint32_t timeout_ms = 1000);
```

查询持久化的默认同步模式（cmd_id=0x21）。返回配置 YAML 中的 `sync_mode`，反映持久化配置而非实时 PTP 状态。实时 PTP 状态请使用 `GetPtpSyncStatus`。

Query persisted default sync mode (cmd_id=0x21). Returns the `sync_mode` from config YAML. This reflects persisted configuration, NOT real-time PTP status. For real-time PTP state, use `GetPtpSyncStatus`.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `mode`: 输出，当前持久化同步模式 / Output: current persisted sync mode
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetPtpSyncStatus`

```cpp
OdinResult GetPtpSyncStatus(OdinDeviceHandle device, OdinPtpSyncStatus& status,
                            uint32_t timeout_ms = 1000);
```

查询 PTP 同步状态（cmd_id=0x22）。返回实时 PTP 同步状态快照，非持久化。

Query PTP synchronization status (cmd_id=0x22). Returns a real-time snapshot of the PTP sync state. Not persisted.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `status`: 输出，PTP 同步状态字段 / Output: PTP sync status fields
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 算法指令 / Algorithm Commands

#### `ResetInitialPose`

```cpp
OdinResult ResetInitialPose(OdinDeviceHandle device, uint32_t timeout_ms = 3000);
```

重置初始位姿（cmd_id=0x70）。重置算法状态（位姿估计、地图、因子图），使 SLAM 从头初始化。不中断数据流。

Reset initial pose (cmd_id=0x70). Resets algorithm state (pose estimate, map, factor graph) so SLAM re-initializes from scratch. Does not interrupt data streaming.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `timeout_ms`: 超时（毫秒，默认 3000）/ Timeout in ms (default: 3000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 设备配置 / Device Configuration

#### `RestoreDefaultConfig`

```cpp
OdinResult RestoreDefaultConfig(OdinDeviceHandle device, uint32_t timeout_ms = 1000);
```

恢复出厂默认配置（cmd_id=0x09）。删除设备上的自定义配置文件，需重启设备使默认配置生效。

Restore factory default configuration (cmd_id=0x09). Deletes the custom configuration file on the device. A reboot is required for the default configuration to take effect.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `timeout_ms`: 超时（毫秒，默认 1000）/ Timeout in ms (default: 1000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `GetNetworkAttribute`

```cpp
OdinResult GetNetworkAttribute(OdinDeviceHandle device, NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);
```

获取设备网络属性配置。通过 HTTP API 检索当前网络设置（IP、网关、子网掩码、DHCP、PTP）。

Get device network attribute configuration. Retrieves current network settings via HTTP API.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `attr`: 输出，网络属性结构 / Output: network attribute structure
  - `timeout_ms`: 超时（毫秒，默认 3000）/ Timeout in ms (default: 3000)
- **返回值 / Returns**: 操作结果 / Operation result

#### `SetNetworkAttribute`

```cpp
OdinResult SetNetworkAttribute(OdinDeviceHandle device, const NetworkAttribute& attr,
                               uint32_t timeout_ms = 3000);
```

设置设备网络属性配置。通过 HTTP API 配置网络设置。更改在设备重启后生效。

Set device network attribute configuration. Configures network settings via HTTP API. Changes take effect after device reboot.

- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `attr`: 网络属性 / Network attribute to set
  - `timeout_ms`: 超时（毫秒，默认 3000）/ Timeout in ms (default: 3000)
- **返回值 / Returns**: 操作结果 / Operation result

---

### 透传通道 / Pass-through Channel

#### `SendAlgoData`

```cpp
OdinResult SendAlgoData(OdinDeviceHandle device, const uint8_t* data, size_t len);
```

向设备发送算法下行透传数据（0x40 通道）。单向 host→device 不透明算法字节。SDK 作为 TCP 服务器，下行监听端口和 0x40 订阅在设备连接时自动建立。

Send algorithm downlink passthrough data to the device (0x40 channel). One-way host→device opaque algorithm bytes. The SDK acts as a TCP server; the downlink listening port and 0x40 subscription are brought up automatically when the device connects.

- **快速失败 / Fail-fast**: 如果模块尚未连接到下行服务器，立即返回 `kNotInitialized`（记录一次警告日志），SDK 不会缓冲或重试。
- **Fail-fast**: If the module has not connected to the downlink server yet, returns `kNotInitialized` immediately (a one-time warning is logged); the SDK does NOT buffer or retry.
- **参数 / Parameters**:
  - `device`: 设备句柄 / Device handle
  - `data`: 不透明算法字节（必须非空）/ Opaque algorithm bytes (must be non-null)
  - `len`: 字节计数 (1 .. 256 KB) / Byte count (1 .. 256 KB)
- **返回值 / Returns**:
  - `kOk`: 成功 / Success
  - `kNotInitialized`: 模块未连接 / Module not connected
  - `kInvalidArgument`: 空指针或大小违规 / Null/size violations
  - `kSocketError`: 写入失败 / Write failure

---

## 典型使用流程 / Typical Usage Flow

以下示例展示从设备发现到数据接收的完整流程。

The following example demonstrates the complete flow from device discovery to data reception.

```cpp
#include "odin_lidar_api_ext.h"

using namespace odin::sdk;

// 1. Define callbacks
void OnPointCloud(const OdinPointCloudPacket& pkt, void* ctx) {
    // Process raw point cloud
    printf("Points: %zu, frame: %u\n", pkt.points.size(), pkt.frame_count);
}

void OnImu(const OdinImuPacket& pkt, void* ctx) {
    // Process IMU data
}

void OnOdom(const OdinOdomPacket& pkt, OdomSourceType type, void* ctx) {
    // Process odometry
}

int main() {
    // 2. Discover devices
    std::vector<DiscoveredDevice> devices;
    if (!DiscoverDevices(devices, 2000)) {
        printf("No devices found\n");
        return 1;
    }

    // 3. Connect to first device
    OdinDeviceHandle handle = ConnectDevice(devices[0]);
    if (handle == kInvalidDeviceHandle) {
        printf("Connection failed\n");
        return 1;
    }

    // 4. Register callbacks
    RegisterPointCloudCallback(handle, OnPointCloud, nullptr);
    RegisterImuCallback(handle, OnImu, nullptr);
    RegisterOdomCallback(handle, OnOdom, nullptr);

    // 5. Enter standby for configuration
    SetDeviceMode(handle, OdinDeviceMode::kStandby);

    // 6. Set LiDAR sensor mode
    SetSensorMode(handle, OdinSensorMode::kHighPeak);

    // 7. Start data streams
    StartStream(handle, OdinDataChannel::kRawPoint, OdinTransportMode::kUdp);
    StartStream(handle, OdinDataChannel::kImu, OdinTransportMode::kUdp);
    StartStream(handle, OdinDataChannel::kOdomLF, OdinTransportMode::kUdp);

    // 8. Switch to normal mode to begin streaming
    SetDeviceMode(handle, OdinDeviceMode::kNormal);

    // 9. ... receive data via callbacks ...

    // 10. Cleanup
    SetDeviceMode(handle, OdinDeviceMode::kStandby);
    CloseStream(handle, OdinDataChannel::kRawPoint);
    CloseStream(handle, OdinDataChannel::kImu);
    CloseStream(handle, OdinDataChannel::kOdomLF);
    DisconnectDevice(handle);

    return 0;
}
```

### 热插拔模式 / Hotplug Mode

```cpp
#include "odin_lidar_api_ext.h"

using namespace odin::sdk;

void OnAttach(const DiscoveredDevice& dev) {
    printf("Device online: %s (%s)\n", dev.sn.c_str(),
           dev.network.ip_address.c_str());
    auto handle = ConnectDevice(dev);
    // Register callbacks, start streams, set normal mode...
}

void OnDetach(const DiscoveredDevice& dev) {
    printf("Device offline: %s\n", dev.sn.c_str());
    // Disconnect and release resources...
}

int main() {
    HotplugCallbacks cbs;
    cbs.on_attach = OnAttach;
    cbs.on_detach = OnDetach;

    StartHotplugListener(cbs, true);  // enumerate existing devices

    // Main loop...
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    StopHotplugListener();
    return 0;
}
```

---

## 错误码 / Result Codes

| 结果码 / Result Code | 枚举值 / Enum | 说明 / Description |
|---|---|---|
| 成功 / Success | `OdinResult::kOk` (0) | 操作成功完成 / Operation completed successfully |
| 未初始化 / Not Initialized | `OdinResult::kNotInitialized` (1) | SDK 未初始化或设备未连接 / SDK not initialized or device not connected |
| 无效参数 / Invalid Argument | `OdinResult::kInvalidArgument` (2) | 传入参数无效 / Invalid argument passed |
| 套接字错误 / Socket Error | `OdinResult::kSocketError` (3) | 网络套接字错误 / Network socket error |
| 超时 / Timeout | `OdinResult::kTimeout` (4) | 操作超时 / Operation timed out |
| 已停止 / Stopped | `OdinResult::kStopped` (5) | 操作被停止 / Operation was stopped |
| 固件非法 / Firmware Illegal | `OdinResult::kFirmwareIllegal` (6) | 无效固件文件 / Invalid firmware file |
| 通信错误 / Communication Error | `OdinResult::kCommunicationError` (7) | 通信错误 / Communication error |
| 未知错误 / Unknown Error | `OdinResult::kUnknownError` (8) | 未知错误 / Unknown error |

> 建议在每次 API 调用后检查返回值，并根据 `OdinResult` 进行错误处理。
>
> It is recommended to check the return value after each API call and handle errors based on `OdinResult`.
