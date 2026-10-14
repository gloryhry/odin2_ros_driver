# Odin 通信协议规范

本文档描述 Odin 雷达与上位机之间的通信协议。

- **设备发现**: UDP 广播 (0x01 DeviceQuery)
- **心跳连接**: TCP 独立通道 (0x07 Heartbeat)
- **命令控制**: TCP 连接 (VersionQuery / GetSensorCapability / SetMode / SetLiDARMode / ChannelConfig)
- **数据推送**: UDP 或 TCP（每通道独立配置）
- **算法下行透传**: TCP（host 作 server，设备 connect 接收；`data_type=0x40`，host 算法 → device 算法单向）

> **传输承载（version 0x01 起）**：本协议可承载于 **网络**（UDP/TCP）或 **USB**（厂商自定义 FunctionFS，bulk/interrupt 端点），按设备/配置二选一。下文涉及端口 / IP / connect 的描述为**网络承载**语义；**USB 承载**下的差异见各节「USB 承载」标注与 §1.4。

[TOC]

## 1. 控制协议

| 端口 | 协议 | 用途 |
| --- | --- | --- |
| 60001 | UDP | 设备广播发现 (DeviceQuery) |
| 60001 | TCP | 命令控制通道 |
| 60002 | TCP | 心跳通道 |

- **UDP 60001**: 仅处理 `0x01 DeviceQuery` 广播发现。设备已被心跳占用时静默忽略
- **TCP 60001**: 命令控制（单客户端策略，断连自动停流）
- **TCP 60002**: 心跳维持（单客户端策略，超时触发全重置）

> **USB 承载**：无物理端口，60001 / 60002 退化为**逻辑控制通道**（命令 / 心跳）；设备发现 = USB 枚举（VID/PID/serial），不使用 UDP 广播。控制帧与数据帧全部 in-band 走 USB 端点。

### 1.1 帧格式

| 字段 | 长度 | 描述 |
| --- | --- | --- |
| sof | 1 | 起始字节，固定为 0xAE |
| version | 1 | 协议版本（**0x01** = 订阅/端口解耦语义，subscribe TLV 权威；0x00 为旧版网络语义，已废弃） |
| length | 2 | 数据帧长度（从 sof 到 data 段结束） |
| seq_num | 4 | 序列号，REQ 递增，ACK 与 REQ 相同 |
| cmd_id | 2 | 指令 ID |
| cmd_type | 1 | 0x00: REQ（请求），0x01: ACK（响应） |
| send_type | 1 | 0: 上位机发送，1: 设备发送 |
| crc16 | 2 | 包头校验码（sof 到 send_type 共 12 字节） |
| crc32 | 4 | data 段校验码 |
| data | n | 负载数据（最大 1454 字节） |

### 1.2 返回码

ACK 消息的 data 段首字节为返回码：

| ret_code | 说明 |
| --- | --- |
| 0 | 成功 |
| 1 | 失败 |
| 2 | 参数错误 |

### 1.3 指令列表

#### 0x01 设备查询 [RT]

**REQ** (Host → Device, 广播)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| host_ip | uint8_t[4] | 上位机 IP 地址 |

**ACK** (Device → Host, 广播)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |
| sn | uint8_t[16] | 设备序列号 |
| device_ip | uint8_t[4] | 设备 IP 地址（应答时刻网卡实际 IPv4；**USB 承载下置 0，host 忽略**） |
| model | uint8_t[32] | 设备型号/模组名（NUL 填充定长 32B；来源于设备 YAML `device.name`，如 `Odin2Special`） |
| device_mac | uint8_t[6] | 设备网卡 MAC（**version 0x01 扩展，追加在 model 尾部**；旧 host 按 53B 解析时忽略；多设备 IP 冲突时凭 MAC 区分并强制访问指定设备） |

> **ACK 长度版本**：data 段 21B（ret+sn+ip）为旧布局；53B（+model）为过渡布局；59B（+device_mac）为当前布局。host 应按实际收到的 data 长度解析，尾部未知字段一律忽略。

> **USB 承载**：设备发现由 USB 枚举（VID/PID/serial）完成；DeviceQuery/ACK 仍可 in-band 走控制端点获取 `sn` / `model`，`device_ip` / `device_mac` 无意义（置 0）。

#### 0x02 版本查询 (VersionQuery)（TCP）[RT]

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |
| firmware_version_major | uint8_t | 主版本号 |
| firmware_version_minor | uint8_t | 次版本号 |
| firmware_version_patch | uint8_t | 修订号 |
| release_notes | char[64] | 版本说明 |

#### 0x03 设备模式设置 (SetDeviceMode)（TCP）[RT]

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| target_mode | uint8_t | 0x00: 待机，0x01: 正常工作 |

- `Normal`: 设备进入工作状态（门控开启）。若已有通道订阅配置，立即按已有配置恢复推流；若无配置则等待 `0x05 ChannelConfig` 订阅
- `Standby`: 暂停传感器流通道（raw/slam/image/imu/odom/tf）推流，但保持 TCP 数据连接不断开，保留全部通道订阅配置。再次切回 Normal 时零延迟恢复

> SetMode 仅作为传感器流通道的全局推流门控，不影响通道订阅状态。服务通道（calib 0x41、telemetry 0x42）不受 SetMode 门控，只要已订阅即可推送。TCP 命令连接断开时才完全重置所有配置。

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |
| current_mode | uint8_t | 当前模式 |

#### 0x04 设备模式查询 (GetDeviceMode)（TCP）[RT]

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |
| current_mode | uint8_t | 0x00: Standby, 0x01: Normal |

#### 0x05 通道订阅配置 (SetChannelConfig)（TCP）[RT]

按通道订阅/取消数据推送（订阅由 `subscribe` TLV 权威决定），并选择传输模式 (UDP/TCP/USB)。

**REQ** (Host → Device, TCP)

```
[data_type: uint8_t]  ← 分段编号 (0x00/0x01/0x10/0x11/0x20/0x30/0x31/0x32/0x40)
[TLV 0] [type: uint8_t] [length: uint8_t] [value: N bytes]
[TLV 1] ...
```

`data_type` 按类别分段编号，每段预留 16 个位置：

| data_type | 数据内容 | 内部索引 | 端口 |
| --- | --- | --- | --- |
| 0x00 | 原始点云 | 0 | 由上位机配置 |
| 0x01 | SLAM 点云 | 1 | 由上位机配置 |
| 0x10 | JPEG 图像（左） | 2 | 由上位机配置 |
| 0x11 | JPEG 图像（右） | 3 | 由上位机配置 |
| 0x20 | IMU 数据 | 4 | 由上位机配置 |
| 0x30 | 位姿低频（odom_lf） | 5 | 由上位机配置 |
| 0x31 | 位姿高频（odom_hf） | 6 | 由上位机配置 |
| 0x32 | TF 数据 | 10 | 由上位机配置 |
| 0x40 | 算法下行透传（host→device） | 7 | 由上位机配置（host 作 server） |

> 0x30/0x31 为独立订阅通道，各自拥有 dst_port。0x40~0x4F 为**功能段**（包含下行透传 0x40、标定文件 0x41、遥测 0x42）。


| type | length | value | 描述 |
| --- | --- | --- | --- |
| `0x01` | 1 | uint8_t | `subscribe`: **1=订阅，0=取消订阅；本 TLV 必填**（唯一订阅权威，网络/USB 统一） |
| `0x02` | 2 | uint16_t (BE) | `dst_port`: 目标端口，**仅 UDP/TCP 路由用**；USB 忽略；**不再表示订阅** |
| `0x03` | 1 | uint8_t | `transport_mode`: 0=UDP（默认），1=TCP；**仅网络承载有效**；USB 承载时设备忽略此 TLV |
| `0x04` | 1 | uint8_t | `resolution_id`: 能力表分辨率索引（0xFF=未指定）。对所有通道有效，不在 Image 。 |
| `0x05` | 1 | uint8_t | `fps_id`: 所选 resolution_id 下的帧率子列表索引（0xFF=未指定）。对所有通道有效。 |
| `0x06` | 1 | uint8_t | `format`: OdinDataFormat 值（仅 Image 通道，0xFF=未指定）。host 指定期望的线缆格式，设备按该格式编码输出 |

**行为逻辑**（version 0x01：订阅由 `subscribe` 权威决定，与端口无关）:
- **缺 `subscribe` TLV**: 参数错误，ACK `ret_code=2`
- `subscribe=1` + 网络承载 + `transport_mode=0(UDP)`: 需 `dst_port>0`，设备通过 UDP sendto 推送到 host_ip:dst_port
- `subscribe=1` + 网络承载 + `transport_mode=1(TCP)`: 需 `dst_port>0`，设备主动 TCP connect 到 host_ip:dst_port
- `subscribe=1` + 网络承载 + **缺 `dst_port`**: 参数错误，ACK `ret_code=2`
- `subscribe=1` + **USB 承载**: 设备自动路由到该 `data_type` 对应的固定 USB 端点（由设备端 `usb_topology` 静态决定）；`transport_mode` 与 `dst_port` 均忽略
- `subscribe=0`: 取消订阅该通道，关闭 TCP 数据连接 / 停止 USB 端点流（`dst_port` 无关）
- `resolution_id` / `fps_id`: 必须在能力表有效索引范围内，否则 ACK ret_code=1

> **承载选择原则**：USB vs 网络由设备物理连接与配置决定（YAML `device.{net,usb}.enable` 互斥），而非由 host 的 `transport_mode` TLV 选择。`transport_mode` 只在网络承载下有意义，用于 host 按应用场景选择 UDP（低延迟/允许丢）或 TCP（可靠/有序）。

**0x40 算法下行透传（下行，方向与上述上行通道相反）**:
- 用途：host 算法 → device 算法的单向下行数据。
- `subscribe=1` + 网络承载 + `transport_mode=1(TCP)`: host 作 **server** 在 `dst_port` 监听，设备作 **client** TCP connect 到 host_ip:dst_port 接收下行（TCP 端到端反压，零丢失）。
- `subscribe=1` + 网络承载 + `transport_mode=0(UDP)`: **不支持**，ACK ret_code=1。
- `subscribe=1` + **USB 承载**: 设备经 USB OUT 端点（`usb_topology` 静态定义）接收下行（bulk 链路级可靠），`transport_mode` 与 `dst_port` 均忽略。
- `subscribe=0`: 取消下行，关闭该连接 / 停止 USB 端点流。
- （TCP）host 须**先**在 `dst_port` 监听再下发本命令；设备 connect 失败一次即放弃（**无重试/退避**），ACK ret_code=1。
- payload 对设备**不透明**：设备按 §2.4 帧头定界重组后原样转发给设备算法进程（经 IPC），多算法流由算法端在 payload 内自定义头解复用。

**0x41 标定文件下载（上行，device→host）**:
- 用途：host 获取设备标定文件（`calib.yaml`）。
- 订阅后设备立即将**整个文件作为一个逻辑帧**推送；`data_frame_header` 的 `chunk_count/chunk_index` 字段提供分片重组信息，host 端 reassembler 收齐所有分片后直接得到完整文件字节流，无应用层分块 header。
- **须用 TCP**（`transport_mode=1`），文件数据不允许丢失。
- 推送完毕后通道保持订阅状态（空闲），re-subscribe 可重新推送。

**0x42 遥测数据（上行，device→host，周期流）**:
- 用途：host 获取设备实时遥测快照（CPU/内存/温区/负载等）。
- 订阅后设备以 **1Hz** 周期推送 JSON 帧：
  ```json
  {"uptime_s":123,"cpu_pct":45,"mem_used_kb":1024,"mem_total_kb":2048,
   "mem_use_rate":50,"load_1":0.50,"load_5":0.45,"load_15":0.42}
  ```
- 与 HTTP `/api/monitor` 同源，网络/USB 承载均可。

**模式交互**:
- **Normal 模式下**：ChannelConfig 立即生效，配置后立即开始/停止推流
- **Standby 模式下**：传感器流通道仅保存配置（含建立 TCP 数据连接），但不推送数据。切回 Normal 后按已有配置自动恢复推流。服务通道（calib 0x41、telemetry 0x42）不受 Standby 影响，订阅后即推送
- **TCP 命令连接断开**：完全重置所有通道配置 + 关闭所有 TCP 数据连接

设备收到未知 TLV type 时跳过 (`offset += 2 + length`)，保证向前兼容。

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败, 2=参数错误 |

> 订阅结果可通过 `0x06 GetChannelConfig` 按需查询。

#### 0x06 通道订阅状态查询 (GetChannelConfig)（TCP）[RT]

查询指定通道的当前完整配置快照，返回与 SetChannelConfig REQ 对称的 TLV 格式。

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| data_type | uint8_t | 查询的通道 wire data_type（如 0x00/0x10/0x20 等） |

**ACK** (Device → Host, TCP)

```
[ret_code:  1B]   0=成功, 2=参数错误（未知 data_type）
[data_type: 1B]   与 REQ 相同（echo）
[TLV 0x01: subscribe]       3B，必填：当前订阅状态
[TLV 0x02: dst_port]        4B，subscribed=1 且 port≠0 时含
[TLV 0x03: transport_mode]  3B，subscribed=1 时含
[TLV 0x04: resolution_id]   3B，已配置（≠0xFF）时含
[TLV 0x05: fps_id]          3B，已配置（≠0xFF）时含
```

TLV 编码格式与 SetChannelConfig REQ 对称，ACK 包含完整当前配置。host 可将 ACK 内容直接用于重放订阅。

#### 0x07 心跳 (Heartbeat)（TCP 60002）[RT]

Host 周期性发送心跳，设备回传当前状态。详细设计见 `heartbeat-design.md`。

**REQ** (Host → Device, TCP 60002)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| heartbeat_interval_ms | uint16_t | Host 心跳发送间隔(ms)，建议值 1000 |

**ACK** (Device → Host, TCP 60002)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 (0=成功) |
| uptime_s | uint32_t | 设备运行时间(秒) |

**超时策略**：设备端若连续 `3 × heartbeat_interval_ms`（默认 3000ms）未收到心跳 → 停流 + 断开命令 TCP + 重置所有配置 + 关闭心跳 TCP + 恢复响应 DeviceQuery。

**设备独占**：心跳 TCP 连接建立后，设备静默忽略 UDP DeviceQuery 广播。心跳断开后恢复可发现。

#### 0x08 设备能力查询 (GetDeviceCapability)（TCP）[RT]

查询设备传感器能力表（分辨率、帧率、数据格式等）。

**REQ** (Host → Device, TCP)

无数据（查询所有通道），或 `data_type` 列表（查询指定通道）。

**ACK** (Device → Host, TCP)

Payload 为 **JSON 文本**（UTF-8 编码，格式化输出便于调试）：

```json
{
  "channels": [
    {
      "data_type": 0,
      "format": 16,
      "default": [0, 0],
      "resolutions": [
        { "width": 256, "height": 192, "fps": [10] }
      ]
    },
    {
      "data_type": 1,
      "format": 32,
      "default": [0, 0],
      "resolutions": [
        { "width": 256, "height": 192, "fps": [10] }
      ]
    },
    {
      "data_type": 16,
      "format": 1,
      "default": [0, 0],
      "resolutions": [
        { "width": 640, "height": 544, "fps": [30] },
        { "width": 1280, "height": 1088, "fps": [30] }
      ]
    },
    {
      "data_type": 17,
      "format": 1,
      "default": [0, 0],
      "resolutions": [
        { "width": 640, "height": 544, "fps": [30] },
        { "width": 1280, "height": 1088, "fps": [30] }
      ]
    },
    {
      "data_type": 32,
      "format": 48,
      "default": [0, 0],
      "resolutions": [
        { "width": 0, "height": 0, "fps": [400] }
      ]
    },
    {
      "data_type": 48,
      "format": 64,
      "default": [0, 0],
      "resolutions": [
        { "width": 0, "height": 0, "fps": [10] }
      ]
    },
    {
      "data_type": 49,
      "format": 64,
      "default": [0, 0],
      "resolutions": [
        { "width": 0, "height": 0, "fps": [400] }
      ]
    },
    {
      "data_type": 50,
      "format": 80,
      "default": [0, 0],
      "resolutions": [
        { "width": 0, "height": 0, "fps": [10] }
      ]
    }
  ]
}
```

**字段说明**：
- `data_type`: 通道类型 (0x00/0x01/0x10/0x11/0x20/0x30/0x31/0x32)
- `format`: OdinDataFormat 值（见下表）。Image 通道上报设备支持的线缆格式（H264 / MJPEG / Raw NV12）
- `default`: [resolution_id, fps_id] — 设备建议的默认配置
- `resolutions`: 分辨率/帧率能力列表
  - `width`, `height`: 分辨率尺寸（无分辨率特征的通道如 IMU/Odom 使用 0）
  - `fps`: 该分辨率支持的帧率数组，索引即 fps_id

所有通道均包含 `default` 和 `resolutions` 字段，结构统一。

**OdinDataFormat 统一定义**:

| format 值 | 含义 | 适用通道 |
| --- | --- | --- |
| 0x01 | NV12（不进行编码，RawData 透传） | Image |
| 0x02 | MJPEG（JPEG 压缩） | Image |
| 0x04 | H264 | Image |
| 0x08 | H265 | Image |
| 0x10 | XYZIC (uint16 压缩坐标, 8B/点) | RawPoint |
| 0x20 | XYZRGBA (uint16 压缩坐标, 10B/点) | SlamPoint |
| 0x30 | 6-axis (Gyro + Acc) | IMU |
| 0x40 | Standard | Odom |
| 0x50 | Standard TF | TF |

> **Image 通道**采用位掩码表示支持的线缆格式（可 OR 组合：`0x07 = RAW|MJPEG|H264`）。能力表 `format` 字段上报设备支持的完整位掩码；host 通过 `SetChannelConfig (0x05)` 下发单一格式位选择。
> 注：`OdinDataFormat` 与 AWVideoEncoder `CodecType`（H264=0, JPEG=1）编号不同，下位机内部通过 `to_codec_type()` 函数映射。

能力表由设备端从 YAML 配置（`odin_default.yaml` / 自定义覆盖）构建（见 `build_capability_table`）。图像通道上报的 `width`/`height` 为旋转后输出几何，`resolutions` 顺序为低分在前（id=0 低分，作为 `default`）。LiDAR 模式通过独立的 `0x30 SetLiDARMode` 命令配置。

#### 0x09 恢复默认配置 (RestoreDefaultConfig)（TCP）[PT]

删除 custom 配置文件，恢复系统出厂默认配置。**需 host 重启设备**使默认配置生效。

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败 |

**行为**：设备遍历 custom 配置候选路径（`/userdata/cfg/odin_custom.yaml`、`/oem/cfg/odin_custom.yaml`），删除所有存在的 custom 配置文件。ACK 返回后 host 须发送 Reboot 命令重启设备，设备重启后因无 custom YAML 而回退到 default 配置。

---

#### 时间同步命令组 (0x20-0x2F)

本组命令用于控制设备时间同步模式及查询 PTP 同步状态。后续时间同步相关命令均在此 group 中扩展。

#### 0x20 设置默认同步模式 (SetDefaultSyncMode)（TCP）[PT]

设置设备的默认时间同步模式。**强制持久化**到 custom YAML 配置文件，**必须重启后生效**。

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| sync_mode | uint8_t | 0x00: sys_tick（系统时钟，默认）；0x01: PTP 硬件时间戳同步 |

- 未知 sync_mode 值 → ACK ret_code=2
- 本命令恒持久化到 custom YAML，无 save_config 参数

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败, 2=参数错误 |
| current_sync_mode | uint8_t | 写入后的同步模式（echo，供 host 确认） |

**行为**：设备将 `sync_mode` 写入配置并调用 `config_save_custom()` 持久化。ACK 返回后 host 须发送 Reboot 命令使配置生效。

#### 0x21 查询默认同步模式 (GetDefaultSyncMode)（TCP）[RT]

查询设备当前配置的时间同步模式（读取持久化配置，非运行时状态）。

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败 |
| sync_mode | uint8_t | 0x00: sys_tick（系统时钟）；0x01: PTP 硬件时间戳同步 |

**行为**：设备读取当前配置中的 `sync_mode` 并返回。该值反映持久化配置（default 或 custom YAML），与运行时实际同步状态无关。运行时 PTP 同步状态请使用 0x22 GetPtpSyncStatus。

#### 0x22 查询 PTP 同步状态 (GetPtpSyncStatus)（TCP）[RT]

查询当前 PTP 同步状态（实时快照，非持久化）。

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败(ptp_daemon未运行/未启用) |
| sync_state | uint8_t | 0=searching, 1=locking, 2=locked, 3=error |
| servo_state | uint8_t | ptp4l 原始伺服状态 0/1/2 |
| offset_ns | int64_t | Master-Slave 偏移量（纳秒） |
| path_delay_ns | int64_t | 路径延迟（纳秒） |
| freq_ppb | int32_t | 频率调整值（ppb） |

ACK 总长 = 1 + 1 + 1 + 8 + 8 + 4 = 23B。字段来源于 `ptp_monitor` 的 `servo_state()` / `offset_ns()` / `path_delay_ns()` / `freq_ppb()`，`sync_state` 由 `determine_state()` 派生。

---

#### LiDAR 命令组 (0x30-0x4F)

本组命令用于 LiDAR 模式控制及滤波参数配置。后续 LiDAR 相关命令均在此 group 中扩展。

#### 0x30 LiDAR 模式设置 (SetLiDARMode)（TCP）[RT]

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| sensor_id | uint8_t | 传感器序号（保留） |
| mode | uint8_t | 0: HDR, 1: High Peak, 2: Low Peak, 3: Adaptive |

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |

#### 0x31 LiDAR 模式查询 (GetLiDARMode)（TCP）[RT]

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 返回码 |
| mode | uint8_t | 0: HDR, 1: High Peak, 2: Low Peak, 3: Adaptive |

#### 0x32 LiDAR 滤波参数设置 (SetLiDARFilterParam)（TCP）[RT]

> **当前状态**：保留命令接口，返回 generic ACK（ret_code=0）。confidence 滤波参数由 YAML 配置决定，不通过协议运行时下发。两级 TLV 结构保留供未来扩展其他 filter。

**REQ** (Host → Device, TCP)

```
[filter_id: uint8_t] [length: uint8_t] [sub-TLV block: length bytes]
  └─ sub-TLV block = [sub_key: uint8_t] [sub_len: uint8_t] [value: sub_len bytes] ...
```

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功（no-op） |

#### 0x33 LiDAR 滤波参数查询 (GetLiDARFilterParam)（TCP）[RT]

> **当前状态**：保留命令接口，返回 generic ACK（ret_code=0，空 TLV）。confidence 滤波参数由 YAML 配置决定。

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| filter_ids | uint8_t[] | 要查询的 filter_id 列表（空 = 查询全部） |

**ACK** (Device → Host, TCP)

```
[ret_code: 1B]
```

#### 0x34 自适应参数设置 (SetAdaptiveParam)（TCP）[RT]

设置 LiDAR 自适应模式（Adaptive）的判定参数。仅在 LiDAR 工作模式为 ADAPTIVE（模式 3）时生效，其他模式下参数保存但不运行。

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| cdf_percentile | uint16_t BE | 万分数 1-10000，CDF 距离百分位（默认 7000 = 70.0%） |
| outdoor_threshold | uint16_t BE | 室外判定距离阈值(mm)（默认 3000 = 3.0m） |

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败, 2=参数错误 |

**参数说明**：

- `cdf_percentile`（uint16_t BE, 万分数 1-10000）：对每帧有效点（0.01m < distance < 20.0m）构建 0.1m 宽度距离直方图，沿距离轴累加至指定百分位，该距离即 CDF 距离。百分位越高，代表场景大部分点距离越远。70%（7000）为当前默认；降低（如 50%）更敏感于近距分布，升高（如 90%）更关注远距点。

- `outdoor_threshold`（uint16_t BE, mm）：CDF 距离 ≥ 此阈值 → 判定室外，否则室内。室外推荐 HIGHPEAK 模式，室内推荐 LOWPEAK 模式。连续 10 帧判定一致才触发切换。增大（如 5000mm）需更远分布才判室外，减少误切换但延迟检测；减小（如 2000mm）更易判室外，响应快但可能误判。

**语义**：轻量操作，不中断数据流。参数在下一帧边界生效。参数取值超界时 ACK ret_code=2。

#### 0x35 自适应参数查询 (GetAdaptiveParam)（TCP）[RT]

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功 |
| cdf_percentile | uint16_t BE | 当前 CDF 距离百分位 |
| outdoor_threshold | uint16_t BE | 当前室外判定距离阈值(mm) |

---

#### Camera 命令组 (0x50-0x5F)

本组命令预留给 Camera 相关控制（编码参数、曝光、白平衡等）。后续 Camera 相关命令均在此 group 中扩展。

---

#### 算法命令组 (0x70-0x7F)

本组命令用于算法进程控制（初始位姿重置等）。后续算法相关命令均在此 group 中扩展。

#### 0x70 初始位姿重置 (ResetInitialPose)（TCP）[RT]

通知设备重置算法初始位姿（清空位姿估计、地图、因子图等），使其从零开始重新初始化。**不中断数据流**，重置在下一帧边界生效。

**REQ** (Host → Device, TCP)

无数据

**ACK** (Device → Host, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | 0=成功, 1=失败(算法进程未运行) |

**行为**：设备向算法进程发送重置信号，清空内部状态（位姿估计、地图、因子图、关键帧等）并重新初始化。重置后 SLAM 点云通道 (`data_type=0x01`) 继续推流，但输出从初始位姿重新开始。host 须在算法丢失跟踪或需要重新建图时调用。

---

### 1.4 连接生命周期

完整的连接建立顺序（**必须严格按序**）：

1. Host 广播 UDP DeviceQuery (60001) 发现设备
2. Host TCP 连接心跳端口 (60002)
3. Host 发送首次 Heartbeat REQ，设备回 ACK，设备进入"已占用"状态
4. Host TCP 连接命令端口 (60001)（**心跳未建立时设备不接受命令连接**）
5. Host 发送 SetMode / ChannelConfig 等业务指令
6. Host 持续周期发送 Heartbeat REQ 维持连接

断开场景：
- **主动断开**: Host 关闭心跳 TCP → Device 全重置
- **心跳超时**: Device 连续 3×interval 未收到心跳 → 全重置
- **命令 TCP 断开但心跳在**: 停流+重置通道配置，保持"已占用"状态（同时关闭 0x40 下行透传连接）

全重置 = 停止推流 + 关闭 TCP 数据连接 + 关闭命令 TCP + 重置通道配置 + 关闭心跳 TCP + 恢复 DeviceQuery 响应。

**USB 承载变体**：
- 物理发现 = USB 枚举（VID/PID/serial）；无 UDP 广播 / TCP 连接。
- `FUNCTIONFS_ENABLE`（host 打开接口）→ 命令 / 心跳逻辑通道 up（等价于命令 TCP + 心跳 TCP 建立）。
- `FUNCTIONFS_DISABLE` / 拔线 → 全重置（停流 + 重置通道配置 + 恢复可发现）。
- 心跳在 USB 下为**次级存活检查**（物理在线由 ENABLE/DISABLE 表达）；超时策略可保留为 app 层健康监测。

详细设计及时序图见 `heartbeat-design.md`。

---

## 2. 数据协议

### 2.1 帧格式（v2，32 字节，自然对齐）

数据面采用协议 **v2**：帧头固定 32 字节、自然对齐，首字节 `sof` 固定为 `0xAD`，第二字节 `version` 固定为 `0x02`。`payload_len` 为纯载荷长度（不含帧头），分片重组由 `chunk_index` + `chunk_count`（及 `flags` bit0 = last_chunk）驱动。

| 字段 | 偏移 | 长度 | 描述 |
| --- | --- | --- | --- |
| sof | 0 | 1 | 数据帧起始字节，固定为 0xAD |
| version | 1 | 1 | 数据面协议版本（v2 = 2） |
| data_type | 2 | 1 | 通道类型（0x00 原始点云 / 0x01 SLAM / 0x10-0x11 图像 / 0x20 IMU / 0x30 Odom / 0x32 TF / 0x40 算法下行透传 / 0x41 标定文件 / 0x42 遥测） |
| flags | 3 | 1 | bit0 = last_chunk（本帧最后一个分片）；其余位保留 |
| frame_seq | 4 | 4 | 帧序号（同一帧的所有分片相同） |
| chunk_index | 8 | 2 | 帧内分片序号，从 0 开始 |
| chunk_count | 10 | 2 | 该帧分片总数 |
| element_count | 12 | 2 | 本分片元素数（点 / 样本 / 字节） |
| payload_len | 14 | 2 | 本分片纯载荷字节数（不含 32B 帧头） |
| timestamp | 16 | 8 | 本分片首元素时间戳 (ns) |
| interval_ns | 24 | 4 | 仅 IMU：样本间隔 (ns)；其他通道为 0 |
| crc32 | 28 | 4 | 载荷 CRC32（数据面默认 0，改用结构字段校验） |
| data | 32 | payload_len | 负载数据 |

> 单分片 `payload_len` 上限 = 链路 MTU − 32（帧头），且受 `payload_len` 字段 uint16 限制 **≤ 65535B**。网络承载 MTU 较小；**USB 承载**链路 MTU 较大（bulk），单分片可取较大值（≤ 65535）以降低帧头开销。每帧按通道 `element_size` 切分为 `chunk_count` 个分片。

### 2.2 分片重组与时间戳

- **新帧判定**：`frame_seq` 变化或 `chunk_index == 0` 视为新帧开始。
- **完成判定**：收到 `flags` bit0(last_chunk) 置位，或累计分片数达到 `chunk_count`。
- **点云 / 图像**：各分片载荷按到达顺序拼接为完整帧（点云累加各分片 `element_count`；图像为完整 JPEG 字节流，**无尾部长度字段**）。
- **IMU**：每分片载荷是若干个 24B `OdinImuSample` 紧密数组；第 *j* 个样本时间戳 = `timestamp + j × interval_ns`，分片自包含、无跨片重组。
- **时间戳单位统一为纳秒 (ns)**，基于设备硬件 monotonic 时基。

### 2.3 数据结构

#### 2.3.1、原始点云 (data_type=0)

每帧 49152 点 (256×192)，按链路 MTU 切分为多个分片(chunk)

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| x | uint16_t | mm |
| y | uint16_t | mm（需 -30000 偏移） |
| z | uint16_t | mm（需 -30000 偏移） |
| intensity | uint8_t | - |
| confidence | uint8_t | - |

> 每点 8 字节，单分片元素数见帧头 `element_count`

#### 2.3.2、SLAM 点云 (data_type=1)

每帧 49152 点，按链路 MTU 切分为多个分片(chunk)

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| x | uint16_t | mm |
| y | uint16_t | mm（需 -30000 偏移） |
| z | uint16_t | mm（需 -30000 偏移） |
| r | uint8_t | - |
| g | uint8_t | - |
| b | uint8_t | - |
| a | uint8_t | - |

> 每点 10 字节，单分片元素数见帧头 `element_count`

#### 2.3.3、IMU 数据 (data_type=0x20)

每帧 1 个 IMU 数据包

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| gyro_x | float | rad/s |
| gyro_y | float | rad/s |
| gyro_z | float | rad/s |
| acc_x | float | m/s² |
| acc_y | float | m/s² |
| acc_z | float | m/s² |

> 每个 IMU 24 字节
>
> 仅支持 6-axis (Gyro + Acc)，不支持 9-axis (Gyro + Acc + Mag)。

#### 2.3.4、Odom 数据 (data_type=0x30)

每帧 1 个 Odom 数据包

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| pos | float[3] | m |
| orient | float[4] | 四元数 (x,y,z,w) |
| linear_velocity | float[3] | m/s |
| angular_velocity | float[3] | rad/s |
| cov | float[18] | - |
| type | uint8_t | - |

> 每个 Odom 125 字节
>
> **破坏性变更**：字段由 `int64_t`（定点 μm/×10⁻⁶）改为 `float`（SI 单位 m / 四元数 / m·s⁻¹ / rad·s⁻¹），单包由 249 字节缩为 125 字节。上位机解析须同步更新。

#### 2.3.5、TF 数据 (data_type=0x32)

每帧 1 个 TF 数据包，对齐 ROS2 `geometry_msgs/TransformStamped`

| 字段 | 类型 | 单位 |
| --- | --- | --- |
| translation | float[3] | m |
| rotation | float[4] | 四元数 (x,y,z,w) |
| frame_id | char[16] | 父坐标系名称（对齐 ROS2 TransformStamped.header.frame_id） |
| child_frame_id | char[16] | 子坐标系名称（对齐 ROS2 TransformStamped.child_frame_id） |

> 每个 TF 60 字节

坐标系命名对齐 [REP 105](https://www.ros.org/reps/rep-0105.html)：

| 字符串 | 含义 |
| --- | --- |
| `"odom"` | odometry 坐标系（局部里程计，短期精确，会漂移） |
| `"base_link"` | 机器人基座 |
| `"map"` | 地图坐标系（全局定位，长期稳定） |
| `"lidar"` | 激光雷达 |
| `"imu"` | 惯性测量单元 |
| `"camera0"` | 相机 0 |
| `"camera1"` | 相机 1 |

标准 TF 树拓扑：`map → odom → base_link → [传感器帧]`

### 2.4 算法下行透传帧（data_type=0x40，下行 host→device）

复用 §2.1 的 32B v2 帧头，方向为 host→device，由设备按 §2.2 规则定界重组：

- `sof=0xAD`、`version=2`、`data_type=0x40`、`flags` bit0=last_chunk、`frame_seq`/`chunk_index`/`chunk_count` 驱动重组。
- `payload_len`：本分片纯 payload 字节，**≤ 60000**（< 64KB，留 TCP/IP 头余量）。
- `timestamp`：host 侧帧时间戳 (ns)，透传，设备不解释。
- `element_count`：对透传无语义（host 可填 payload_len 或 0）；`interval_ns=0`；`crc32=0`（CRC 默认关）。
- 重组后单条消息上限（保守）**256KB**。
- **设备侧校验（fail-fast）**：帧头不变量 `sof==0xAD && version==2 && data_type==0x40 && chunk_index<chunk_count && payload_len≤60000 && 累计≤256KB`；任一不满足或链路异常 → 设备丢弃在途数据并关闭连接，等 host 重新订阅 0x40 重同步（**无重试/退避**）。
- payload 内容对设备不透明：设备重组后原样经 IPC 转发给设备算法进程，由算法端解析/解复用。

---

## 3. HTTP 文件传输与 OTA

设备的文件收发与固件升级（OTA）已迁移至 HTTP 服务器（端口 8080），详见 `docs/http-ota-design.md`。

