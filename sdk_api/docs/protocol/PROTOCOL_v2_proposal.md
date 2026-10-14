# Odin 协议演进提案 / Odin Protocol Evolution Proposal

> **状态 / Status**: Draft — 未实现，未合入 `PROTOCOL.md`
> **Status**: Draft — not implemented, NOT merged into `PROTOCOL.md`
>
> 本文档收录对当前 Odin 协议（控制 v1 / 数据 v2）的**破坏性变更提案**。任何条目在通过 RFC 评审、获得双端（设备固件 + Host SDK）原型验证之前，**不得视为协议事实**。
>
> This document collects **breaking-change proposals** for the current Odin protocol (control v1 / data v2). No item here is normative until it passes RFC review and is validated by paired prototypes (device firmware + host SDK).

[TOC]

## 0. 兼容性策略 / Compatibility Strategy

每条提案必须给出明确的协商/降级路径，避免一次性破坏现网设备：
Every proposal MUST define an explicit negotiation/downgrade path so that field devices are not broken in one step:

1. **`0x02 VersionQuery`** ACK 中扩展 `supported_protocol_versions: uint8_t[]`，host 选取最低公共版本。
2. 新增 TLV / 字段时遵循"未知 type 跳过、未知字段忽略"原则。
3. 涉及帧头/字节布局的破坏性改动只在大版本号 (`version` 字段) 跨越时启用。
4. 至少维护**一个大版本周期**的旧客户端兼容性。

## 1. 控制平面提案 / Control-Plane Proposals

### P-CTRL-01：扩展 `ret_code` + 错误描述字段

**现状 / Current**：`ret_code` 只有 `0/1/2`，无错误描述，调试时无法区分"参数越界"与"通道不在能力表"。

**提案 / Proposal**：协议 `version >= 2` 时，所有 ACK data 段使用扩展布局：

```
[ret_code: uint8_t]
[err_msg_len: uint8_t]              ← 0 表示无附加描述
[err_msg: char[err_msg_len] (UTF-8, no NUL terminator)]
[command-specific fields...]
```

| ret_code | 名称 / Name | 说明 / Description |
| --- | --- | --- |
| `0` | OK | 成功 |
| `1` | Failure | 一般失败 |
| `2` | InvalidArgument | 参数错误 |
| `3` | UnsupportedVersion | 协议版本不被支持 |
| `4` | UnsupportedChannel | 通道不支持或未在能力表 |
| `5` | OutOfRange | 索引/取值越界 |
| `6` | Busy | 设备已被其他 host 占用 |
| `7` | InternalError | 内部错误 |
| `8..127` | Reserved | 协议保留 |
| `128..255` | VendorDefined | 厂商扩展 |

**风险 / Risks**：所有 ACK 解析需要改；旧 host 解析新 ACK 会越界。

**降级路径 / Downgrade**：仅在 v2 协商成功后启用；v1 会话仍按 3 个返回码且无 `err_msg`。

---

### P-CTRL-02：`cmd_type` bit 重组（折叠 `send_type`）

**现状**：帧头 offset 10 = `cmd_type` (REQ/ACK)，offset 11 = `send_type` (Host/Device)。两个字节高度冗余。

**提案**：v2 起，`cmd_type` 重新定义为 bit 字段：

| Bit | 含义 |
| --- | --- |
| bit0 | 0 = REQ, 1 = ACK |
| bit7 | 0 = Host→Device, 1 = Device→Host |
| bit1..bit6 | 保留 0 |

offset 11 改为 `reserved` (1 字节，必须 0)，**保持帧头总长 18B 不变**以便二进制兼容旧 parser 的偏移读取。

**风险**：现网若按 `send_type == 1` 数值比较而非按字段位置取值，需要源码级适配；旧设备读到 `cmd_type = 0x81` 可能误判为未知类型。

**降级路径**：v1 沿用旧布局；v2 协商成功后切换。

---

### P-CTRL-03：`0x06 ChannelConfig` 新增 TLV

**现状**：仅支持 5 个 TLV (`0x01..0x05`)；分片大小硬编码 1454，UDP 数据面 `crc32` 默认 0。

**提案**：新增两个 TLV，向前兼容（未知 type 仍按原规则跳过）：

| type | length | value | 描述 |
| --- | --- | --- | --- |
| `0x06` | 2 | uint16_t (LE) | `max_chunk_payload`：上位机期望的单分片最大载荷（不含数据帧头），设备取 `min(host_hint, link_capability)`；`0` = 设备自决 |
| `0x07` | 1 | uint8_t | `enable_crc32`：1 = 启用数据面 `crc32`，0 = 置 0 跳过校验 |

ACK 在 v2 布局基础上追加：

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| negotiated_chunk_payload | uint16_t (LE) | 协商后的实际值，0 = 默认 |

**风险**：v1 host 不会发这两个 TLV，设备保持当前行为；v2 host 与 v1 设备组合时设备会跳过未知 TLV，host 需检查 ACK 中是否带 `negotiated_chunk_payload`，否则按 v1 默认处理。

---

### P-CTRL-04：`0x08 TimeSync` 时间同步指令

**动机**：当前数据帧 `timestamp` 基于设备 monotonic 时基；设备掉电重启后 timestamp 跳变，host 无法识别。

**提案**：新增 PTP-lite 指令。

**REQ** (Host → Device, TCP)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| t1_host_ns | uint64_t | Host 发送时刻 (host monotonic, ns) |

**ACK** (Device → Host, v2 布局)

| 名称 | 类型 | 描述 |
| --- | --- | --- |
| ret_code | uint8_t | |
| err_msg_len + err_msg | | 见 P-CTRL-01 |
| t1_host_ns | uint64_t | 回显 |
| t2_device_ns | uint64_t | 设备接收时刻 (device monotonic) |
| t3_device_ns | uint64_t | 设备发送时刻 (device monotonic) |
| epoch_id | uint32_t | 设备时基 epoch；掉电重启后 +1 |

**降级**：v1 设备返回 `ret_code = 1`；host 据此回退到不做时间同步。

---

### P-CTRL-05：心跳超时窗口默认值调整

**现状**：默认 `heartbeat_interval_ms = 1000`，超时 = 3000 ms。Windows 用户态调度 + 防火墙抖动可能误触发全重置。

**提案**：建议默认值改为 `heartbeat_interval_ms = 1000` 但超时倍数从 3× 改为 5×（即 5000 ms）；倍数由设备能力表新字段 `heartbeat_timeout_factor` 报告，host 端不再硬编码 3。

**风险**：与现网设备固件默认值不一致——若设备端写死 3×，host 按 5× 发会被误重置。**只能在设备固件先升级后再放宽 host 行为**。

---

### P-CTRL-06：用户自定义不透明消息 / User-Defined Opaque Message

**现状**：模组上可能运行厂商/应用算法，但协议没有"应用层透传"通道。现网做法是每加一种算法就新申请一个 `cmd_id`，污染命名空间，并强制 SDK 头文件跟随发版。

**提案**：保留单一 `cmd_id = 0xE0 (kUserMessage)` 作为不透明应用通道，SDK 不解析载荷，多路复用通过 payload 头 2B `topic` 完成。

**REQ payload** (Host → Device):

| Offset | 名称 | 类型 | 描述 |
| --- | --- | --- | --- |
| 0 | topic | uint16_t (LE) | 应用自定义主题 ID |
| 2.. | payload | bytes[] | 任意字节，SDK 不解析 |

**ACK payload** (v1 布局；v2 在 `ret_code` 后按 P-CTRL-01 插 `err_msg_len + err_msg`):

| Offset | 名称 | 类型 | 描述 |
| --- | --- | --- | --- |
| 0 | ret_code | uint8_t | 0 = OK，非 0 由应用语义定义 |
| 1.. | response | bytes[] | 算法返回的不透明字节 |

**长度字段 / Length encoding**：本提案**不在 payload 内重复**长度字段，长度由外层 TLV 帧头的 16-bit `length` 给出（见 `PROTOCOL.md` 控制帧定义）。接收方按下式反算：

```
REQ : payload_len  = TLV.length - 2   // 减去 topic (uint16)
ACK : response_len = TLV.length - 1   // 减去 ret_code (uint8)；
                                      //   v2 还需再减去 err_msg_len(2B) + err_msg
```

由此 `payload` / `response` 都是变长但**自定界**：长度由 TLV 头唯一确定，topic / ret_code 是定长前缀，剩余字节即为不透明数据。单包上限受 TLV 16-bit `length` 限制（≈ 64 KB，扣帧头约 60 KB 可用）。

The length of the variable-length `payload` / `response` is **not** re-encoded inside the payload; it is taken from the outer TLV header's 16-bit `length` field (see `PROTOCOL.md`). Receivers compute `payload_len = TLV.length - 2` (REQ, after `topic`) and `response_len = TLV.length - 1` (ACK v1, after `ret_code`; v2 additionally subtracts `err_msg_len + err_msg`). Maximum single-frame size is bounded by the TLV 16-bit length (~60 KB usable).

**Topic 命名空间**：`0x0000` 保留；`0x0001..0x7FFF` 项目内分配；`0x8000..0xFFFF` 厂商扩展。未注册 topic 设备必须返回非零 `ret_code`。

**SDK 透传契约 / SDK pass-through contract**：

- SDK 仅承担传输职责。`SendUserMessage` 的返回值 (`OdinResult`) **只反映传输结果**（ACK 是否到达、是否超时 / 帧错），**不得**将设备应用层 `ret_code` 折叠成 SDK 错误码。
- ACK payload **整体透传**给调用方（含首字节 `ret_code`），调用方按下列约定解析：
  - `response[0]` = 设备 `ret_code`（0 = OK，非 0 由应用语义定义）
  - `response[1..]` = 应用不透明响应字节
- 理由：`ret_code` 语义由两端应用约定，SDK 没有依据将 `ret_code == N` 映射到任一 `OdinResult` 值；折叠会丢失信息且与本提案"SDK 不解析载荷"原则冲突。

The SDK is transport-only. `SendUserMessage`'s `OdinResult` reflects **only** transport outcome; the device-side `ret_code` is **never** collapsed into it. The full ACK payload (including the leading `ret_code` byte) is delivered verbatim to the caller, who parses `response[0]` as `ret_code` and `response[1..]` as the opaque application response. This preserves the proposal's "SDK does not interpret payload" invariant.

**约束**：

- 单包 ≤ ~60 KB（TLV 16-bit length）；大数据走 `Upload/Download` 或数据通道。
- 不承载 device→host 持续流。
- 不复用已有强类型接口能表达的功能（Reboot / Upload / StartStream / SetSensorMode 等）。

**风险**：设备端 topic 路由表治理需要外部 `user_topic_registry.md`；SDK 层抓包只见 `topic + length`，详细语义在两端。被滥用为业务总线会抹掉本次重构的强类型边界。

**兼容性 / 降级**：纯增量（新增一个 cmd_id），不改既有帧布局；可与 v1 控制面并存。设备未实现 0xE0 时返回 `ret_code = 1`，host 据此回退。

---

## 2. 数据平面提案 / Data-Plane Proposals

### P-DATA-01：UDP 通道强制 `crc32`

**现状**：数据帧头 `crc32` 默认 0，UDP 完全裸跑，链路层 CRC 后任何中间层 (路由器、虚拟交换) 出错都无法检测。

**提案**：通过 `P-CTRL-03` 新增的 TLV `0x07 enable_crc32 = 1` 显式启用；建议数据 v3 起对 UDP 通道**强制 `crc32 != 0`**，否则 host 应丢弃该分片。

**降级**：TCP / USB Bulk 由介质保障可靠性，可继续允许 0。

---

### P-DATA-02：`interval_ns` 字段在非 IMU 通道复用为 `epoch_id`

**现状**：非 IMU 通道 `interval_ns` 恒为 0，4 字节浪费。

**提案**：v3 起，非 IMU 通道该字段复用为 `epoch_id` (低 32 bit)，与 `0x08 TimeSync` ACK 中的 epoch_id 对齐。host 据此识别设备重启导致的 timestamp 跳变。

**风险**：v2 host 解析 v3 流会把 epoch_id 误读为 0；需通过 data-plane `version = 3` 区分。

---

### P-DATA-03：点云 `xyz` 重新编码

**现状**：`x` 用 uint16_t (mm)，`y / z` 用 uint16_t 但隐式 `-30000` 偏移；不对称、易错、对弱类型 SDK 不友好。

**提案**：数据 v3 起：

- `xyz` 全部改为 `int16_t` (mm)
- `0x03 GetSensorCapability` 的 JSON 里每通道增加 `geometry: { scale: 1.0, offset: [0, 0, 0], unit: "mm" }`
- SDK 按能力表声明的 scale/offset 反算，不再硬编码 `-30000`

**风险**：需要 SDK 同步升级；旧 host 收到 v3 流会按 uint16 读取得到错误几何值。

---

### P-DATA-04：Odom 字段单位/标度规范化

**现状**：`linear_velocity / angular_velocity / cov` 单位标注为 "-"；`orient` 标 `×10⁻⁶` 但没说反算到什么；249B 奇数字节。

**提案**：在协议正文中明确（已在主文档 §2.3.4 修订草案中描述）：

| 字段 | 单位/标度 | 反算 |
| --- | --- | --- |
| pos | μm | float (m) = `value × 1e-6` |
| orient | 1e-6 | float = `value × 1e-6` |
| linear_velocity | μm/s | float (m/s) = `value × 1e-6` |
| angular_velocity | 1e-6 rad/s | float (rad/s) = `value × 1e-6` |
| cov | 1e-9 | float = `value × 1e-9` |
| type | enum | 0=Invalid, 1=Visual, 2=Inertial, 3=Fused |

并建议 v3 起 pad 到 256B 以利数组化。

**风险**：仅文档约定与现状不一致，需查清现有固件是否真按上述标度发送；若不一致，需先固件矫正。

---

### P-DATA-05：`flags` bit1 = `samples_self_contained`

**现状**：协议解析依赖 `data_type == 0x20` 判断 IMU 自包含分片，新增通道时需要修改解析逻辑。

**提案**：在 `flags` bit1 中显式编码该语义（IMU 默认 1，其他通道默认 0），解析路径不再依赖 `data_type` 分支。

**风险**：v2 host 不读 bit1，但行为与现状等价（按 `data_type` 隐式判断）；v3 SDK 要切到 bit1 解析。

---

## 3. 传输绑定提案 / Transport Binding Proposals

### P-XPRT-01：USB Bulk 绑定

**现状**：协议层硬编码 IP/端口；无 USB 直连方案。

**提案**：补完 `transport_mode = 2 (USB Bulk)` 的具体绑定：

- **设备发现**：USB 描述符 (VID/PID + Vendor-specific interface) 枚举，无 DeviceQuery
- **心跳**：USB 总线连接由内核维护，不需要应用层心跳；driver 用 `claim interface` 实现独占
- **命令通道**：专用 Bulk IN/OUT endpoint pair，建议 `wMaxPacketSize ≥ 512`
- **数据通道**：每个 `data_type` 映射独立 IN endpoint；`ChannelConfig` 中 `dst_port = 0`
- **MTU**：`negotiated_chunk_payload` 默认 16384，由 driver 单 URB 上限决定
- **跨平台 driver**：基于 libusb / WinUSB；Windows 提供 INF 安装包

**待办 / TBD**：

- VID/PID 申请与 USB Vendor 接口编号
- endpoint 数量是否随通道动态分配，还是固定 N 个 IN endpoint 通过 `data_type` 多路复用单 endpoint
- 大包（图像）是否需要分段或 isochronous 端点

---

### P-XPRT-02：共享内存绑定（同机部署）

**动机**：同机 ROS 部署下，UDP 回环依旧有 syscall 开销与拷贝。

**提案**：预留 `transport_mode = 3 (Shared Memory)`，由设备/驱动协商共享内存名 + 信号量；具体方案待 RFC。

---

## 4. 安全提案 / Security Proposals

### P-SEC-01：TLS 叠加

在 TCP 60001 (命令)、60002 (心跳)、数据 TCP 通道之上叠加 TLS 1.3；端口由部署方约定。**不改协议字段**，仅作为传输绑定层的可选叠加。

### P-SEC-02：DeviceQuery 挑战-应答

在 `0x01 DeviceQuery` 中加入 32 字节随机 challenge，设备签名后返回，防止局域网中任意主机抢占设备。需要预共享密钥或证书机制。

### P-SEC-03：序列号防重放

Host SDK 应做 `seq_num` (控制) 与 `frame_seq` (数据) 单调性检查，丢弃倒退包。**SDK 实现要求，不改协议字段**。

---

## 5. 评审清单 / Review Checklist

每条提案进入主文档前必须满足：
Before any item is merged into the main spec, it MUST satisfy:

- [ ] 设备固件原型实现并通过 CI 自动化测试
- [ ] Host SDK (Linux + Windows) 双端原型对接验证
- [ ] 与现网设备的兼容性矩阵 (v1↔v1 / v1↔v2 / v2↔v1 / v2↔v2)
- [ ] 性能影响测量（CRC、TLS、USB 吞吐）
- [ ] 文档双语化 + 抓包样例
- [ ] 至少一个客户/QA 测试场景验收通过
