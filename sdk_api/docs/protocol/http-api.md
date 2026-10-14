# Odin2 HTTP API 接口文档

本文档面向上位机 SDK 开发人员，描述 Odin2 设备 HTTP 服务器提供的所有 REST API 和 WebSocket 接口。

---

## 概述

| 项目 | 说明 |
|------|------|
| 协议 | HTTP/1.1 |
| 默认端口 | `8080` |
| 数据格式 | JSON（`Content-Type: application/json`） |
| 认证方式 | Cookie-based（`access_token`），详见 [认证](#认证) |
| WebSocket | `ws://<device_ip>:8080/websocket` |

### 权限等级

| 等级 | 角色 | 说明 |
|------|------|------|
| 0 | Guest | 公开接口，无需认证 |
| 3 | User | 默认权限（未登录即具备），可访问大部分功能 |
| 7 | Admin | 需登录，可执行 OTA、网络配置、重启等操作 |

> 未登录时默认权限为 User(3)。Admin 权限需通过 `/api/login` 获取。

---

## 认证

### POST `/api/login`

登录并获取 Admin 权限。

**权限**: 公开

**请求**: HTTP Basic Authentication（`Authorization: Basic base64(user:pass)`）

**响应**:

成功 (200):
```json
{
  "user": "admin",
  "level": 7
}
```

响应头包含 `Set-Cookie: access_token=<token>; Path=/; HttpOnly; SameSite=Lax; Max-Age=86400`

失败 (401):
```json
{
  "error": "Invalid credentials"
}
```

**示例**:
```bash
curl -X POST http://<ip>:8080/api/login \
  -H "Authorization: Basic $(echo -n 'admin:admin123' | base64)"
```

---

### POST `/api/logout`

注销当前会话，清除认证 Cookie。

**权限**: 公开

**响应** (200):
```json
{
  "ok": true
}
```

---

## 设备状态

### GET `/api/health`

健康检查，用于确认设备 HTTP 服务是否在线。

**权限**: 公开 (Level 0)

**响应** (200):
```json
{
  "ok": true
}
```

---

### GET `/api/heartbeat`

心跳接口。

**权限**: 公开 (Level 0)

**响应** (200):
```json
{
  "version": 0
}
```

---

### GET `/api/version`

获取设备版本及监控信息。与 `/api/monitor` 返回相同数据。

**权限**: 公开 (Level 0)

**响应**: 同 `/api/monitor`

---

### GET `/api/monitor`

获取设备实时监控数据，包含设备身份信息、系统资源和传感器状态。

**权限**: 公开 (Level 0)

**响应** (200):
```json
{
  "firmware_version": "2.0.1",
  "serial": "ODIN2_N0000001",
  "module_name": "Odin",
  "thermal_zones": [
    { "name": "cpu-thermal", "temp": 52 },
    { "name": "gpu-thermal", "temp": 48 }
  ],
  "cpu": {
    "total": 22,
    "per_core": [25, 18, 30, 15]
  },
  "memory": {
    "total_kb": 2048000,
    "used_kb": 512000,
    "use_rate": 25
  },
  "load_avg": {
    "1min": 2.05,
    "5min": 1.82,
    "15min": 1.45
  },
  "sensors": [
    { "name": "raw", "configured_fps": 10, "fps": 10 },
    { "name": "slam", "configured_fps": 10, "fps": 9 },
    { "name": "imu", "configured_fps": 400, "fps": 398 }
  ],
  "sensor_thermal": [
    { "name": "lidar", "temp": 45 }
  ]
}
```

**字段说明**:

| 字段 | 类型 | 说明 |
|------|------|------|
| `firmware_version` | string | 固件版本号 |
| `serial` | string | 设备序列号 (SN) |
| `module_name` | string | 设备模块名称 |
| `thermal_zones` | array | SoC 温度区，`temp` 单位为 ℃（整数） |
| `cpu.total` | int | CPU 总使用率 (0-100%) |
| `cpu.per_core` | int[] | 每核 CPU 使用率 |
| `memory.total_kb` | int | 总内存 (KB) |
| `memory.used_kb` | int | 已用内存 (KB) |
| `memory.use_rate` | int | 内存使用率 (0-100%) |
| `load_avg` | object | 系统负载平均值 (1/5/15 分钟) |
| `sensors` | array | 传感器帧率，`configured_fps` 为配置值，`fps` 为实际值 |
| `sensor_thermal` | array | 传感器温度，`temp` 单位为 ℃ |

**错误** (500):
```json
{
  "error": "No monitor data available"
}
```

> 设备启动后首次采集数据前（约 1 秒内），可能返回 500。

---

## 网络配置

### GET `/api/network`

获取当前网络配置。

**权限**: User (Level 3)

**响应** (200):
```json
{
  "ip_address": "192.168.1.100",
  "gateway": "192.168.1.1",
  "netmask": "255.255.255.0",
  "dhcp": false
}
```

---

### POST `/api/network`

修改网络配置。**仅保存到配置文件，重启后生效**。

**权限**: User (Level 3)

**请求**:
```json
{
  "ip_address": "192.168.1.200",
  "gateway": "192.168.1.1",
  "netmask": "255.255.255.0",
  "dhcp": false
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ip_address` | string | 是 | 静态 IP 地址 |
| `gateway` | string | 是 | 网关地址 |
| `netmask` | string | 是 | 子网掩码 |
| `dhcp` | bool | 是 | 是否启用 DHCP（启用时忽略静态 IP 配置） |

**响应** (200):
```json
{
  "ok": true,
  "reboot_required": true
}
```

**错误** (500):
```json
{
  "error": "Failed to save config"
}
```

---

## OTA 固件升级

### OTA 状态机

```
IDLE → UPLOADING → VERIFYING → INSTALLING_MCU → INSTALLING_SOC → REBOOTING → POST_VERIFY → DONE
                                                                                           ↘ FAILED
```

### GET `/api/ota/status`

获取当前 OTA 状态。

**权限**: 公开 (Level 0)

**响应** (200):
```json
{
  "ota": {
    "state": "IDLE",
    "progress": 0,
    "message": "",
    "firmware_file": "",
    "expected_fw_version": "",
    "expected_mcu_version": "",
    "mcu_result": "",
    "error": ""
  }
}
```

**字段说明**:

| 字段 | 类型 | 说明 |
|------|------|------|
| `state` | string | 当前状态: `IDLE`, `UPLOADING`, `VERIFYING`, `INSTALLING_MCU`, `INSTALLING_SOC`, `REBOOTING`, `POST_VERIFY`, `DONE`, `FAILED` |
| `progress` | int | 当前阶段进度 (0-100) |
| `message` | string | 人类可读的状态描述 |
| `firmware_file` | string | 上传的固件文件名 |
| `expected_fw_version` | string | 预期固件版本 |
| `expected_mcu_version` | string | 预期 MCU 版本 |
| `mcu_result` | string | MCU OTA 结果: `success`, `failed`, `skipped`, 或空 |
| `error` | string | 错误信息（`FAILED` 状态时有值） |

---

### POST `/api/ota/upload`

上传 OTA 固件包。支持整包上传和分块上传（Content-Range）。

**权限**: User (Level 3)

**请求头**:

| Header | 必填 | 说明 |
|--------|------|------|
| `Content-Length` | 是 | 本次请求 body 大小 |
| `X-Filename` | 否 | 文件名（默认 `firmware.bin`） |
| `Content-Range` | 否 | 分块上传时使用，格式: `bytes <start>-<end>/<total>` |

**请求体**: 二进制固件数据 (raw body)

**响应** (200):
```json
{
  "ok": true,
  "size": 3145728
}
```

**错误** (409) — OTA 正在进行中:
```json
{
  "error": "OTA in progress (state=INSTALLING_MCU)"
}
```

**整包上传示例**:
```bash
curl -X POST http://<ip>:8080/api/ota/upload \
  -H "X-Filename: firmware_v2.0.1.bin" \
  -H "Content-Type: application/octet-stream" \
  --data-binary @firmware_v2.0.1.bin
```

**分块上传示例** (1MB 分块):
```bash
# 第 1 块: bytes 0-1048575
curl -X POST http://<ip>:8080/api/ota/upload \
  -H "X-Filename: firmware.bin" \
  -H "Content-Range: bytes 0-1048575/3145728" \
  --data-binary @chunk0.bin

# 第 2 块: bytes 1048576-2097151
curl -X POST http://<ip>:8080/api/ota/upload \
  -H "Content-Range: bytes 1048576-2097151/3145728" \
  --data-binary @chunk1.bin

# 第 3 块: bytes 2097152-3145727
curl -X POST http://<ip>:8080/api/ota/upload \
  -H "Content-Range: bytes 2097152-3145727/3145728" \
  --data-binary @chunk2.bin
```

> **注意**: UPLOADING 状态下 1000ms 内无新数据将自动超时回退到 FAILED。

---

### POST `/api/ota/trigger`

触发 OTA 安装流程。仅在固件上传完成（UPLOADING 100%）后调用。

**权限**: User (Level 3)

**响应** (200):
```json
{
  "ok": true
}
```

**错误** (400):
```json
{
  "error": "OTA trigger failed"
}
```

---

### POST `/api/ota/reset`

重置 OTA 状态到 IDLE。仅允许在 `IDLE`、`UPLOADING`、`DONE`、`FAILED` 状态下调用。

**权限**: User (Level 3)

**响应** (200):
```json
{
  "ok": true
}
```

**错误** (409) — 安装过程中不可重置:
```json
{
  "error": "Reset rejected: OTA in progress"
}
```

---

## 文件操作

### GET `/api/file/download/calib`

下载标定文件。

**权限**: User (Level 3)

**响应**: 二进制文件流

---

### GET `/api/file/download/map`

下载地图文件。

**权限**: User (Level 3)

**响应**: 二进制文件流

---

### GET `/api/file/download/log`

下载系统日志。

**权限**: User (Level 3)

**响应**: 二进制文件流

---

### POST `/api/file/upload/calib`

上传标定文件。

**权限**: User (Level 3)

**请求**: 二进制文件（raw body），建议设置 `X-Filename` header。

**响应** (200):
```json
{
  "ok": true
}
```

---

## 系统管理

### POST `/api/reboot`

重启设备。

**权限**: User (Level 3)

**响应** (200):
```json
{
  "ok": true
}
```

> 响应返回后设备将立即开始重启流程。

---

## WebSocket 实时推送

### 连接

```
ws://<device_ip>:8080/websocket
```

**权限**: 公开

### 消息格式

设备每秒推送一次状态数据，格式如下：

```json
{
  "status": {
    "firmware_version": "2.0.1",
    "serial": "ODIN2_N0000001",
    "module_name": "Odin",
    "thermal_zones": [...],
    "cpu": { "total": 22, "per_core": [25, 18, 30, 15] },
    "memory": { "total_kb": 2048000, "used_kb": 512000, "use_rate": 25 },
    "load_avg": { "1min": 2.05, "5min": 1.82, "15min": 1.45 },
    "sensors": [...],
    "sensor_thermal": [...]
  }
}
```

> `status` 字段内容与 `GET /api/monitor` 响应格式完全相同。

OTA 过程中还会推送 OTA 状态更新：

```json
{
  "ota": {
    "state": "INSTALLING_MCU",
    "progress": 45,
    "message": "Installing MCU firmware...",
    "firmware_file": "firmware_v2.0.1.bin",
    "expected_fw_version": "2.0.1",
    "expected_mcu_version": "1.5.3",
    "mcu_result": "",
    "error": ""
  }
}
```

### SDK 集成建议

```python
import websocket
import json

def on_message(ws, message):
    data = json.loads(message)
    if "status" in data:
        status = data["status"]
        print(f"CPU: {status['cpu']['total']}%")
        print(f"Load: {status['load_avg']['5min']}")
        for s in status["sensors"]:
            print(f"  {s['name']}: {s['fps']} fps")
    if "ota" in data:
        ota = data["ota"]
        print(f"OTA: {ota['state']} ({ota['progress']}%)")

ws = websocket.WebSocketApp("ws://192.168.1.100:8080/websocket",
                            on_message=on_message)
ws.run_forever()
```

---

## OTA 完整流程示例

```
SDK                                  设备
 |                                    |
 |  POST /api/ota/upload              |
 |  (Content-Range: bytes 0-1M/3M)   |
 |----------------------------------->|  → UPLOADING (progress 0-33%)
 |  {"ok":true,"size":1048576}        |
 |<-----------------------------------|
 |                                    |
 |  POST /api/ota/upload              |
 |  (Content-Range: bytes 1M-2M/3M)  |
 |----------------------------------->|  → UPLOADING (progress 33-66%)
 |                                    |
 |  POST /api/ota/upload              |
 |  (Content-Range: bytes 2M-3M/3M)  |
 |----------------------------------->|  → UPLOADING (progress 100%)
 |                                    |
 |  POST /api/ota/trigger             |
 |----------------------------------->|  → VERIFYING → INSTALLING_MCU
 |  {"ok":true}                       |    → INSTALLING_SOC → REBOOTING
 |                                    |
 |  (设备重启, WebSocket 断开)         |
 |                                    |
 |  GET /api/health (轮询重连)         |
 |----------------------------------->|  → POST_VERIFY → DONE
 |  {"ok":true}                       |
 |                                    |
 |  GET /api/ota/status               |
 |----------------------------------->|
 |  {"ota":{"state":"DONE",...}}      |
 |<-----------------------------------|
 |                                    |
 |  POST /api/ota/reset               |
 |----------------------------------->|  → IDLE
 |  {"ok":true}                       |
```

---

## 错误码汇总

| HTTP 状态码 | 场景 |
|-------------|------|
| 200 | 请求成功 |
| 400 | 请求参数错误或操作失败 |
| 401 | 认证失败（用户名/密码错误） |
| 405 | 不支持的 HTTP 方法 |
| 409 | OTA 状态冲突（正在执行中，不允许操作） |
| 500 | 服务器内部错误 |
