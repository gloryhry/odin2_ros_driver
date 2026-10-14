# ROS 话题、消息与服务

以下示例使用 `/odin2` 设备命名空间。所有数据话题支持 ROS remap。驱动状态每 0.5 秒发布并 latch，diagnostic_updater 向标准 `/diagnostics` 发布。

## 数据话题

| 默认相对路径 | ROS 类型 | 字段与语义 |
|---|---|---|
| `points_raw` | sensor_msgs/PointCloud2 | x/y/z/intensity FLOAT32，confidence UINT8；offset 0/4/8/12/16，point_step=20，末尾3字节 padding |
| `points_slam` | sensor_msgs/PointCloud2 | x/y/z FLOAT32，rgba UINT32 offset=12，point_step=16；RGBA 值为 AARRGGBB |
| `camera{0,1}/image_raw` | sensor_msgs/Image | bgr8、连续行，step=width×3；header 对应解码帧 |
| `camera{0,1}/encoded` | EncodedFrame | device_timestamp_ns、frame_count、format、width、height、is_keyframe、data；format 1/2/4/8 = NV12/MJPEG/H264/H265 |
| `camera{0,1}/image_raw/compressed` | sensor_msgs/CompressedImage | 只对 MJPEG 直接发送 SDK JPEG 字节 |
| `camera{0,1}/camera_info` | sensor_msgs/CameraInfo | 标准标定，header 与图像一致 |
| `imu/data_raw` | sensor_msgs/Imu | SI 单位，orientation covariance 首项 -1 |
| `odom_lf`、`odom_hf` | nav_msgs/Odometry | 父 odom／子 imu，用户配置标准 covariance |
| `odom_{lf,hf}/sdk` | OdomPacket | 原始 packet 时间／帧号／source_type，每样本 position、orientation、linear/angular velocity、float32[18] sdk_covariance 和 source_type |
| `transforms` | tf2_msgs/TFMessage | SDK 父子 frame 均加设备前缀；可选额外全局 `/tf` 广播 |
| `telemetry` | Telemetry | 全部 SDK 字段：uptime_s、cpu_pct、mem_used_kb、mem_total_kb、mem_use_rate、load_1/5/15，加原始时间和帧号 |
| `status` | DeviceStatus | name/state/SN/IP/last_error/reconnects/clock_resets、covariance 配置标记、PTP 可用标记及状态、逐通道统计 |
| `file_transfer/status` | TransferStatus | job_id、QUEUED/RUNNING/SUCCEEDED/FAILED、progress（0～100）、Result |

EncodedFrame 的 width/height 是 SDK 回调值，未指定自动模式时编码帧可能是 0；解码 Image 始终使用码流解析尺寸。is_keyframe 重新扫描实际 IDR/HEVC 随机接入 NAL，单独的 SPS 不视为恢复帧。所有自定义消息以具名字段承载数据；枚举常量可从生成的 ROS 类型读取。

`ChannelStatus` 区分 received、published（发布过输出的 SDK 包数）、decoded_frames（软件解码帧数）、sequence_gaps（SDK 帧号缺口）、queue_drops、decode_errors、invalid_packets、out_of_order、queue_depth、queue_bytes、last_receive_age 和 last_publish_latency_ms。`reconnects` 记录成功建立连接的次数，包含首次连接。重复／倒序帧计入 out_of_order。last_receive_age=-1 表示本连接还没收到数据。SDK 没有暴露完整网络丢包统计，sequence_gaps 不等于所有 UDP 丢包。

## 服务约定

每个服务返回 `Result {success, sdk_code, message}`。结果码对齐 SDK：0 OK、1 NOT_INITIALIZED、2 INVALID_ARGUMENT、3 SOCKET_ERROR、4 TIMEOUT、5 STOPPED、6 FIRMWARE_ILLEGAL、7 COMMUNICATION_ERROR、8 UNKNOWN_ERROR。ROS 服务正常完成不代表 Result 成功；控制调用等待实际完成，不因 ROS 超时而谎报动作取消。

| 相对服务 | srv 类型 | 输入／返回 |
|---|---|---|
| `get_device_info` | GetDeviceInfo | 空 → SN/model/SDK 版本/固件版本/network/capabilities |
| `get_channel_config` | GetChannelConfig | channel → effective、subscribed、destination_port、resolution_id、fps_id；transport/format 为 SDK 读回，width/height/fps 为驱动所请求模式（0=自动） |
| `set_stream` | SetStream | 完整 StreamConfig → Result 和当前 effective；保留该通道 YAML 的 topic/frame/queue/解码策略 |
| `get_device_mode`、`set_device_mode` | GetMode／SetMode | 0 standby、1 normal、2 upgrade；设置后读回，失败回滚 |
| `get_sensor_mode`、`set_sensor_mode` | GetMode／SetMode | 0 HDR、1 high_peak、2 low_peak、3 adaptive |
| `get_sync_mode`、`set_sync_mode` | GetMode／SetMode | 0 systick、1 PTP；设置为持久化默认值，需要重启设备生效 |
| `get_filters`、`set_filters` | GetFilters／SetFilters | FilterConfig 的 has_* 选择要修改的组，返回有效值；SDK 部分修改失败时恢复旧值 |
| `get_adaptive`、`set_adaptive` | GetAdaptive／SetAdaptive | cdf_percentile、outdoor_threshold；单位同参数表 |
| `get_ptp_status` | GetPtpStatus | sync_state、servo_state、offset_ns、path_delay_ns、freq_ppb |
| `get_network`、`set_network` | GetNetwork／SetNetwork | NetworkConfig 的 ip_address/gateway/netmask/mac/dhcp/ptp；成功设置后提示 reboot_required |
| `reset_initial_pose` | DeviceCommand | 空 → Result；受控重连以清除旧算法输出 |
| `restore_defaults` | DeviceCommand | 空 → Result 与 reboot_required；驱动进程中的期望 YAML 配置仍会在下次连接应用 |
| `reboot` | DeviceCommand | 空 → Result；随后清队列、断开并自动重连 |
| `start_file_transfer` | StartFileTransfer | direction、file_type、绝对 path、timeout_ms → 立即返回 job_id；传输完成由任务状态确认 |
| `get_transfer_status` | GetTransferStatus | job_id → TransferStatus；保留最近64个任务 |
| `send_algo_data` | SendAlgoData | uint8[] data，长度1～262144字节，返回 SDK 结果 |

通道 ID 在 StreamConfig 中有具名常量：RAW_POINT=0、SLAM_POINT=1、CAMERA0=16、CAMERA1=17、IMU=32、ODOM_LF=48、ODOM_HF=49、TF=50、TELEMETRY=66。非相机服务请求 format 必须为 SDK UNKNOWN=255，width/height/fps 为0。相机 format 对应单一位1/2/4/8，不能把15作为请求格式。

### 常用调用

```bash
rosservice call /odin2/get_device_info '{}'
rosservice call /odin2/get_channel_config '{channel: 16}'
# 先查询当前值；set_stream 是完整通道请求
rosservice call /odin2/set_stream '{config: {channel: 0, enabled: true, transport: 0, width: 0, height: 0, fps: 0, format: 255, publish_rate: 10}}'
rosservice call /odin2/set_stream '{config: {channel: 16, enabled: true, transport: 1, width: 0, height: 0, fps: 0, format: 2, publish_rate: 0}}'
rosservice call /odin2/set_sensor_mode '{mode: 3}'
rosservice call /odin2/set_device_mode '{mode: 0}'
rosservice call /odin2/set_device_mode '{mode: 1}'
rosservice call /odin2/get_filters '{}'
rosservice call /odin2/set_filters '{filters: {has_confidence: true, confidence_threshold: 80, has_adaptive: false, has_range: true, min_depth: 150}}'
rosservice call /odin2/set_adaptive '{cdf_percentile: 7000, outdoor_threshold: 3000}'
rosservice call /odin2/get_ptp_status '{}'
rosservice call /odin2/set_sync_mode '{mode: 1}'
# 此动作将重启真实设备，请在允许中断采集的维护窗口执行
rosservice call /odin2/reboot '{}'
rosservice call /odin2/send_algo_data '{data: [1, 2, 3]}'
```

通道启停／相机模式／限频通过同一个配置控制流程调整，并同步 dynamic_reconfigure。调整时会重连，短时中断所有该设备通道。发布限频使用墙钟，不改变设备采样。滤波／模式命令先查询、执行、读回，失败恢复，恢复失败时设备未就绪。单设备文件任务独占控制通道，其他管理服务返回 busy；数据转换继续运行。

### 网络变更

```bash
rosservice call /odin2/get_network '{}'
rosservice call /odin2/set_network '{network: {ip_address: "192.168.1.11", gateway: "192.168.1.1", netmask: "255.255.255.0", mac: "", dhcp: false, ptp: true}}'
rosservice call /odin2/reboot '{}'
```

成功设置后只持久化网络；显式调用 reboot 后，驱动将当前进程的连接目标切换到新 IP。DHCP 需配置或已发现的 SN，并在 reboot 后通过 SN 重新发现。host_ip 和主机路由必须适合新子网；跨子网变更应同步维护网卡和 YAML。进程重启后仍以 YAML 为准。空 mac 是否可用由固件校验，建议基于 get_network 返回值修改。

### 异步文件任务

```bash
# 0=下载，1=上传；file_type 0地图、1标定、2日志、3固件
rosservice call /odin2/start_file_transfer '{direction: 0, file_type: 1, path: "/output/calibration.bin", timeout_ms: 5000}'
# 使用返回的 job_id
rosservice call /odin2/get_transfer_status '{job_id: 1}'
rostopic echo /odin2/file_transfer/status
# 上传文件必须已存在于容器中；SDK 不支持固件下载
rosservice call /odin2/start_file_transfer '{direction: 1, file_type: 0, path: "/output/map.bin", timeout_ms: 5000}'
```

任务进度是 SDK 进度，SUCCEEDED 才表示成功。SDK 上传接口没有可控 timeout 参数；下载 timeout_ms 传给 SDK。固件上传完成（或 SDK 中途失败）后受控重连。SDK 没有取消 API，故没有取消服务；正常退出可能等待正在进行的 SDK 文件调用，Compose 留出120秒关闭窗口。路径是驱动所在机器／容器文件系统中的绝对路径，下载父目录需已存在。
