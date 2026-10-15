# 参数、数据单位与坐标

启动参数位于节点私有空间 `~devices`，它是非空数组。每项产生一个 `/设备名称` 命名空间。节点可通过 ROS namespace 启动，设备命名空间会跟随节点所在的 namespace。未知设备／通道／滤波参数直接报错；参数配置错误不会被忽略。修改 YAML 后须重启节点；运行时更改只保留在当前进程，不自动写回 YAML。

## 设备参数

| 参数 | 默认 | 约束与含义 |
|---|---|---|
| `name` | `odin2` | 合法 ROS 名称，只含字母、数字、下划线；设备间唯一 |
| `sn` | 空 | 发现过滤；多设备推荐配置真实序列号 |
| `device_ip` | 空 | 设备查询结果中的目标 IPv4 过滤；与 SN／host_ip 同时匹配且须唯一；非空也执行 SDK 发现握手 |
| `host_ip` | 空 | 数据回传网卡 IPv4 和发现结果过滤；指定 device_ip 时为空则按路由确定 |
| `tf_prefix` | `name` | 非空相对前缀，设备间唯一；所有输出 frame 加此前缀 |
| `time_mode` | `mapped` | `mapped`／`receive`／`offset`，详见下文 |
| `time_offset_ns` | 字符串 `"0"` | 有符号 64 位纳秒偏移；**必须加引号**，ROS XMLRPC 整数仅有 32 位 |
| `slam_world` | `false` | SDK SLAM／LF Odom 同步；开启时 SLAM frame 强制为 `odom` |
| `broadcast_tf` | `false` | TF 通道发布局部 `transforms`，另可广播到标准 `/tf` |
| `sensor_mode` | 不设置 | `hdr`、`high_peak`、`low_peak`、`adaptive`；设置后在每次重连恢复 |
| `filters` | 不设置 | 仅支持adaptive参数；无配置时不写设备；范围见下表 |
| `sync_lag` | `10` | SDK 同步容忍帧数，1～1000 |
| `command_timeout_ms` | `1000` | 支持超时的 SDK 命令，1～60000 ms；不扩展 SDK 未提供的超时接口 |
| `worker_threads` | `2` | 每设备转换／解码工作线程 1～32，另有一个控制线程 |
| `cpu_affinity` | `[]` | Linux CPU 编号列表；控制和工作线程使用该集合，SDK 自有线程不受此项控制 |
| `pose_covariance` | 对角线 `1e6` | 标准里程计 36 项行优先数组；所有值有限且对角线非负 |
| `twist_covariance` | 对角线 `1e6` | 与 pose 必须一起配置；占位值触发诊断警告 |
| `streams` | 默认通道表 | 按下列通道名设置 |

| `filters` 子参数 | 类型／范围 | 单位／默认 |
|---|---|---|
| `confidence_threshold` | 不支持 | 当前固件无法运行时设置，出现此键则启动配置校验失败 |
| `min_depth` | 不支持 | 当前固件无法运行时设置，出现此键则启动配置校验失败 |
| `cdf_percentile` | 1～10000 整数 | 0.01%；同组只设置 outdoor 时默认 7000 |
| `outdoor_threshold` | 0～65535 整数 | mm；同组只设置 percentile 时默认 3000 |

```yaml
devices:
  - name: front
    sn: YOUR_SERIAL_NUMBER
    device_ip: 192.168.1.10
    host_ip: 192.168.1.100
    sensor_mode: adaptive
    filters: {cdf_percentile: 7000, outdoor_threshold: 3000}
    time_mode: mapped
    tf_prefix: front
    worker_threads: 2
    streams:
      raw_point: {enabled: true, queue_size: 4, publish_rate: 10}
      imu: {enabled: true}
      camera0:
        enabled: true
        transport: tcp
        format: mjpeg
        # 具体模式必须在设备能力表中；0 表示 SDK 自动选模式
        width: 0
        height: 0
        fps: 0
        camera_info_url: file:///calibration/front_camera0.yaml
```

## 通道默认值

所有通道默认 `publish_rate: 0`，表示不作主机限频；`queue_bytes: 33554432`（32 MiB）。只对已经启用的输出发布数据。关闭的通道仍可能有 ROS publisher 注册，但不发送消息。开启世界 SLAM 时低频里程计作为 SDK 内部依赖订阅，配置关闭仍不发布 ROS Odom。

| 通道名 | SDK ID | enabled | topic | frame_id | transport | queue_size |
|---|---:|---|---|---|---|---:|
| `raw_point` | 0 | true | `points_raw` | `lidar` | udp | 4 |
| `slam_point` | 1 | false | `points_slam` | `imu` | udp | 4 |
| `camera0` | 16 | false | `camera0/image_raw` | `camera0` | tcp | 4 |
| `camera1` | 17 | false | `camera1/image_raw` | `camera1` | tcp | 4 |
| `imu` | 32 | true | `imu/data_raw` | `imu` | udp | 2000 |
| `odom_lf` | 48 | false | `odom_lf` | `odom` | udp | 256 |
| `odom_hf` | 49 | false | `odom_hf` | `odom` | udp | 256 |
| `tf` | 50 | false | `transforms` | `odom` | udp | 256 |
| `telemetry` | 66 | true | `telemetry` | `base_link` | tcp | 16 |

通用通道参数：

| 参数 | 类型／约束 | 行为 |
|---|---|---|
| `enabled` | bool | 开启 SDK 订阅与 ROS 输出 |
| `topic` | 非空相对 ROS topic | 不允许绝对路径；同设备主话题不能重复，支持 launch remap |
| `frame_id` | 非空字符串，无空白 | 加设备 TF 前缀；TF 消息使用 SDK 自带父子名称 |
| `transport` | `udp`／`tcp` | 遥测必须 TCP |
| `publish_rate` | 有限非负数 | 以墙钟限制ROS发布Hz；相机仅限制Image/CameraInfo与MJPEG CompressedImage，编码透传不受限制；视频仍解码每帧 |
| `queue_size` | 1～1000000 | 待处理 SDK 包数量限制，不等于点数／字节数 |
| `queue_bytes` | 1～1073741824 | 待处理包估计内存上限；超限丢旧包，单包超限丢该包；视频积压时清除待处理参考链，等待恢复帧 |

相机额外参数：

| 参数 | 默认 | 约束 |
|---|---|---|
| `format` | `mjpeg` | `mjpeg`／`h264`／`h265`／`nv12`，单一格式；能力表中对应位必须存在 |
| `width`、`height` | `0`、`0` | 0～65535；同时为 0 或同时非零，非零必须精确匹配能力模式 |
| `fps` | `0` | 0～65535，非零时必须设置宽高并匹配能力表 |
| `publish_decoded` | `true` | 自动优先硬件解码为 bgr8，无可用硬解时回退 CPU，并发布匹配尺寸的 CameraInfo |
| `publish_encoded` | `true` | 发布自定义 EncodedFrame；不把 H264/H265 标成 JPEG |
| `camera_info_url` | 空 | camera_info_manager 的标准 ROS 标定 YAML URL |

NV12 要求显式偶数尺寸和精确 `width × height × 3/2` 字节。解码额外限制 8192×8192、编码包最大 64 MiB。MJPEG 自动发布 JPEG `CompressedImage`，无重新编码；此输出随相机 enabled／限频控制。相机 topic 的父路径决定 encoded 和 camera_info 路径；例如 `left/image_raw` 派生 `left/encoded`、`left/camera_info` 和 `left/image_raw/compressed`。

非相机通道拒绝 `format` 或非零宽高／设备 fps，SDK 没有对应采集控制 API。`publish_rate` 是主机限频。队列总量还应考虑正在处理的包、ROS publisher 队列、SDK 内部缓冲和解码帧，这些不计入 `queue_bytes`。

## 单位、坐标与标定

原始及 SLAM 回调坐标仍为毫米，转换一次 `× 0.001`。SDK 已对 Y/Z 减去 30000，驱动不重复偏移、不交换轴。原始 x≤0 或非有限点设 NaN；SLAM 非有限或全零坐标设 NaN，世界坐标负 x 保留。输出为非组织化 PointCloud2，`height=1`，空数据 `width=0`。

`slam_world=false` 关闭 SDK 同步，SLAM 使用 IMU 坐标。`true` 开启同步并强制订阅 LF Odom，世界 frame 为 `<prefix>/odom`；它不是全局地图 frame。SDK 对缺失点的补零可能经过偏移／世界变换，回调没有逐点有效标记，驱动无法识别所有失效 SLAM 补点。

标准 Odom 默认父 frame `<prefix>/odom`、子 frame `<prefix>/imu`；四元数 SDK 顺序为 x,y,z,w，标准输出校验并归一化，自定义消息保留原值。SDK Odom/TF 位置是米，IMU 为 rad/s 与 m/s²，不缩放。18 项 SDK covariance 没有明确到 ROS 6×6 的映射，故原样保存在 `sdk_covariance`。IMU 无姿态估计，`orientation_covariance[0]=-1`。

CameraInfo 仅接受标准 ROS YAML，`config/camera_info_example.yaml` 提供未标定格式模板（K=0），请用实际标定结果替换；没有标定时 K=0，并填当前图像尺寸。尺寸不匹配时抑制 CameraInfo 并告警，图像仍输出。SDK 下载的标定 bin 是文件交付，不猜测内部格式。相机 frame 应按实际设备定义；有明确外参时，用户可通过 `static_transform_publisher` 发布到 optical frame，驱动不凭空生成旋转关系。

## 时间映射

- `mapped`：首个有效包建立设备 ns 与 ROS 时间锚点，后续全部通道共用偏移；IMU 使用 SDK 已重建的逐样本时间，不重复推算采样间隔。
- `receive`：采用回调接收 ROS 时间，保留设备时间的乱序／回退检查。
- `offset`：ROS ns = device ns + 明确的有符号 `time_offset_ns`。仅在已确认设备时基时使用。

设备重连、同通道设备时间回退超过 1 秒、ROS 时间回退或相对于墙钟前跳超过 5 秒时重建锚点、清队列并重置解码器。小幅设备乱序丢弃。`/use_sim_time` 下没有有效 `/clock` 时不发布数据，计入 invalid_packets。PTP locked 只表示 SDK 同步状态，不证明设备 timestamp 是 UNIX 时间，驱动不自动切换 offset 模式。

## 运行时动态配置

每台设备 `/name/set_parameters` 为独立 dynamic_reconfigure server；可调九个 `*_enabled` 和九个 `*_publish_rate`（0～2000 Hz）。`apply_adaptive=true` 显式应用cdf_percentile/outdoor_threshold，确认成功后复位为false。数字控件表示候选配置，实际值通过get_adaptive查询。旧apply_filters、confidence_threshold、min_depth控件已弃用；apply_filters=true或修改不支持字段会整批拒绝，包括同次请求中的通道变更。

通道调整通过受控重连完成；adaptive单独更新不重连。adaptive先查询、设置、严格读回，失败恢复调用前实际值。无法确认恢复时保留独立恢复目标、进入retrying并停止旧数据输出；重连优先恢复该目标。正常重连恢复已接受配置，无配置则不主动写参数。YAML的filters空对象仍为无效配置。

硬件探测、后端优先级、30 秒重试与容器设备传入见 [硬件解码](hardware-decoding.md)。
