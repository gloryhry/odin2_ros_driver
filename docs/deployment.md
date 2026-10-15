# 部署、发布与性能

## 平台支持边界

| 平台 | 构建／运行选择 | 验证边界 |
|---|---|---|
| x86_64 | Ubuntu20／ROS Noetic 原生或 linux/amd64 镜像 | 容器编译、模拟验收及 NVIDIA 合成码流硬解；真实 Odin2 待验收 |
| ARM64 | Ubuntu20 原生或 linux/arm64 镜像 | 本轮 QEMU 编译／模拟验收；实际 ARM CPU 性能待验证 |
| Orin JetPack5 | 通用或 JetPack 专用 ARM64 镜像，host 网络 | JetPack 专用镜像使用 nvv4l2decoder；实际 Jetson 待验收 |
| Orin JetPack6 | 专用 Ubuntu22 容器内源码构建 ROS Noetic | 匹配 L4T 库与插件；实际 Jetson 待验收 |

同架构发布镜像不启用 `-march=native`、fast-math 或 Orin 专有指令。Release 允许编译器基础自动向量化；可选 LTO。CPU 亲和性只影响驱动工作／控制线程。Orin 默认配置提供较小点云队列和编码透传选项；开启两路视频解码前，先按能力表选择合适分辨率／fps，再测量 CPU 和丢帧。

## 升级兼容性

升级保留现有 topic 和 `.msg/.srv` 定义，但需同步检查以下配置与订阅端：

- 原始 PointCloud2 恢复官方 18 字节布局：`x/y/z` 为 FLOAT32（偏移 0/4/8），`intensity/confidence` 为 UINT8（偏移 12/13），`offset_time` 为 FLOAT32 秒（偏移 14），`row_step=width×18`。将订阅端旧 FLOAT32 intensity 注册及固定 20 字节解析改为新布局；PCL 使用字段映射读取。逐点时间按原始索引每 1536 点递增 `1/328.5` 秒，无效点保留索引，坐标为 NaN。
- 滤波控制仅支持 adaptive。移除 YAML 中的 `confidence_threshold/min_depth`，否则配置校验失败；含 confidence/range 的服务请求整单拒绝，`set_filters` 仅接受 adaptive-only 请求。动态配置改用 `apply_adaptive`，旧 `apply_filters` 被拒绝。驱动不添加主机端滤波，参数详见 [参数说明](parameters.md)。
- 相机 `publish_rate` 限制解码图像及 MJPEG CompressedImage，EncodedFrame 透传不按此值丢包。需要降低编码流带宽时调整设备采集 FPS，并确认新模式受能力表支持；升级前按设备 FPS 重新评估网络及订阅端负载。

## Docker 和 Compose

```bash
# 单架构本机编译（测试自动执行）
docker build -f docker/Dockerfile --build-arg BUILD_JOBS=4 -t odin2-ros-driver:local .
# 多架构，适用于已配置 QEMU 的 Buildx 环境
docker buildx build --platform linux/amd64,linux/arm64 \
  -f docker/Dockerfile --build-arg BUILD_JOBS=2 \
  -t ghcr.io/owner/repository:manual --push .
# 只执行测试并导出 Catkin XML 报告
docker buildx build --platform linux/amd64 --target test-results \
  -f docker/Dockerfile --output type=local,dest=/tmp/odin2-reports .
```

Docker 是 build → test → install → runtime 多阶段，test-results 为独立 scratch 导出阶段。runtime 依赖通过测试的 install，并执行安装空间 smoke：验证 ldd、ROS Python 消息和真实 SDK 节点状态，不要求连接硬件。运行镜像不包含编译器和 mock 节点。

```bash
mkdir -p calibration output
export ROS_IP=192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311
export ODIN2_IMAGE=ghcr.io/gloryhry/odin2_ros_driver:latest
docker compose -f docker/compose.yaml up -d
docker compose -f docker/compose.yaml logs -f odin2
# 使用特定配置，可建立 compose override 或直接启动
docker run --rm --network host -e ROS_IP -e ROS_MASTER_URI \
  -v "$PWD/config:/config:ro" -v "$PWD/calibration:/calibration:ro" \
  -v "$PWD/output:/output" "$ODIN2_IMAGE" \
  roslaunch odin2_ros_driver driver.launch config:=/config/orin.yaml
```

Compose 使用 init、host 网络、restart unless-stopped 和120秒关闭宽限。运行用户默认为基础镜像用户，下载文件归属随其 UID；需要自定义用户时应给 ROS_HOME 与 output 目录写权限。配置与标定挂载只读，文件任务写 `/output`。不要在同一主机用不同容器重复控制同一个传感器。

外部 ROS Master 还要求 ROS 节点公布的 ROS_IP／ROS_HOSTNAME 对所有订阅端可达；只设置 ROS_MASTER_URI 不足以保证数据可达。device_ip／host_ip 用于选择设备和回传接口，不能代替 SDK 的设备查询握手。host 网络仅适用于 Linux 的这种部署模式，跨子网／防火墙须允许 SDK 的广播与动态 UDP/TCP 回传端口。

## GitHub Actions → GHCR

仓库 workflow `.github/workflows/docker.yaml` 使用固定版本 Action SHA，Buildx／QEMU、分架构缓存和 GITHUB_TOKEN。启用 Actions，并允许 GITHUB_TOKEN 写 Packages；fork PR 不需要 registry 写入凭据，也不会推送镜像。首次上传后可在 Package 设置连接仓库并设置可见性。

| 触发 | 行为／标签 |
|---|---|
| pull_request | 通用两架构及 Jetson 18 项矩阵编译、gtest、ROS 集成、runtime smoke，导出报告，不推送 |
| main / master push | 两架构分别构建成功后发布平台镜像，再合并通用多架构 manifest；更新各自 `latest` |
| vX.Y.Z tag（含预发布） | 通用、amd64、arm64、JetPack 5/6 镜像均发布版本标签并更新对应 `latest` |
| workflow_dispatch | 按所选 ref 构建／发布对应 workflow 的镜像并更新 `latest` |

所有名称使用小写；本仓库发布如下镜像：

| 运行平台 | 镜像 | 基础环境 |
|---|---|---|
| 自动匹配 amd64 / arm64 | `ghcr.io/gloryhry/odin2_ros_driver:latest` | Ubuntu20 / ROS Noetic，多架构 manifest |
| x86_64 | `ghcr.io/gloryhry/odin2_ros_driver-amd64:latest` | Ubuntu20 / ROS Noetic |
| 通用 ARM64 | `ghcr.io/gloryhry/odin2_ros_driver-arm64:latest` | Ubuntu20 / ROS Noetic |
| JetPack 5 | `ghcr.io/gloryhry/odin2_ros_driver-jetpack5:latest` | 别名目标 L4T 35.4.1 / Ubuntu20 / ROS Noetic |
| JetPack 6 | `ghcr.io/gloryhry/odin2_ros_driver-jetpack6:latest` | 别名目标 L4T 36.4.0 / Ubuntu22 / ROS Noetic 源码构建 |

通用／amd64／arm64 的 `latest` 指向对应平台最近成功发布的构建，包括手动选择的 ref 和预发布版本；通用 workflow 不生成新的 `sha-xxx` 或 `edge` 标签。Jetson 为全部 18 个列出的 L4T 提供 `l4t<VERSION>`、`sha-<完整提交号>-l4t<VERSION>` 以及版本发布的 `<发布版本>-l4t<VERSION>`。Jetson 的既有 `latest` 和无 L4T 后缀版本别名保留在 35.4.1／36.4.0；部署必须选择宿主对应的 L4T 标签及 digest。当前 36.4.3 使用 `-jetpack6:l4t36.4.3`。完整范围、预检、切换和回滚见 [Jetson 部署手册](jetson-deployment.md)。历史标签不删除。

镜像前缀自动取小写 `ghcr.io/<owner>/<repository>`，平台后缀固定。通用 workflow 的 publish job 依赖两项 build 全部成功，先按架构发布 `-amd64`／`-arm64` 镜像，再读取两个已测试的 digest 并合并 manifest；Job summary 记录 manifest 和架构 digest。OCI 标签含 source、revision 与 SDK 版本，镜像内 `/opt/odin2/share/odin2_ros_driver/build-info.txt` 记录 SDK、源码修订、目标架构、编译器。SBOM 和 provenance 随 CI 发布。PR 测试 XML 和每架构 digest 分别以 artifact 保存。

```bash
# 检查通用多架构入口和指定平台镜像
docker buildx imagetools inspect ghcr.io/gloryhry/odin2_ros_driver:latest
docker buildx imagetools inspect ghcr.io/gloryhry/odin2_ros_driver-amd64:latest
# 固定部署使用实际 manifest digest
docker pull ghcr.io/gloryhry/odin2_ros_driver@sha256:YOUR_MANIFEST_DIGEST
```

自动部署在本方案中指将镜像发布到 GHCR，不自动登录设备／重启生产容器。设备端通过 Compose pull 与 up 使用指定标签或 digest 更新。Jetson workflow 从统一清单生成全部 18 项原生 ARM 构建，汇总验证后发布到 `-jetpack5`／`-jetpack6`。同一 workflow 的发布构建串行执行，避免不同 ref 同时覆盖 `latest`。通用镜像仅在两架构构建成功后发布。镜像构建与发布成功不代表实机硬件解码验收。

## 启动失败诊断

配置 `device_ip` 仍需先完成 SDK 的 UDP `DeviceQuery`（60001 端口），此请求携带设备回传数据所需的主机地址；IP、SN 和 `host_ip` 用于从发现结果中筛选唯一目标。每次重连也会重新查询。仅能 ping 通或建立 TCP 命令连接不代表订阅初始化已完成。请保证发现广播及其应答可通过设备所在接口，容器使用 host 网络。未找到目标或匹配到多个设备时，驱动报错重试，不会连接其他设备。

先查看 `/odin2/status` 的 `state`、`last_error`、`reconnects` 和各通道计数。`retrying` 且 `reconnects=0` 表示初始化尚未完成。驱动会记录失败接口、通道、传输方式和 SDK 返回码；SDK 原始日志写入标准错误并及时刷新，可直接通过 `docker logs` 查看。

`OdinCommand::SendStartStream: error code 1` 表示设备已应答，但拒绝了通道订阅，设备协议的 `ret_code=1` 仅表示通用失败。它与驱动中的 `SDK result 1`（SDK 未初始化或设备未连接）不是同一组错误码。应结合失败通道、设备固件版本及匹配的 SDK 协议继续排查，不能仅凭此码认定网络断开或固件版本不兼容。`algo downlink 0x40 subscribe rejected` 是可选下行通道警告，SDK 会继续初始化；启动失败应看其后的具体接口错误。

需要详细 SDK 日志时，可先停止原服务，再临时运行诊断实例，避免同一设备被重复控制；诊断结束后按 Ctrl+C 并恢复原服务：

```bash
docker compose -f docker/compose.yaml stop odin2
docker compose -f docker/compose.yaml run --rm --no-deps -e LOG_LEVEL=3 odin2
# 诊断结束后
docker compose -f docker/compose.yaml up -d odin2
```

## Orin／网络调优

先使用默认 worker_threads=2，按状态与 CSV 记录增至3～4；更多线程不一定提高吞吐。适当减小队列可以降低积压延迟，但会增加丢弃，需要结合业务选择。仅编码透传可关闭 publish_decoded；MJPEG compressed 直接保留 JPEG，因此不做二次编码。

系统调优需用户在目标主机上显式设置，驱动不自动更改电源模式／sysctl：

```bash
# 先读取，确认硬件允许的功耗与散热策略
sudo nvpmodel -q
# 选择电源模式／jetson_clocks 应按具体 Orin 型号和现场策略执行
sysctl net.core.rmem_max net.core.wmem_max net.core.netdev_max_backlog
# 下面是候选值，设置前按网络带宽和系统内存评估
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
sudo sysctl -w net.core.netdev_max_backlog=5000
ip -s link show YOUR_SENSOR_INTERFACE
```

原版 SDK 的 socket buffer 请求与上限受内核限制；改变最大值不保证实际 socket 全部采用此大小。记录网卡丢包、PTP、温度、CPU 频率及同机其他负载，避免只凭 ROS 帧号缺口判断网络根因。

## 基准记录

```bash
# PID 是测量所在机器／容器 PID namespace 内的 odin2_node PID
rosrun odin2_ros_driver benchmark.py --device odin2 --seconds 120 \
  --pid YOUR_DRIVER_PID --output /output/odin2-benchmark.csv
```

CSV 包括墙钟、各通道 received／published／decoded_frames、序号缺口、队列丢弃／深度／字节、解码失败、最新回调到主机发布延迟、RSS、CPU 百分比及接收／发布／解码速率。CPU 百分比以一个逻辑 CPU 为100%，多核可超过100%；ROS status 每0.5秒采样，last_latency_ms 是最近包的主机处理延迟，不是所有包延迟分布、网络延迟或传感器到ROS总延迟。

测量至少覆盖默认输出、双目MJPEG、H264/H265、编码透传、全通道和两设备，每种模式运行5～10分钟，保存配置、固件、网卡、系统／镜像版本、CPU频率与功耗。QEMU 数据只用于功能验证，不作为 ARM／Orin 实机性能结论。

## 硬件解码部署

驱动默认自动优先硬解。NVIDIA、VAAPI、V4L2 与 JetPack5/6 的镜像和 Compose 用法见 [硬件解码](hardware-decoding.md)。通用镜像未传入设备时自动使用 CPU，专用 Jetson 镜像核对主机 L4T 版本。
