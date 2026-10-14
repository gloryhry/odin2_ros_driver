# 部署、发布与性能

## 平台支持边界

| 平台 | 构建／运行选择 | 验证边界 |
|---|---|---|
| x86_64 | Ubuntu20／ROS Noetic 原生或 linux/amd64 镜像 | 容器编译、模拟验收及 NVIDIA 合成码流硬解；真实 Odin2 待验收 |
| ARM64 | Ubuntu20 原生或 linux/arm64 镜像 | 本轮 QEMU 编译／模拟验收；实际 ARM CPU 性能待验证 |
| Orin JetPack5 | 通用或 JetPack 专用 ARM64 镜像，host 网络 | JetPack 专用镜像使用 nvv4l2decoder；实际 Jetson 待验收 |
| Orin JetPack6 | 专用 Ubuntu22 容器内源码构建 ROS Noetic | 匹配 L4T 库与插件；实际 Jetson 待验收 |

同架构发布镜像不启用 `-march=native`、fast-math 或 Orin 专有指令。Release 允许编译器基础自动向量化；可选 LTO。CPU 亲和性只影响驱动工作／控制线程。Orin 默认配置提供较小点云队列和编码透传选项；开启两路视频解码前，先按能力表选择合适分辨率／fps，再测量 CPU 和丢帧。

## Docker 和 Compose

```bash
# 单架构本机编译（生产构建与动态库检查）
docker build -f docker/Dockerfile --build-arg BUILD_JOBS=4 -t odin2-ros-driver:local .
# 多架构，适用于已配置 QEMU 的 Buildx 环境
docker buildx build --platform linux/amd64,linux/arm64 \
  -f docker/Dockerfile --build-arg BUILD_JOBS=2 \
  -t ghcr.io/owner/repository:manual --push .
```

Docker 是 build → install → runtime 多阶段，runtime 构建检查正式节点的动态库依赖。驱动测试源码、测试构建配置与 Docker／CI 测试执行步骤保留在本地，提交后的镜像不执行 gtest、ROS 模拟集成或安装空间 smoke；完整本地版本的验证结果见验证文档。通用运行镜像不包含编译器；Jetson 镜像保留匹配的 L4T／ROS 基础环境。

```bash
mkdir -p calibration output
export ROS_IP=192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311
export ODIN2_IMAGE=ghcr.io/owner/repository:edge
docker compose -f docker/compose.yaml up -d
docker compose -f docker/compose.yaml logs -f odin2
# 使用特定配置，可建立 compose override 或直接启动
docker run --rm --network host -e ROS_IP -e ROS_MASTER_URI \
  -v "$PWD/config:/config:ro" -v "$PWD/calibration:/calibration:ro" \
  -v "$PWD/output:/output" "$ODIN2_IMAGE" \
  roslaunch odin2_ros_driver driver.launch config:=/config/orin.yaml
```

Compose 使用 init、host 网络、restart unless-stopped 和120秒关闭宽限。运行用户默认为基础镜像用户，下载文件归属随其 UID；需要自定义用户时应给 ROS_HOME 与 output 目录写权限。配置与标定挂载只读，文件任务写 `/output`。不要在同一主机用不同容器重复控制同一个传感器。

外部 ROS Master 还要求 ROS 节点公布的 ROS_IP／ROS_HOSTNAME 对所有订阅端可达；只设置 ROS_MASTER_URI 不足以保证数据可达。设备发现广播受限时填写 device_ip／host_ip。host 网络仅适用于 Linux 的这种部署模式，跨子网／防火墙须允许 SDK 的广播与动态 UDP/TCP 回传端口。

## GitHub Actions → GHCR

仓库 workflow `.github/workflows/docker.yaml` 使用固定版本 Action SHA，Buildx／QEMU、分架构缓存和 GITHUB_TOKEN。启用 Actions，并允许 GITHUB_TOKEN 写 Packages；fork PR 不需要 registry 写入凭据，也不会推送镜像。首次上传后可在 Package 设置连接仓库并设置可见性。

| 触发 | 行为／标签 |
|---|---|
| pull_request | 两架构编译和运行时动态库依赖检查，不推送 |
| main push | 两架构分别构建成功后发布 digest，再合并 manifest；`sha-<完整SHA>`、`edge` |
| vX.Y.Z tag | `X.Y.Z` 和 SHA；稳定版本同时更新 `latest` |
| 带 `-` 的预发布版本 | 版本标签和 SHA，不更新 latest |
| workflow_dispatch | 按所选 ref 重新构建／发布；分支通常发布 SHA，main 另更新 edge |

镜像名称自动取小写 `ghcr.io/<owner>/<repository>`。publish job 依赖两项 build 全部成功，读取两个构建成功的 digest 并合并 manifest；Job summary 记录 manifest 和架构 digest。OCI 标签含 source、revision 与 SDK 版本，镜像内 `/opt/odin2/share/odin2_ros_driver/build-info.txt` 记录 SDK、源码修订、目标架构、编译器。SBOM 和 provenance 随 CI 发布。发布任务的每架构 digest 以 artifact 保存。

```bash
# 检查已经发布的版本（需真实仓库替换占位值）
docker buildx imagetools inspect ghcr.io/owner/repository:edge
docker pull ghcr.io/owner/repository@sha256:YOUR_MANIFEST_DIGEST
```

自动部署在本方案中指将镜像发布到 GHCR，不自动登录设备／重启生产容器。设备端通过 Compose pull 与 up 使用指定标签或 digest 更新。当前仓库未配置 Git remote，本轮只实现并本地验证 workflow，没有远端发布结果。

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
