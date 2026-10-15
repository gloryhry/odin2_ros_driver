# 自动硬件解码

编码由 Odin2 完成。驱动对 MJPEG/H264/H265 自动尝试硬件解码，全部候选不可用时使用 FFmpeg CPU 解码。NV12 仅转换为 bgr8，EncodedFrame 和 MJPEG CompressedImage 保留设备原始字节。默认配置即可使用此行为，无需选择后端。

## 选择与恢复

顺序为 Jetson nvv4l2decoder、CUDA/NVDEC、VAAPI、QSV、V4L2 M2M、VDPAU、CPU。FFmpeg 配置与当前 codec 匹配后，还须通过设备初始化和实际码流的 profile、分辨率、像素格式协商。专用 CUVID／V4L2 解码器在解析实际码流尺寸后初始化，避免默认尺寸造成误判。硬件候选拒绝静默软件输出；hardware_active 仅在实际产出硬件解码帧后为 true。硬件帧下载时保留色彩范围和矩阵元数据，再转换为 bgr8。CPU 内存下载和 BGR 色彩转换不属于 CPU 码流解码。

CUDA 设备按编号、DRM render 节点按数字顺序尝试。旧版 FFmpeg 的 V4L2 M2M wrapper 自行扫描设备，没有可指定设备的公开参数，因此诊断显示 device=auto；驱动不会将猜测的设备路径当作实际使用路径。QSV 等后端只有在所链接 FFmpeg 编译了对应接口时才会被枚举；Ubuntu20 默认 FFmpeg 没有 QSV 时，Intel 使用 VAAPI。

设备、硬件格式协商或硬件帧下载失败后尝试其余硬解，再回退 CPU。失败候选冷却 30 秒，在下一 MJPEG／随机接入帧重新尝试；编码格式、已知分辨率或 SPS/PPS/VPS 内容变化，以及设备／时钟重连立即重新探测。损坏码流重置参考状态，保留当前连接参数集，不将硬件加入冷却。H264/H265 非随机接入帧不能在新上下文中重放，须等待 IDR／HEVC 随机接入帧。

Jetson 使用 appsrc → h264parse/h265parse/jpegparse → nvv4l2decoder → nvvidconv → appsink，MJPEG 开启 mjpeg=true。BGRx 主机帧转为 bgr8；不禁用 DPB，保留 B 帧兼容性。工作线程每 5 ms 轮询未完成任务。输出携带原始 header、接收时间和两级 generation；重新配置后旧任务不会发布。

输入包上限 64 MiB，参数 NAL 上限各 256 KiB，未完成上下文上限 64 个；GStreamer appsrc 限制 64 MiB，appsink 最多 8 帧。另保留最多一个独立帧用于异步故障后重试。MJPEG存在任意pending，或仅等待一个可恢复视频关键帧时，5秒无输出即判停滞；依赖视频尾帧保留至少8个pending的条件，避免误判正常重排序。计时从pending由空变为非空开始，实际输出刷新，清空后取消；长期空闲不消耗新任务的超时预算。重置先停止后端再丢弃旧缓冲，按其他硬件、CPU顺序恢复安全独立帧并保留原始上下文。

## 构建与容器

原生构建增加 libgstreamer1.0-dev、libgstreamer-plugins-base1.0-dev，以及 gstreamer1.0-plugins-base/bad。可用 -DODIN_DRIVER_GSTREAMER=OFF 构建仅 FFmpeg 版本。硬件选择只看到当前进程可访问的设备，容器必须显式传入设备。

```bash
# NVIDIA 桌面 GPU：主机安装 NVIDIA Container Toolkit。
docker compose -f docker/compose.yaml -f docker/compose.nvidia.yaml up -d
# Intel/AMD VAAPI：传入 DRM 设备；镜像含 Mesa，amd64 额外含 Intel media driver。
docker compose -f docker/compose.yaml -f docker/compose.vaapi.yaml up -d
# ARM 等 V4L2 M2M 设备。
ODIN2_VIDEO_DEVICE=/dev/video0 docker compose -f docker/compose.yaml -f docker/compose.v4l2.yaml up -d
```

Jetson 专用镜像使用与主机完全匹配的 NVIDIA L4T 基础镜像和插件。通用 Ubuntu20 镜像可在 Jetson 上回退 CPU，但不能承诺专用硬解。先读取主机 /etc/nv_tegra_release，再选择匹配的 L4T 标签；`latest` 始终指向对应 JetPack 最近成功发布的构建。入口核对主机与镜像 L4T 版本，不匹配时明确退出。

```bash
# JetPack 5 示例（L4T 35.4.1）；默认基础镜像也是此版本。
docker build -f docker/Dockerfile.jetson --build-arg BUILD_JOBS=2 \
  --build-arg L4T_BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.4.1 \
  -t odin2-ros-driver:jetson-local .
# JetPack 6 示例（L4T 36.4.0）：Ubuntu22 内从发行源码构建 Noetic。
docker build -f docker/Dockerfile.jetson --build-arg BUILD_JOBS=2 \
  --build-arg L4T_BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0 \
  -t odin2-ros-driver:jetson-local .
docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml up -d
```

使用 GHCR 发布的镜像时设置 Compose 的专用镜像变量：

```bash
# 主机 L4T 35.4.1：JetPack 5
export ODIN2_JETSON_IMAGE=ghcr.io/gloryhry/odin2_ros_driver-jetpack5:l4t35.4.1
# 主机 L4T 36.4.0：JetPack 6（选择这一项时替换上一行）
# export ODIN2_JETSON_IMAGE=ghcr.io/gloryhry/odin2_ros_driver-jetpack6:l4t36.4.0
docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml pull
docker compose -f docker/compose.yaml -f docker/compose.jetson.yaml up -d
```

两个仓库也分别提供 `:latest`。通用 amd64／arm64 镜像命名见 [部署文档](deployment.md)。

JetPack6 镜像保存 Noetic 的 source-manifest.rosinstall 供核查。Jammy 源码构建使用 rosconsole 官方 glog 日志后端，避免发行版 log4cxx 的 API 不兼容，并保留 rosout 与动态 logger level 服务；JetPack5 使用 Focal 的 Noetic 二进制包。专用镜像保留 ROS 和 NVIDIA 基础环境的工具，体积会大于通用运行镜像；不复制其他 JetPack 版本的宿主库。生产构建不需要设备，实际硬解须在匹配的 Jetson 上验收。Jetson workflow 在 `main`／`master` 推送、版本标签和手动触发时构建，并发布对应平台的 GHCR 镜像。

## 诊断与验收

在 /diagnostics 中查看各相机 decoder_backend、decoder_device、hardware_active、cpu_fallback_reason、decoder_failures。后端选择、硬解确认和回退会记录 ROS 日志。ChannelStatus 的 published 仍按发布过输出的 SDK 包去重，decoded_frames 包含解码完成但因限频未发布的帧；消息格式不变。

完整本地测试版本覆盖故障注入、30 秒冷却、参考帧恢复、独立相机、延迟最后一帧与旧 generation 阻止发布。实际硬解检查要求 hardware_active=true，并确认有效 bgr8 图像。驱动测试源码、GPU ROS 验证脚本、测试 CMake 配置和 Docker／CI 测试步骤保留在本地，未随本次提交交付；下列验证证据来自该完整本地版本，提交后的镜像只进行生产构建与动态库检查。

硬解样例为 256×256 的 YUV420 JPEG、低延迟 H264/H265，另外保留 16×16 样例验证小尺寸 CPU 回退。实机还应验证实际 Odin2 profile／尺寸、双目、丢帧恢复、故障切换及资源释放；合成样例不代表设备固件和实机吞吐验收。

## 本轮验证记录（2026-10-14）

结构化记录见 [hardware-decoding-validation.json](hardware-decoding-validation.json)。已有基础驱动记录保留在 validation.md／validation-results.json。本轮未发布镜像。记录的镜像与源码指纹属于包含测试的完整本地版本。

- 无 GPU 的 amd64／arm64 容器：28项单测通过，显式硬解测试跳过；原有多设备 ROS 集成、新增异步 ROS 集成及安装空间 smoke 分别记录。arm64 使用 BuildKit QEMU，不代表原生 ARM 性能。
- NVIDIA RTX 5060 Laptop GPU，驱动595.91.07，通过 NVIDIA Container Toolkit 暴露给 amd64 容器：29项单测通过。256×256 MJPEG／H264／H265 分别确认 cuda/mjpeg、cuda/h264、cuda/hevc，两个独立上下文均产出 bgr8，原始 header 保持隔离，色彩与 CPU 输出对比通过。16×16不支持码流可回退 CPU。另以模拟 SDK 经过真实 ROS 工作线程，三格式双路均发布有效图像，diagnostics 确认硬解、图像／透传 header 匹配，SIGINT 正常退出。
- 故障切换、30秒冷却恢复和异步延迟／旧 generation 使用注入后端和模拟 SDK 验证，未人为移除正在使用的真实 GPU。
- Ubuntu22 的75个 Noetic 源码包构建通过；GCC11.4、FFmpeg4.4／GStreamer1.20 下28项单测、两个 ROS 集成与安装空间 smoke 通过。NVIDIA GPU 下29项单测、双路三格式 ROS 硬解及 logger 服务通过。此项使用普通 Ubuntu 基础镜像；不等同于匹配 L4T 的 JetPack 镜像构建或 Jetson 硬解验收。
- Intel／AMD VAAPI、QSV、V4L2 M2M、VDPAU、JetPack5/6、真实 Odin2 双目与长时间运行仍须在相应设备上验收。

GPU ROS 验证使用本地保留的 `docs/evidence/gpu-ros-validation.py`。脚本只在测试容器内替换模拟 SDK 的压缩载荷；模拟能力表／编码消息尺寸仍为16×16，实际载荷和解码图像为256×256。因此记录验证工作线程和 diagnostics 的硬解路径，不验证真实设备的能力表或固件。
