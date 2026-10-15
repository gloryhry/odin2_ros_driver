# Odin2 ROS1 驱动

`odin2_ros_driver` 基于随仓库提供的 Odin SDK **3.3.0_20260822**，使用 ROS Noetic 和 C++17。支持 x86_64、ARM64、Jetson Orin JetPack 5/6。SDK 源码保持原样，图像自动优先硬件解码，全部硬解不可用时回退 CPU；Jetson 提供与 JetPack 匹配的专用镜像，见 [硬件解码与部署](docs/hardware-decoding.md)。

每台设备具有独立命名空间、连接状态机、有界队列、时间映射、解码器和动态配置。支持 SDK 的原始／SLAM 点云、左右相机、IMU、高低频里程计、TF 和遥测；默认仅启用原始点云、IMU、遥测和诊断。提供具名管理服务及异步文件任务。代码和模拟验收已交付；设备固件兼容性、真实网络、ARM／Orin 性能仍须实机验收，见 [验证记录](docs/validation.md)。

## 快速启动

```bash
# 在仓库根目录构建生产镜像，并检查运行时动态库依赖
 docker build -f docker/Dockerfile --build-arg BUILD_JOBS=4 -t odin2-ros-driver:local .
 mkdir -p calibration output
 # 修改 config/default.yaml，填写 SN，或填写 device_ip 和 host_ip
 export ROS_IP=192.168.1.100
 docker compose -f docker/compose.yaml up -d
```

单设备自动发现只接受唯一匹配。多设备必须各自指定 SN 或 IP，推荐填写两者。指定 IP 同样先执行 SDK 设备查询以建立回传地址，再筛选目标；须允许 UDP 60001 的发现广播和应答。`host_ip` 是传感器可回连的网卡地址，与 ROS 远程通信使用的 `ROS_IP` 各有用途。

```bash
# 通用镜像自动匹配 amd64 / arm64
export ODIN2_IMAGE=ghcr.io/gloryhry/odin2_ros_driver:latest
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100
docker compose -f docker/compose.yaml up -d
```

没有外部 Master 时，`roslaunch` 自动启动本机 Master。SDK 使用广播发现和动态回传端口，Linux 上推荐 host 网络。容器中的配置文件、标定与设备文件分别挂载到 `/config`、`/calibration`、`/output`。

## 原生 Catkin 构建

在已安装 ROS Noetic 的 Ubuntu 20.04 上：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake pkg-config libeigen3-dev libssl-dev \
  libavcodec-dev libavutil-dev libswscale-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-bad \
  ros-noetic-camera-info-manager ros-noetic-diagnostic-updater \
  ros-noetic-dynamic-reconfigure ros-noetic-tf2-ros
mkdir -p ~/odin2_ws/src
# 将当前仓库放在 ~/odin2_ws/src/odin2_ros_driver
cd ~/odin2_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DODIN_DRIVER_NATIVE=OFF
source devel/setup.bash
roslaunch odin2_ros_driver driver.launch config:=$(rospack find odin2_ros_driver)/config/default.yaml
# 安装空间也可启动，无需源码路径
catkin_make install
source install/setup.bash
```

SDK 作为构建依赖编译，示例程序关闭，完整 `libodin_sdk.so*` 链安装到 Catkin 的 `lib`。本机专用构建可启用 `-DODIN_DRIVER_NATIVE=ON`，可选 `-DODIN_DRIVER_LTO=ON`；发布镜像禁止使用本机指令集和 fast-math。ARM32 不在本轮支持范围内。

## 配置与接口

配置示例：

| 文件 | 用途 |
|---|---|
| [default.yaml](config/default.yaml) | 点云、IMU、遥测 |
| [all_channels.yaml](config/all_channels.yaml) | 所有 SDK 输出；需能力表支持 |
| [stereo.yaml](config/stereo.yaml) | 双目 MJPEG 与基本传感器输出 |
| [multi_device.yaml](config/multi_device.yaml) | 两台独立设备；必须替换 SN |
| [orin.yaml](config/orin.yaml) | 有界小队列、可选编码透传，按负载启用相机 |

```bash
rosservice call /odin2/get_device_info '{}'
rosservice call /odin2/get_channel_config '{channel: 0}'
rostopic echo /odin2/status
rosrun rqt_reconfigure rqt_reconfigure
sudo apt-get install ros-noetic-rviz
roslaunch odin2_ros_driver rviz.launch
```

完整说明：

- [全部参数与时间／坐标约定](docs/parameters.md)
- [话题、消息和全部服务调用](docs/interfaces.md)
- [架构、SDK 差异与生命周期](docs/architecture.md)
- [多平台、容器、GHCR 和性能测量](docs/deployment.md)
- [自动化验证与实机验收清单](docs/validation.md)

## CI 与版权

驱动测试源码和测试构建配置保留在本地，本次提交包含 SDK 独立的 `https_client_test` 示例。GitHub Actions 在 PR 上构建两种架构；`main`／`master`、版本标签和手动触发发布 GHCR 镜像。通用入口 `ghcr.io/gloryhry/odin2_ros_driver:latest` 自动匹配 `linux/amd64`／`linux/arm64`；各平台镜像为同名前缀加 `-amd64:latest`、`-arm64:latest`、`-jetpack5:latest`、`-jetpack6:latest`。每次成功发布更新对应 `latest`，不再生成 `sha-xxx` 标签；源码提交号保留在 OCI 元数据中。Jetson 另保留匹配 L4T 的标签，名称、版本和权限见 [部署文档](docs/deployment.md)。

[LICENSE](LICENSE) 仅覆盖新增驱动。供应商 SDK 和第三方版权保持原声明，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。镜像携带对应 SDK／驱动源码、构建信息和第三方说明。`package.xml` 维护者地址是占位值，公开发布前请维护者填写真实联系方式；本项目不重新授予 SDK 许可。
