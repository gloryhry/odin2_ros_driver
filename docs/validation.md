# 验证记录与实机验收

基础驱动验证日期：2026-10-14；以下16项测试与镜像指纹为自动硬解变更前的记录。功能验收使用模拟设备、FFmpeg 生成的16×16协议样例、原版 SDK assembler 和容器。没有接入真实 Odin2、ARM 主机或 Jetson；QEMU 结果不代表 ARM／Orin 性能。

自动硬解变更的测试、NVIDIA GPU 结果及尚未完成的硬件验收单独记录在 [硬件解码](hardware-decoding.md) 和 [hardware-decoding-validation.json](hardware-decoding-validation.json)，不覆盖下列已有证据。

## 此前完整本地版本的自动化覆盖

以下结果来自包含测试的完整本地版本。本次提交排除驱动 `test/`、测试构建配置、测试依赖及 Docker／CI 测试执行步骤，保留本地完整版本；包含 SDK 独立的 `https_client_test` 示例。提交后的 Docker／CI 仅执行生产构建与动态库检查。

`test/driver_tests.cpp`：16项 gtest，涵盖点云字段／缩放／RGBA／NaN／空包、SI IMU、Odom quaternion 与原始18项 covariance、跨通道时间映射／回退／极值／ROS跳变、双界队列、能力掩码与非相机模式拒绝、NV12校验、MJPEG/H264/H265真实软件解码、独立参数NAL／丢帧后恢复／双目解码隔离。LiveSdk额外验证不可达设备连接返回失败且不崩溃。SDK protocol 测试直接链接原版 SDK，确认IMU逐样本时间、原始点云偏移和毫米缩放、四格式TCP拆包组装。

`test/integration.test`／`integration.py`：多设备 ROS 集成，覆盖独立 frame／世界SLAM内部LF依赖、双目不同编码、关闭输出、服务／dynamic配置、能力拒绝、启动失败回滚与恢复失败未就绪、部分滤波失败回滚、文件任务进度、下载失败与任务错误、算法下行1与256KiB边界、遥测kOk误报、重复包、命令离线、数据停流重连、清除旧时间、模式读回、网络持久化重启目标切换与正常进程退出。

`test/runtime_smoke.py`：runtime 阶段使用安装到 `/opt/odin2` 的正式节点、SDK动态库和生成Python消息启动，检查状态输出与正常退出。目标设备故意不可达，此项不是实机采集测试。镜像构建同时检查 ldd 无缺失库。

| 检查 | 结果 |
|---|---|
| 原版 SDK x86_64 与 Ubuntu20/GCC9 编译 | 已通过 |
| amd64 最终16项单测／多设备ROS集成／runtime smoke | 全部通过，GCC9.4.0，Ubuntu20／Noetic |
| arm64 QEMU 最终16项单测／多设备ROS集成／runtime smoke | 全部通过，OCI平台元数据确认为linux/arm64 |
| SDK原始文件SHA256校验 | 106项全部通过，原始文件未修改 |
| 实机吞吐、丢包、PTP、固件兼容性 | 待实机验收 |
| GHCR远端镜像发布 | 本轮未触发；由提交到GitHub后的workflow完成 |

完整结构化记录见 [validation-results.json](validation-results.json)，包含测试代码指纹与本地镜像digest。amd64 镜像为 `odin2-ros-driver:local`；arm64 OCI归档位于 `/tmp/odin2-arm64-runtime.tar`。两架构 Catkin 汇总均为 `34 tests, 0 errors, 0 failures, 0 skipped`，对应16项独立gtest、一个多设备集成场景及汇总节点的重复计数。安装镜像真实SDK节点在故意不可达设备条件下正常报告retrying并退出。

静态检查通过：actionlint1.7.12、clang-format18.1.8、全部YAML／XML／Python语法和Compose解析。验证过程中发现SDK错误日志的空回调问题，封装按供应商示例初始化logger；真实SDK不可达连接与零句柄拒绝均有回归覆盖。

## 生产构建复现

提交后的生产构建方式见 [部署文档](deployment.md)。上述自动化覆盖需要另行保留的本地测试版本，驱动测试文件及其构建配置不随本次提交交付。

## 实机验收清单

- [ ] 记录设备型号、SN、固件、SDK／镜像digest、CPU／JetPack／内核、网卡型号、配置与标定文件。
- [ ] 默认模式启动，关闭相机／SLAM／Odom／TF无消息；启用通道的类型与频率正确。
- [ ] 对已知距离靶确认米制点云；验证XYZ方向、SDK Y/Z偏移不重复，强度／置信度／RGBA字段正确。
- [ ] IMU rad/s、m/s²、逐样本stamp正确，静置与旋转测试；orientation covariance=-1。
- [ ] 各相机枚举实际能力表；分别MJPEG/H264/H265/NV12、有效／无效模式；解码色彩、尺寸和标定一致。
- [ ] JPEG compressed 与SDK编码字节一致；H264/H265不被误标为JPEG；网络中断后恢复帧可解码，双目不串流。
- [ ] 原始Odom18项covariance原样；标准36项为配置值，未配置产生占位警告；四元数方向与TF正确。
- [ ] SLAM传感器模式为IMU坐标，世界模式为odom并有内部LF依赖；LF关闭发布时无ROS Odom消息。
- [ ] 两台设备SN/IP严格匹配，topic／frame／时间／任务互相隔离；反复修改一台不改变另一台输出。
- [ ] 服务读回实际订阅与模式；模拟命令失败可观测且旧配置恢复，恢复失败明确未就绪。
- [ ] 遥测字段与固件schema核对，拒绝仅凭StartTelemetry返回成功判定通道健康。
- [ ] PTP未锁／锁定／失锁状态正确；核对设备epoch后才使用offset模式。
- [ ] 拔网线／设备重启／只停数据，恢复后不发布旧队列和旧时间戳；重试间隔最大30秒。
- [ ] 下载标定／地图／日志、上传地图及维护窗口固件任务，状态、错误、百分比与文件内容正确。
- [ ] 持久化网络切换并显式reboot，静态IP／DHCP后重新发现，重启驱动时YAML一致。
- [ ] 常规停机和文件任务中停机，确认接收线程、控制线程与进程退出，不遗留端口／上下文。
- [ ] x86、ARM64、Orin各自运行至少5～10分钟负载矩阵，记录CSV、RSS／CPU／解码FPS／队列丢弃／温度／频率。

## 故障排查

| 现象 | 排查路径 |
|---|---|
| discovering／retrying | status.last_error；配置SN／IP／host_ip、广播路由，必要时直接IP |
| 能力组合拒绝 | get_device_info能力表；format是单一位，NV12必须偶数显式尺寸 |
| subscribed=true但无数据 | 网卡回传地址、动态端口防火墙、设备模式、status.last_receive_age |
| 遥测没启动却kOk | get_channel_config和真实到达；固件遥测订阅／JSON schema |
| decode_errors增长 | 编码格式、负载、参考帧损失、SPS/PPS/VPS、设备恢复帧；查sequence_gaps与queue_drops |
| queue_drops增长 | CPU／订阅端负载、worker_threads、分辨率、队列容量；队列增加可能提高延迟 |
| ROS看到Master却收不到话题 | ROS_IP／ROS_HOSTNAME发布地址和跨机端口可达性 |
| invalid_packets且ROS时间为0 | /use_sim_time与/clock；等待有效时钟 |
| 时间／frame不一致 | 每设备tf_prefix、slam_world、时间模式；PTP locked不能替代epoch确认 |
| CameraInfo缺失 | 标定URL、权限、图像尺寸；尺寸错误会抑制输出 |
| 文件busy或退出等待 | SDK同步文件调用无取消；查询任务而非重复提交 |
| 动态库不存在 | source安装setup.bash，ldd节点；使用完整SO版本链，勿只拷贝无目标的symlink |
