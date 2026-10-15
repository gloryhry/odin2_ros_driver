# JetPack 5／6 Docker 部署

本手册对应 `docker/jetson-versions.json` 的明确版本清单。镜像构建支持与现场验收分别记录在 [jetson-validation.json](jetson-validation.json)；没有记录真机结果的版本不能视为已完成硬解验收。当前目标 Orin 主机为 **36.4.3**，原容器使用通用 ARM64 镜像和 `runc`，需切换到对应 Jetson 镜像及 NVIDIA Runtime。

## 支持范围与官方依据

截至 2026-10-15，纳入以下 18 个正式版本，包括 NVIDIA 发布的 APT 安全更新。`35.1`、`36.3`、`36.4` 输入规范化为三段版本；拒绝开发预览版 `34.1`、`34.1.1`、`36.2` 以及 IGX 的 `36.4.5`、`36.4.6`，也拒绝其他未列出的补丁号。

| JetPack 系列 | JetPack 发布 | L4T |
|---|---|---|
| 5 | 5.0.2 | 35.1.0 |
| 5 | 5.1 | 35.2.1 |
| 5 | 5.1.1 | 35.3.1 |
| 5 | 5.1.2 | 35.4.1 |
| 5 | 5.1.3 | 35.5.0 |
| 5 | 5.1.4 | 35.6.0 |
| 5 | 5.1.5 | 35.6.1 |
| 5 | 5.1.5 | 35.6.2 |
| 5 | APT security update | 35.6.3 |
| 5 | 5.1.6 | 35.6.4 |
| 5 | 5.1.7 | 35.6.5 |
| 6 | 6.0 | 36.3.0 |
| 6 | 6.1 | 36.4.0 |
| 6 | 6.2 | 36.4.3 |
| 6 | 6.2.1 | 36.4.4 |
| 6 | APT security update | 36.4.7 |
| 6 | 6.2.2 | 36.5.0 |
| 6 | 6.2.3 | 36.5.2 |

常规对应关系依据 [JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive)；35.6.3、36.4.7 依据 [NVIDIA 安全更新公告](https://forums.developer.nvidia.com/t/jetpack-releases-with-security-fixes-cve-2025-33182-cve-2025-33177/347678)。可用 Jetson 型号遵循 [Jetson Linux Archive 平台矩阵](https://developer.nvidia.com/embedded/jetson-linux-archive)。早期 JetPack 5 不承诺覆盖后来推出的 Orin 型号，部署前须核对模块与载板支持。

## 镜像与构建契约

所有版本分别构建 `linux/arm64` 镜像，仓库为 `ghcr.io/gloryhry/odin2_ros_driver-jetpack5` 或 `-jetpack6`。每次发布生成 `l4t<VERSION>`、`sha-<完整提交号>-l4t<VERSION>`；版本标签构建另生成 `<发布版本>-l4t<VERSION>`（去掉 Git 标签的 `v`，保留预发布后缀）。部署固定标签及 digest。`latest` 和无 L4T 后缀的发布版本标签保留原兼容契约：JetPack 5 指向 **35.4.1**，JetPack 6 指向 **36.4.0**，不能据此推断与宿主匹配。

清单固定 ARM64 Ubuntu 基础镜像 digest、NVIDIA GStreamer 完整 Debian 版本、下载 URL 和 SHA256，以及 Ubuntu 依赖与签名快照日期。依照 [NVIDIA 容器 recipe](https://gitlab.com/nvidia/container-images/l4t-jetpack) 提取插件，不运行 BSP 包的安装脚本、不安装内核，也不将 Ubuntu 镜像伪装成已安装 L4T。NGC 无需存在全部对应标签。底层 BSP 共享库和设备由宿主 NVIDIA Runtime 注入。

JetPack 5 使用 Focal／Noetic 二进制包；JetPack 6 使用 Jammy，按 `docker/noetic-sources.lock.json` 的 75 个源码归档及 SHA256 构建 Noetic，保留 rosconsole 官方 glog 后端。Ubuntu 依赖由签名快照固定解析。镜像保留：

- `/opt/odin2/jetson/jetson-versions.json` 和 `noetic-sources.lock.json`：构建输入。
- `/opt/odin2/jetson-metadata.json`：目标 L4T、实际基础镜像、插件包及源码提交。
- `/opt/odin2/ubuntu-package-manifest.txt`：实际安装的 Ubuntu／ROS 包清单。
- JetPack 6 的 `/opt/ros/noetic/source-manifest.lock.json`：源码校验清单。

```bash
# 在仓库根目录生成所需构建参数；在 ARM64 上构建或自行配置 QEMU。
L4T=36.4.3
BASE=$(python3 docker/jetson-support.py select --l4t "$L4T" | python3 -c 'import json,sys; print(json.load(sys.stdin)["base_image"])')
docker buildx build --platform linux/arm64 --load --target runtime \
  -f docker/Dockerfile.jetson --build-arg L4T_BASE_IMAGE="$BASE" \
  --build-arg L4T_VERSION="$L4T" --build-arg SOURCE_REVISION="$(git rev-parse HEAD)" \
  --build-arg BUILD_JOBS=2 -t "odin2-ros-driver:l4t$L4T" .
```

入口将宿主 `/etc/nv_tegra_release` 与已安装的 `nvidia-l4t-core` 版本交叉核对，再比较镜像目标清单。宿主部分升级、镜像目标不匹配或不支持的版本会明确失败；实际 Ubuntu 基础镜像 tag 不参与 L4T 版本推断。

## 宿主准备与环境预检

主机需有与其 JetPack 匹配的 NVIDIA Container Runtime，并在 `docker info --format '{{json .Runtimes}}'` 中注册 `nvidia`。Compose 显式指定 `runtime: nvidia`、`NVIDIA_VISIBLE_DEVICES=all` 和 `NVIDIA_DRIVER_CAPABILITIES=all`、`NVIDIA_REQUIRE_JETPACK=csv-mounts=all`，只读挂载宿主版本文件与 dpkg 状态。Jetson 设备和 BSP 库按宿主 `/etc/nvidia-container-runtime/host-files-for-container.d/*.csv` 注入，机制见 [NVIDIA CSV mode 文档](https://github.com/NVIDIA/nvidia-container-toolkit/blob/main/cmd/nvidia-container-runtime/README.md#csv-mode)。Toolkit 1.11／1.16 的 CSV 实现会在缺少 `NVIDIA_REQUIRE_JETPACK=csv-mounts=all` 时只选基础 CSV；镜像、Compose 和预检均显式保留该约定，依据 [NVIDIA 1.16.2 源码](https://github.com/NVIDIA/nvidia-container-toolkit/blob/v1.16.2/internal/modifier/csv.go)。运行时同时保留 `tegra`／`tegra-egl`／`nvidia` 库搜索路径。仅添加 `--gpus all` 或在容器中安装 CUDA 不足以修复缺失的 Jetson 多媒体环境。

预检检查宿主版本与 core 包一致、ARM64、已注册 Runtime、本地镜像平台与目标、实际 NVDEC／nvmap 字符设备和访问权限、动态库及 GStreamer `appsrc/appsink/jpegparse/h264parse/h265parse/nvv4l2decoder/nvvidconv`。支持两代设备与 `tegra`／`nvidia` 库目录；JetPack 6 的 `/dev/v4l2-nvdec` 是 `1,3` 属于正常行为，不能据此判错。检查只创建临时容器，关闭网络、覆盖 ROS 入口，不连接 Odin2、不拉取镜像、不重启服务；退出码 0 为通过，1 为失败。临时容器始终清理。

```bash
# 主机上显式获取已发布镜像；镜像尚未发布时可先使用上面的本地构建。
IMAGE=ghcr.io/gloryhry/odin2_ros_driver-jetpack6:l4t36.4.3
docker pull "$IMAGE"
PIN=$(docker image inspect "$IMAGE" --format '{{index .RepoDigests 0}}')
scripts/jetson-preflight.sh --image "$PIN"
```

预检要求完整 Jetson Linux 多媒体 BSP 环境；缺库／缺节点时先检查匹配版本的宿主 Runtime CSV 及 BSP 包，不从其他 L4T 复制 `.so`。本修复不升级宿主系统、驱动或内核。使用 `ghcr.nju.edu.cn` 等镜像代理时显式替换镜像引用，并核对下载后 digest；预检不自动改用其他 registry 或版本。

## 当前 36.4.3 主机的手动切换和回滚

以下在目标主机操作；仓库不自动 SSH 部署。先确定现有 `odin2` 容器，例如 `C=docker-odin2-1`。所有命令沿用原 Compose 项目名；保存当前双目配置、命令、环境和挂载。只有环境预检通过、展开配置核对完毕后才切换。备份包含环境信息，按私有文件保存。

```bash
C=docker-odin2-1
PROJECT=$(docker inspect "$C" --format '{{index .Config.Labels "com.docker.compose.project"}}')
SERVICE=$(docker inspect "$C" --format '{{index .Config.Labels "com.docker.compose.service"}}')
test -n "$PROJECT" && test "$SERVICE" = odin2
WORKDIR=$(docker inspect "$C" --format '{{index .Config.Labels "com.docker.compose.project.working_dir"}}')
FILES=$(docker inspect "$C" --format '{{index .Config.Labels "com.docker.compose.project.config_files"}}')
# 在原工作目录与原 .env／环境下展开；核对 inspect 与展开结果一致后继续。
cd "$WORKDIR"
compose=(docker compose -p "$PROJECT")
IFS=, read -r -a paths <<< "$FILES"
for file in "${paths[@]}"; do compose+=(-f "$file"); done
umask 077
BACKUP="$WORKDIR/jetson-backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP"
"${compose[@]}" config > "$BACKUP/original.yaml"
docker inspect "$C" > "$BACKUP/container.json"
OLD_ID=$(docker inspect "$C" --format '{{.Image}}')
docker image inspect "$OLD_ID" > "$BACKUP/old-image.json"
printf '%s\n' "$PROJECT" > "$BACKUP/project-name"
# 保存本地不可变镜像的回滚入口；旧 registry digest 同时保存在 old-image.json。
ROLLBACK_IMAGE="odin2-rollback:$(date +%Y%m%d-%H%M%S)"
docker image tag "$OLD_ID" "$ROLLBACK_IMAGE"
printf 'services:\n  odin2:\n    image: %s\n' "$ROLLBACK_IMAGE" > "$BACKUP/rollback.yaml"
```

保留原配置文件及 `.env` 的私有副本；`original.yaml` 的 bind source 已展开为绝对路径。将新仓库绝对目录保存为 `REPO`，沿用上一节已预检的 `PIN`：

```bash
REPO=/absolute/path/to/Odin2-ROS-Driver
export ODIN2_JETSON_IMAGE="$PIN"
docker compose -p "$PROJECT" -f "$BACKUP/original.yaml" \
  -f "$REPO/docker/compose.jetson.yaml" config > "$BACKUP/candidate.yaml"
diff -u "$BACKUP/original.yaml" "$BACKUP/candidate.yaml" || true
# 确认只变更镜像、NVIDIA Runtime／环境及两个宿主状态挂载。
# 尤其核对双目配置路径、command、ROS_IP、host 网络和所有现场挂载。
"$REPO/scripts/jetson-preflight.sh" --image "$PIN"
docker compose -p "$PROJECT" -f "$BACKUP/candidate.yaml" \
  up -d --no-deps --no-build --pull never --force-recreate odin2
```

只重建原项目的 `odin2` 服务，不另起项目、不同时运行第二个驱动来控制同一设备。检查 `docker inspect` 的 runtime、镜像 ID／digest、挂载及 `/diagnostics`，随后完成下节验收。原容器被替换前会按原停止宽限关闭。若预检失败则不执行 `up`。

失败回滚：

```bash
# 如旧 registry digest 可用，也可将 rollback.yaml 的 image 固定为该 digest。
# 本地回滚标签绑定备份时的 OLD_ID；核对 old-image.json 后执行。
docker compose -p "$PROJECT" -f "$BACKUP/original.yaml" -f "$BACKUP/rollback.yaml" \
  up -d --no-deps --no-build --pull never --force-recreate odin2
```

验证旧服务恢复后再处理失败证据。保留备份与旧镜像，直到新部署的现场验收完成；不执行 `down -v`、系统升级或自动清理。

## 验收与故障可见性

压缩图像解码启用后实际使用 CPU 时，驱动将当前回退合并为诊断 WARN；硬解恢复后清除该 WARN，其他 WARN／ERROR 按原状态保留。未启用、仅透传、尚未启动解码以及 NV12 转换不会误报 CPU 回退。候选失败仍冷却 30 秒并自动重试；`hardware_active=true` 只在收到实际硬解输出后成立。`swscaler` 的 full-range／无加速颜色转换日志可出现在 CPU 色彩转换中，不能以屏蔽日志作为修复验收。

首先在 **36.4.3** 主机用现场分辨率／帧率连续运行双路真实 MJPEG **10 分钟**，记录：

- 两路 `/diagnostics` 均为 `jetson/nvv4l2decoder`、`hardware_active=true`，记录 decoder_device 与回退原因。
- 图像尺寸、颜色、相机对应关系及时间戳正确，图像与原始编码 header 对应；不是只看到话题就判通过。
- 定期采样 received／published／decoded_frames／decode_errors／queue_drops 的增量，解码错误和队列丢弃不持续增长；结合发布限频解释帧数差异。
- 记录实际帧率、`docker stats --no-stream` 的 CPU／内存和宿主 `tegrastats`，包括开始／结束时间、镜像 digest、源码提交、宿主 release／core 包版本。
- 在同一项目正常停止／重启 `odin2`，确认退出不超时、恢复后两路重新确认硬解且设备未被重复控制。

各 L4T 分别执行 MJPEG／H264／H265 双独立上下文，包含正常恢复和资源释放测试。35.6.3、36.4.7 另做内存压力与视频重启，结合上述安全公告记录 NVIDIA 已公开的宿主回归及其实际影响；不可将宿主回归或未执行项标为通过。现有 `driver_tests` 的 `HardwareDecoder.RealHardwareWhenExplicitlyRequested` 可提供三格式双上下文合成烟测，在匹配 Jetson 上以 `ODIN_TEST_REQUIRE_HARDWARE=1` 显式启用，但不能替代真实双目验收。

每版本独立保存 `build`、`environment_preflight`、`real_hardware` 的结果、日期、digest 和证据路径。完整支持验收需 18 个版本分别具备对应真机记录。无目标设备时保持 `not_run`，新增官方版本通过更新清单、包 SHA256、锁文件及记录纳入。

## CI

`.github/workflows/jetson.yaml` 从同一目录生成 18 项原生 ARM64 构建矩阵。PR 编译、解码／回退／诊断恢复回归、安装空间 smoke 并导出 XML；主分支、版本标签与手动发布沿用 GHCR、SBOM／provenance。矩阵只推送 digest，汇总任务先核对全部 18 份构建记录、平台和 OCI 提交／L4T 标签，再发布每版本标签及两个公共别名，避免矩阵互相覆盖。CI 报告的环境／真机项始终保持 `not_run`，等待现场证据补齐。
