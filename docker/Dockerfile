# syntax=docker/dockerfile:1
ARG BASE_IMAGE=ros:noetic-ros-base
FROM ${BASE_IMAGE} AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ARG SOURCE_REVISION=unknown
ARG TARGETARCH
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config libeigen3-dev libssl-dev libavcodec-dev libavutil-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-bad \
    ros-noetic-catkin ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros \
    ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . /ws/src/odin2_ros_driver
RUN source /opt/ros/noetic/setup.bash && catkin_make -j${BUILD_JOBS} -DCMAKE_BUILD_TYPE=Release \
    -DODIN_DRIVER_NATIVE=OFF -DCATKIN_ENABLE_TESTING=OFF
FROM build AS install
RUN source /opt/ros/noetic/setup.bash && catkin_make install && \
    mkdir -p /ws/install/share/odin2_ros_driver/vendor && \
    tar -czf /ws/install/share/odin2_ros_driver/vendor/sdk-source.tar.gz -C /ws/src/odin2_ros_driver sdk_api && \
    cp /usr/share/common-licenses/GPL-2 /ws/install/share/odin2_ros_driver/vendor/GPL-2 && \
    cp /ws/src/odin2_ros_driver/THIRD_PARTY_NOTICES.md /ws/install/share/odin2_ros_driver/vendor/ && \
    cp /ws/src/odin2_ros_driver/LICENSE /ws/install/share/odin2_ros_driver/ && \
    tar --exclude=sdk_api --exclude=.git --exclude=build --exclude=devel --exclude=install \
      --exclude=output --exclude=__pycache__ -czf /ws/install/share/odin2_ros_driver/vendor/driver-source.tar.gz \
      -C /ws/src/odin2_ros_driver . && \
    printf 'sdk=3.3.0_20260822\nsource=%s\narchitecture=%s\ncompiler=%s\n' \
      "$SOURCE_REVISION" "$TARGETARCH" "$(c++ -dumpfullversion)" > /ws/install/share/odin2_ros_driver/build-info.txt
ARG BASE_IMAGE=ros:noetic-ros-base
FROM ${BASE_IMAGE} AS runtime-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl1.1 libavcodec58 libavutil56 libswscale5 \
    libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-bad \
    mesa-va-drivers \
    ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros \
    ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then apt-get update && \
    apt-get install -y --no-install-recommends intel-media-va-driver && \
    rm -rf /var/lib/apt/lists/*; fi
COPY --from=install /ws/install /opt/odin2
COPY docker/entrypoint.sh /odin2-entrypoint.sh
RUN chmod +x /odin2-entrypoint.sh && mkdir /output
FROM runtime-base AS runtime
RUN source /opt/ros/noetic/setup.bash && source /opt/odin2/setup.bash && \
    ! ldd /opt/odin2/lib/odin2_ros_driver/odin2_node | grep -q 'not found'
ENV ROS_MASTER_URI=http://127.0.0.1:11311
ENTRYPOINT ["/odin2-entrypoint.sh"]
CMD ["roslaunch", "odin2_ros_driver", "driver.launch"]
LABEL org.opencontainers.image.title="Odin2 ROS1 driver" \
      org.opencontainers.image.description="ROS Noetic; automatic hardware decode with CPU fallback; amd64/arm64; SDK 3.3.0"
