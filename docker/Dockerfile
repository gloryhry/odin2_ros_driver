# syntax=docker/dockerfile:1
ARG BASE_IMAGE=ros:noetic-ros-base
FROM ${BASE_IMAGE} AS build
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ARG SOURCE_REVISION=unknown
ARG TARGETARCH
RUN apt-get -o Acquire::Retries=5 update && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    build-essential cmake pkg-config libeigen3-dev libssl-dev libavcodec-dev libavutil-dev libswscale-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-bad \
    ros-noetic-catkin ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros \
    ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-camera-info-manager \
    ros-noetic-rostest libpcl-dev ffmpeg python3-yaml && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . /ws/src/odin2_ros_driver
RUN source /opt/ros/noetic/setup.bash && catkin_make -j${BUILD_JOBS} -DCMAKE_BUILD_TYPE=Release \
    -DODIN_DRIVER_NATIVE=OFF -DCATKIN_ENABLE_TESTING=ON && catkin_make -j${BUILD_JOBS} tests
RUN mkdir -p /fixtures && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=16x16:r=10 -frames:v 1 -threads 1 /fixtures/mjpeg.jpg && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=16x16:r=10 -frames:v 1 -c:v libx264 -threads 1 -preset ultrafast -tune zerolatency -f h264 /fixtures/h264.bin && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=16x16:r=10 -frames:v 1 -c:v libx265 -threads 1 -tune zerolatency -x265-params pools=none:frame-threads=1:bframes=0:rc-lookahead=0 -f hevc /fixtures/h265.bin && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=256x256:r=10 -frames:v 1 -pix_fmt yuvj420p -threads 1 /fixtures/hw-mjpeg.jpg && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=256x256:r=10 -frames:v 1 -c:v libx264 -threads 1 -preset ultrafast -tune zerolatency -f h264 /fixtures/hw-h264.bin && \
    ffmpeg -loglevel error -f lavfi -i color=c=red:s=256x256:r=10 -frames:v 1 -c:v libx265 -threads 1 -tune zerolatency -x265-params pools=none:frame-threads=1:bframes=0:rc-lookahead=0 -f hevc /fixtures/hw-h265.bin && \
    ffmpeg -loglevel error -f lavfi -i testsrc2=s=16x16:r=25 -frames:v 8 -c:v libx264 -threads 1 -bf 0 -g 8 -preset ultrafast -tune zerolatency -x264-params aud=1:repeat-headers=1 -f h264 /fixtures/sequence.h264 && \
    ffmpeg -loglevel error -f lavfi -i testsrc2=s=16x16:r=25 -frames:v 8 -c:v libx265 -threads 1 -tune zerolatency -x265-params pools=none:frame-threads=1:bframes=0:rc-lookahead=0:aud=1:repeat-headers=1:keyint=8 -f hevc /fixtures/sequence.h265
FROM build AS test
ENV ODIN_TEST_FIXTURES=/fixtures
RUN source /opt/ros/noetic/setup.bash && catkin_make run_tests && catkin_test_results --all /ws/build/test_results
RUN source /ws/devel/setup.bash && /ws/devel/lib/odin2_ros_driver/driver_tests
FROM scratch AS test-results
COPY --from=test /ws/build/test_results /
FROM test AS install
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
RUN apt-get -o Acquire::Retries=5 update && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
    libssl1.1 libavcodec58 libavutil56 libswscale5 \
    libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-bad \
    mesa-va-drivers \
    ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-tf2-ros \
    ros-noetic-diagnostic-updater ros-noetic-dynamic-reconfigure ros-noetic-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then apt-get -o Acquire::Retries=5 update && \
    apt-get -o Acquire::Retries=5 install -y --no-install-recommends intel-media-va-driver && \
    rm -rf /var/lib/apt/lists/*; fi
COPY --from=install /ws/install /opt/odin2
COPY docker/entrypoint.sh /odin2-entrypoint.sh
COPY test/runtime_smoke.py /tmp/runtime_smoke.py
RUN chmod +x /odin2-entrypoint.sh && mkdir /output
FROM runtime-base AS runtime
RUN source /opt/ros/noetic/setup.bash && source /opt/odin2/setup.bash && \
    ! ldd /opt/odin2/lib/odin2_ros_driver/odin2_node | grep -q 'not found' && \
    python3 /tmp/runtime_smoke.py && rm /tmp/runtime_smoke.py
ENV ROS_MASTER_URI=http://127.0.0.1:11311
ENTRYPOINT ["/odin2-entrypoint.sh"]
CMD ["roslaunch", "odin2_ros_driver", "driver.launch"]
LABEL org.opencontainers.image.title="Odin2 ROS1 driver" \
      org.opencontainers.image.description="ROS Noetic; automatic hardware decode with CPU fallback; amd64/arm64; SDK 3.3.0"
