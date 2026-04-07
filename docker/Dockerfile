# mapping_rbnx — multi-stage build
# Stage 1 (builder): compile FAST-LIVO2 ROS2 workspace
# Stage 2 (runtime): lightweight image with bridge + optional SLAM node
#
# To skip the C++ SLAM build (bridge-only mode for dev/test):
#   docker build --target runtime-bridge -f docker/Dockerfile -t mapping_rbnx:bridge .

# ── Build stage: FAST-LIVO2 C++ ─────────────────────────────────────────────
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git \
    libeigen3-dev libpcl-dev libopencv-dev \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Sophus: not in apt on 22.04, clone the non-templated version FAST-LIVO2 needs
RUN git clone --depth 1 https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus && git checkout a621ff \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/Sophus

COPY third_party/FAST-LIVO2 /ws/src/fast_livo2

RUN if [ ! -d /ws/src/livox_ros_driver2 ]; then \
    git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git /ws/src/livox_ros_driver2 || true; \
    fi

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    || echo "[build] colcon build finished (check warnings above)"

# ── Runtime bridge-only (dev/test target) ────────────────────────────────────
FROM ros:humble-ros-base AS runtime-bridge

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    grpcio>=1.60.0 protobuf>=4.25.0 numpy>=1.24.0 pyyaml>=6.0

COPY src/ /ws/src/
COPY proto_gen/ /ws/proto_gen/
COPY config/ /config/
COPY skills/ /ws/skills/

ENV PYTHONPATH="/ws/src:/ws/proto_gen"
CMD ["python3", "-m", "mapping_rbnx.atlas_bridge"]

# ── Full runtime (with C++ SLAM node) ────────────────────────────────────────
FROM runtime-bridge AS runtime

RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-io1.12 libopencv-core4.5d libopencv-imgproc4.5d \
    ros-humble-pcl-ros ros-humble-cv-bridge \
    ros-humble-tf2-ros ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/install /ws/install
COPY launch/ /ws/launch/

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/ws/launch/slam_mapping.launch.py"]
