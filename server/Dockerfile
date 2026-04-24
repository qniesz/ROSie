# ROSie ROS 2 Jazzy Server
# Multi-stage build: build workspace, then slim runtime image
#
# Cache strategy: the builder only copies files needed for compilation
# (rosie_msgs + package manifests). Launch files, configs, URDF, server
# scripts, and entrypoint are overlaid in late runtime layers so edits
# to those files rebuild in seconds, not 20+ minutes.

# ============================================================
# Stage 1: Build (only rosie_msgs + package stubs)
# ============================================================
FROM ros:jazzy AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install paho-mqtt (required by rosie_bridge)
RUN pip3 install --break-system-packages paho-mqtt==1.6.1

# Copy ONLY what needs compilation — launch/config/urdf changes won't
# invalidate this layer.
WORKDIR /ros2_ws
COPY ros2_ws/src/rosie_msgs src/rosie_msgs
COPY ros2_ws/src/rosie_bringup/CMakeLists.txt ros2_ws/src/rosie_bringup/package.xml src/rosie_bringup/
RUN mkdir -p src/rosie_bringup/launch src/rosie_bringup/config src/rosie_bringup/maps src/rosie_bringup/urdf

# Build
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: Runtime
# ============================================================
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages paho-mqtt==1.6.1 Pillow numpy scipy

# Copy built workspace from builder (cached unless msgs/manifests change)
COPY --from=builder /ros2_ws /ros2_ws

# --- Fast layers: overlay frequently-changed files LAST ---

# Bringup assets (launch, config, urdf, maps)
COPY ros2_ws/src/rosie_bringup/launch/ /ros2_ws/install/rosie_bringup/share/rosie_bringup/launch/
COPY ros2_ws/src/rosie_bringup/config/ /ros2_ws/install/rosie_bringup/share/rosie_bringup/config/
COPY ros2_ws/src/rosie_bringup/urdf/   /ros2_ws/install/rosie_bringup/share/rosie_bringup/urdf/
COPY ros2_ws/src/rosie_bringup/maps/   /ros2_ws/install/rosie_bringup/share/rosie_bringup/maps/

# Server scripts (map pipeline + cleanup)
COPY rosie_server.py /ros2_ws/rosie_server.py
COPY clean_map.py /ros2_ws/clean_map.py

# Entrypoint
WORKDIR /ros2_ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/ros2_ws/rosie_server.py"]
