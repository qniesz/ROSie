# ROSie Nav — Supervisor + Nav2/SLAM/Foxglove stack
#
# Runs rosie_server.py which manages:
#   - robot_state_publisher + Foxglove bridge (always via rosie_bringup.launch.py)
#   - Nav2 stack (default, for autonomous navigation)
#   - SLAM Toolbox (during create_map pipeline)
#   - HA MQTT discovery + map pipeline + diagnostics
#
# The Neato driver lives in the rosie-driver container — NOT here.
#
# Rebuild when changing:
#   server/ros2_ws/src/rosie_bringup/**  (launch files, nav params, URDF)
#
# Python-only changes to rosie_server.py / clean_map.py:
#   docker compose restart rosie-nav   ← NO rebuild needed (bind-mounted)
#
# Build context is the repo root:
#   docker compose build rosie-nav   (~3-4 min on Pi 4)

# ============================================================
# Stage 1: Build
# ============================================================
FROM ros:jazzy AS builder

ENV DEBIAN_FRONTEND=noninteractive

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
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    Pillow \
    numpy \
    scipy \
    psutil

WORKDIR /ros2_ws

COPY server/ros2_ws/src/rosie_msgs src/rosie_msgs
# Only copy package manifests for rosie_bringup — launch/config/urdf go in
# a late runtime layer so edits rebuild in seconds, not minutes.
COPY server/ros2_ws/src/rosie_bringup/CMakeLists.txt server/ros2_ws/src/rosie_bringup/package.xml src/rosie_bringup/
RUN mkdir -p src/rosie_bringup/launch src/rosie_bringup/config src/rosie_bringup/maps src/rosie_bringup/urdf

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

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
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    Pillow \
    numpy \
    scipy \
    psutil

COPY --from=builder /ros2_ws /ros2_ws

# --- Fast layers: overlay frequently-changed bringup assets LAST ---
COPY server/ros2_ws/src/rosie_bringup/launch/ /ros2_ws/install/rosie_bringup/share/rosie_bringup/launch/
COPY server/ros2_ws/src/rosie_bringup/config/ /ros2_ws/install/rosie_bringup/share/rosie_bringup/config/
COPY server/ros2_ws/src/rosie_bringup/urdf/   /ros2_ws/install/rosie_bringup/share/rosie_bringup/urdf/
COPY server/ros2_ws/src/rosie_bringup/maps/   /ros2_ws/install/rosie_bringup/share/rosie_bringup/maps/

# Baked-in copies as fallback (bind mounts in compose override these at runtime)
COPY server/rosie_server.py /ros2_ws/rosie_server.py
COPY server/clean_map.py    /ros2_ws/clean_map.py

RUN mkdir -p /ros2_ws/maps /data

WORKDIR /ros2_ws
COPY pi/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/ros2_ws/rosie_server.py"]
