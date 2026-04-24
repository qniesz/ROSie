# ROSie Pi 4 — All-In-One Container
# ROS 2 Jazzy + Neato D6 driver + SLAM/Nav2 + Foxglove + map pipeline
#
# Based on ros:jazzy (ARM64 on Pi 4).  Replaces the old Unraid server
# container entirely.  Everything runs on-robot; MQTT is used only for
# HA integration.

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

# paho-mqtt for driver_node + rosie_server; pyserial for Neato serial port;
# pigpio for GPIO no-go guard; Pillow/numpy/scipy for clean_map
RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    pyserial \
    pigpio \
    Pillow \
    numpy \
    scipy \
    psutil

# ── ROS 2 workspace ──────────────────────────────────────────────────────────
# Build order matters: rosie_msgs first (custom message types),
# then rosie_driver (Pi driver node), then rosie_bringup (launch files).
# rosie_bridge package has been removed — its functionality is now in
# driver_node.py.
WORKDIR /ros2_ws

# rosie_msgs (custom message types used by other packages)
COPY server/ros2_ws/src/rosie_msgs src/rosie_msgs

# rosie_bringup (launch files, nav2/slam config, URDF)
COPY server/ros2_ws/src/rosie_bringup src/rosie_bringup

# rosie_driver (Pi-side ROS 2 driver node — new package)
# Note: core hardware modules are copied from pi/rosie_driver/ into the
# package so they are importable as rosie_driver.serial_handler etc.
COPY pi/ros2_ws/src/rosie_driver src/rosie_driver
COPY pi/rosie_driver/serial_handler.py src/rosie_driver/rosie_driver/serial_handler.py
COPY pi/rosie_driver/odometry.py       src/rosie_driver/rosie_driver/odometry.py
COPY pi/rosie_driver/lidar.py          src/rosie_driver/rosie_driver/lidar.py
COPY pi/rosie_driver/sensors.py        src/rosie_driver/rosie_driver/sensors.py
COPY pi/rosie_driver/commands.py       src/rosie_driver/rosie_driver/commands.py
COPY pi/rosie_driver/no_go_guard.py    src/rosie_driver/rosie_driver/no_go_guard.py

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

# Use CycloneDDS to avoid fastcdr ABI mismatch with nav2 typesupport libraries
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# pigpio Python client connects to pigpiod running on the Pi host (localhost:8888).
# pigpio is NOT in Ubuntu 24.04 (Noble) apt repos — install via pip only.
RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    pyserial \
    pigpio \
    Pillow \
    numpy \
    scipy \
    psutil

# Copy built workspace
COPY --from=builder /ros2_ws /ros2_ws

# Copy map pipeline supervisor and map clean script from server/
COPY server/rosie_server.py /ros2_ws/rosie_server.py
COPY server/clean_map.py    /ros2_ws/clean_map.py

# Persistent data dirs created at build time so they exist even without a volume
RUN mkdir -p /ros2_ws/maps /data

WORKDIR /ros2_ws
COPY pi/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# rosie_server.py is the supervisor: it starts the ROS 2 bridge launch,
# then toggles SLAM / Nav2 as needed.
CMD ["python3", "/ros2_ws/rosie_server.py"]
