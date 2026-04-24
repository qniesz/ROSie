# ROSie Driver — Neato D6 ROS 2 driver node
#
# Minimal image: only what's needed for the hardware driver.
# NO Nav2, NO SLAM, NO Foxglove — those live in rosie-nav.
#
# Rebuild when changing:
#   pi/rosie_driver/*.py  OR  pi/ros2_ws/src/rosie_driver/
#
# Build context is the repo root:
#   docker compose build rosie-driver   (~2 min on Pi 4)

# ============================================================
# Stage 1: Build
# ============================================================
FROM ros:jazzy AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'Acquire::Check-Valid-Until "false"; Acquire::Check-Date "false";' > /etc/apt/apt.conf.d/99no-check-valid-until

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-tf2-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    pyserial \
    psutil \
    RPi.GPIO

WORKDIR /ros2_ws

# rosie_msgs must be built first (driver depends on custom message types)
COPY server/ros2_ws/src/rosie_msgs src/rosie_msgs

# rosie_driver package + hardware modules from pi/rosie_driver/
COPY pi/ros2_ws/src/rosie_driver src/rosie_driver
COPY pi/rosie_driver/serial_handler.py src/rosie_driver/rosie_driver/serial_handler.py
COPY pi/rosie_driver/odometry.py       src/rosie_driver/rosie_driver/odometry.py
COPY pi/rosie_driver/lidar.py          src/rosie_driver/rosie_driver/lidar.py
COPY pi/rosie_driver/sensors.py        src/rosie_driver/rosie_driver/sensors.py
COPY pi/rosie_driver/commands.py       src/rosie_driver/rosie_driver/commands.py
COPY pi/rosie_driver/no_go_guard.py    src/rosie_driver/rosie_driver/no_go_guard.py
COPY pi/rosie_driver/bumper_sensors.py src/rosie_driver/rosie_driver/bumper_sensors.py

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ============================================================
# Stage 2: Runtime
# ============================================================
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'Acquire::Check-Valid-Until "false"; Acquire::Check-Date "false";' > /etc/apt/apt.conf.d/99no-check-valid-until

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-tf2-ros \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN pip3 install --break-system-packages \
    paho-mqtt==1.6.1 \
    pyserial \
    psutil \
    RPi.GPIO

COPY --from=builder /ros2_ws /ros2_ws

RUN mkdir -p /data

WORKDIR /ros2_ws
COPY pi/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "rosie_driver", "driver_node"]
