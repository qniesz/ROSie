#!/bin/bash
set -e

# Source ROS 2 base and the built workspace
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
