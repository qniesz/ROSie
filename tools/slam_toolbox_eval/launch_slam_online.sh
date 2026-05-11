#!/bin/bash
# launch_slam_online.sh — start the rosie-slam-online container with the
# right mounts/network/env. Run on the Pi.
#
#   ~/slam-online/launch_slam_online.sh             # foreground
#   ~/slam-online/launch_slam_online.sh -d          # detached
#
# Stop:
#   docker stop slam_online

set -euo pipefail
DETACH=""
[[ "${1:-}" == "-d" ]] && DETACH="-d"

mkdir -p /home/rosie/slam_maps
mkdir -p /home/rosie/slam_online_out

exec docker run --rm $DETACH \
    --name slam_online \
    --network host \
    --memory=300m --memory-swap=1500m \
    --env-file /home/rosie/rosie-driver.env \
    -v /home/rosie/slam_maps:/slam_maps \
    -v /home/rosie/slam_online_out:/out \
    rosie-slam-online
