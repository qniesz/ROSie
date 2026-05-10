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

mkdir -p /home/rosie/slam-online/out

exec docker run --rm $DETACH \
    --name slam_online \
    --network host \
    --memory=240m --memory-swap=600m \
    --env-file /home/rosie/rosie-driver.env \
    -v /home/rosie/slam-online/cyclonedds.xml:/cfg/cyclonedds_no_shm.xml:ro \
    -v /home/rosie/slam-online/out:/out \
    rosie-slam-online
