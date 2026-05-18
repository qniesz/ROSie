#!/bin/bash
# run_eval.sh — entrypoint for slam_toolbox offline evaluation
#
# Mounts:
#   /data/scan.jsonl   input scan log
#   /out/              output dir (writes slam_map.pgm, slam_map.yaml, log.txt)
#
# Env (optional):
#   ROSIE_REPLAY_SPEED   default 5  (x real-time)
#   ROSIE_REPLAY_SETTLE  default 10 (sec to wait for final loop-closure)

set -eo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

OUT=/out
mkdir -p "$OUT"
LOG="$OUT/log.txt"
exec > >(tee -a "$LOG") 2>&1

echo "=== slam_toolbox eval ==="
date
echo "Input:    $(ls -lh /data/scan.jsonl 2>/dev/null || echo MISSING)"
echo "Out:      $OUT"
echo "Speed:    ${ROSIE_REPLAY_SPEED:-5}x"
echo "Settle:   ${ROSIE_REPLAY_SETTLE:-10}s"

# 1) Launch slam_toolbox async node in background (output -> /out/slam.log)
echo "--- launching slam_toolbox ---"
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file /eval/slam_params.yaml \
    > /out/slam.log 2>&1 &
SLAM_PID=$!
trap "echo cleanup; kill $SLAM_PID 2>/dev/null || true" EXIT

# Give slam_toolbox time to come up before we start blasting messages at it
sleep 8
echo "slam_toolbox PID=$SLAM_PID  alive? $(ps -p $SLAM_PID -o stat= 2>/dev/null || echo DEAD)"
echo "--- slam_toolbox boot log (head 40) ---"
head -40 /out/slam.log || true

# slam_toolbox in jazzy is a lifecycle node — must configure + activate
# before it subscribes to /scan or advertises save_map.
echo "--- lifecycle: configure ---"
ros2 lifecycle set /slam_toolbox configure
echo "--- lifecycle: activate ---"
ros2 lifecycle set /slam_toolbox activate
sleep 2
echo "--- post-activate state: $(ros2 lifecycle get /slam_toolbox 2>&1) ---"

echo "--- topics ---"
ros2 topic list 2>&1 | head -30
echo "--- services with slam in name ---"
ros2 service list 2>&1 | grep -i slam || echo "(no slam services)"

# 2) Run replay (blocks until log exhausted + settle period)
echo "--- replaying log ---"
python3 /eval/replay_node.py

# 3) Save map via slam_toolbox /slam_toolbox/save_map service
echo "--- saving map ---"
echo "(post-replay services:)"
ros2 service list 2>&1 | grep -i -E 'slam|map' || true
# slam_toolbox SaveMap service takes std_msgs/String name (no extension).
# Use SIGKILL after 30s — ros2 service call ignores SIGTERM during rclpy
# shutdown and would hang the whole script otherwise.
timeout --kill-after=5 30 ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/out/slam_map'}}" || \
    echo "WARN: save_map service call timed out (map may still have been written)"

# Briefly wait for slam_toolbox to flush /out/slam_map.{pgm,yaml}
for i in 1 2 3 4 5; do
    if [ -f /out/slam_map.pgm ] && [ -f /out/slam_map.yaml ]; then
        break
    fi
    sleep 1
done

# 3b) Save the loop-closed posegraph so localization sessions use this
#     geometrically correct map instead of the drifted online SLAM posegraph.
#     Only written if /slam_maps is mounted (production map_pipeline path).
if [ -d /slam_maps ]; then
    echo "--- saving posegraph to /slam_maps/rosie_home ---"
    SERIALIZE_TIMEOUT=${SERIALIZE_MAP_TIMEOUT_SECS:-90}
    if timeout "${SERIALIZE_TIMEOUT}s" ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '/slam_maps/rosie_home'}"; then
        echo "  eval posegraph saved OK → localization will use loop-closed map"
    else
        echo "  WARNING: eval posegraph save failed or timed out — localization posegraph unchanged"
    fi
else
    echo "--- /slam_maps not mounted, skipping posegraph save ---"
fi

# Skip map_saver_cli to save ~80 MB of RAM on the Pi.
# slam_toolbox's save_map writes /out/slam_map.{pgm,yaml} directly.

echo "--- slam_toolbox log tail ---"
tail -60 /out/slam.log || true

# 4) Memory snapshot
echo "--- final memory ---"
cat /proc/meminfo | head -5
echo "slam_toolbox RSS:"
cat /proc/$SLAM_PID/status 2>/dev/null | grep -E 'VmRSS|VmPeak' || true

ls -lh /out/
echo "=== done ==="
