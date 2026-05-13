#!/bin/bash
# run_online.sh — start slam_toolbox + MQTT bridge inside the container.
#
# Design:
#   - NO ros2 CLI calls during the first LIFECYCLE_WAIT_SECS seconds.
#     Repeated DDS participant churn from ros2 CLI (lifecycle get, topic hz)
#     blocks slam_toolbox from completing its own DDS initialization on the
#     Pi Zero 2 W.
#   - Lifecycle configure+activate is attempted once per LIFECYCLE_INTERVAL
#     seconds after the initial silence window.
#   - Monitoring loop uses only /proc/$PID/status (no ros2 CLI).

set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

OUT=/out
mkdir -p "$OUT"
exec > >(tee -a "$OUT/online.log") 2>&1

echo "=== rosie online slam_toolbox ==="
date
free -m

# Thread count: Orange Pi Zero 2W (Armbian, 1.5 GB) gets all 4 cores;
# Raspberry Pi Zero 2W (416 MB) is constrained to 1 to save ~30 MB RAM.
# Can be overridden via ROSIE_SLAM_OMP_THREADS env var.
_board=$(cat /proc/device-tree/model 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "unknown")
if echo "$_board" | grep -q "orange"; then
    _default_threads=4
else
    _default_threads=1
fi
_threads=${ROSIE_SLAM_OMP_THREADS:-$_default_threads}
export OMP_NUM_THREADS=$_threads
export OPENBLAS_NUM_THREADS=$_threads
export MKL_NUM_THREADS=$_threads
unset EIGEN_DONT_PARALLELIZE  # let Eigen follow OMP_NUM_THREADS

# CycloneDDS — loopback only, no multicast, unicast peer on 127.0.0.1.
# The image's /cfg/cyclonedds_no_shm.xml uses autodetermine="true" which
# causes CycloneDDS to attempt multicast discovery on Docker bridge eth0
# and hang indefinitely in futex_wait_queue waiting for multicast join.
# Writing a new config here avoids that hang without an image rebuild.
cat > /tmp/cyclone_lo.xml << 'CYCLONE_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <SharedMemory><Enable>false</Enable></SharedMemory>
    <General>
      <Interfaces>
        <NetworkInterface name="lo"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="127.0.0.1"/>
      </Peers>
    </Discovery>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
CYCLONE_EOF
export CYCLONEDDS_URI="file:///tmp/cyclone_lo.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SLAM_MAPS_DIR=${SLAM_MAPS_DIR:-/slam_maps}
SLAM_MAP_NAME=${SLAM_MAP_NAME:-rosie_home}

# Auto-select mode: if a serialized posegraph exists use localization, else mapping.
# PARAMS env can override completely (e.g. force mapping for a remap cycle).
if [ -z "${PARAMS:-}" ]; then
    if [ -f "${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}.posegraph" ]; then
        echo "=== saved map found — using LOCALIZATION mode ==="
        PARAMS=/eval/slam_params_localization.yaml
        SLAM_MODE=localization
    else
        echo "=== no saved map — using MAPPING mode ==="
        PARAMS=/eval/slam_params_online.yaml
        SLAM_MODE=mapping
    fi
else
    SLAM_MODE=unknown
fi

POSEGRAPH_PATH="${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}.posegraph"
DATA_PATH="${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}.data"
if [ -f "$POSEGRAPH_PATH" ] && [ -f "$DATA_PATH" ]; then
    posegraph_stat=$(stat -c '%Y-%s' "$POSEGRAPH_PATH" 2>/dev/null || echo "unknown")
    data_stat=$(stat -c '%Y-%s' "$DATA_PATH" 2>/dev/null || echo "unknown")
    SLAM_MAP_ID="${SLAM_MAP_NAME}:${posegraph_stat}:${data_stat}"
    SLAM_HAS_POSEGRAPH=1
else
    SLAM_MAP_ID=""
    SLAM_HAS_POSEGRAPH=0
fi
export SLAM_MODE SLAM_MAP_NAME SLAM_MAP_ID SLAM_HAS_POSEGRAPH
cat > "$OUT/mode.json" << EOF
{"mode":"${SLAM_MODE}","map_name":"${SLAM_MAP_NAME}","map_id":"${SLAM_MAP_ID}","has_posegraph":${SLAM_HAS_POSEGRAPH}}
EOF

# async node: queues scans for background processing; may process a scan against
# slightly stale odom if the Pi is under load.  With minimum_travel_distance=0.20
# scans are sparse enough that the queue stays short and stale-odom drift is minimal.
# The sync node (sync_slam_toolbox_node) was tried but consistently crashes with
# "unable to get pointer in probability search" during onActivate() in mapping mode
# due to a CorrelationGrid initialization race condition — do not use it.
SLAM_NODE=${SLAM_NODE:-async_slam_toolbox_node}

# How long (seconds) to wait silently before the first lifecycle attempt.
# CycloneDDS loopback-only config (no multicast) stabilises in ~5 s on the
# Pi Zero 2 W so 30 s is plenty.  Old 300 s value was needed before the
# loopback fix; keeping it as an env override for emergencies.
LIFECYCLE_WAIT_SECS=${LIFECYCLE_WAIT_SECS:-30}
# Gap between retries if configure/activate fails.
LIFECYCLE_INTERVAL=${LIFECYCLE_INTERVAL:-30}
# Maximum number of lifecycle attempts before giving up. 0 means keep trying.
LIFECYCLE_MAX_ATTEMPTS=${LIFECYCLE_MAX_ATTEMPTS:-0}

SAVE_STATUS_PATH="${OUT}/save_status.json"

last_scan_count() {
    grep -oE 'scan=[0-9]+' "$OUT/bridge.log" 2>/dev/null \
        | tail -1 | cut -d= -f2
}

write_save_status() {
    local state="${1:-unknown}"
    local detail="${2:-}"
    local scan_count="${3:-0}"
    SAVE_STATE="$state" \
    SAVE_DETAIL="$detail" \
    SAVE_SCAN_COUNT="$scan_count" \
    SAVE_STATUS_PATH="$SAVE_STATUS_PATH" \
    SLAM_MAPS_DIR="$SLAM_MAPS_DIR" \
    SLAM_MAP_NAME="$SLAM_MAP_NAME" \
    python3 - <<'PY'
import json
import os
import time
from pathlib import Path

root = Path(os.environ["SLAM_MAPS_DIR"])
name = os.environ["SLAM_MAP_NAME"]
artifacts = {}
for artifact_name, path in {
    "posegraph": root / f"{name}.posegraph",
    "data": root / f"{name}.data",
    "pgm": root / "home.pgm",
    "yaml": root / "home.yaml",
}.items():
    if path.exists():
        stat = path.stat()
        artifacts[artifact_name] = {"path": str(path), "size": stat.st_size, "mtime": stat.st_mtime}
    else:
        artifacts[artifact_name] = {"path": str(path), "missing": True}

payload = {
    "state": os.environ["SAVE_STATE"],
    "detail": os.environ.get("SAVE_DETAIL", ""),
    "scan_count": int(os.environ.get("SAVE_SCAN_COUNT") or 0),
    "timestamp": time.time(),
    "artifacts": artifacts,
}
Path(os.environ["SAVE_STATUS_PATH"]).write_text(json.dumps(payload, indent=2))
PY
}

write_save_status "running" "sidecar started" "0"

# 1) Bridge first so /scan publisher exists when slam_toolbox subscribes
echo "--- starting MQTT<->ROS bridge ---"
python3 /eval/mqtt_ros_bridge.py > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!

# 2) slam_toolbox
echo "--- starting slam_toolbox ($SLAM_NODE) ---"
ros2 run slam_toolbox "$SLAM_NODE" \
    --ros-args --params-file "$PARAMS" \
    --log-level DEBUG \
    > "$OUT/slam.log" 2>&1 &
SLAM_PID=$!

# 2b) Lifecycle activator — uses rclpy (persistent DDS participant, no CLI churn).
# The ros2 CLI --no-daemon approach creates a new DDS participant on every call
# which blocks slam_toolbox from completing DDS initialization on Pi Zero 2 W.
# lifecycle_activate.py waits for the slam_toolbox lifecycle services, then
# configure→activates and (in localization mode) seeds the initial pose.
LIFECYCLE_STATUS_FILE="$OUT/lifecycle_status.txt"
LIFECYCLE_ACTIVATOR_PID=""
SEED_POSE_ARGS=""
if [ "$SLAM_MODE" = "localization" ]; then
    SEED_POSE_ARGS="--seed-pose ${INITIAL_POSE_X:-0.0} ${INITIAL_POSE_Y:-0.0} ${INITIAL_POSE_THETA:-0.0}"
fi
echo "--- starting lifecycle activator (mode=$SLAM_MODE) ---"
# shellcheck disable=SC2086
python3 /eval/lifecycle_activate.py \
    --status-file "$LIFECYCLE_STATUS_FILE" \
    --slam-mode "$SLAM_MODE" \
    $SEED_POSE_ARGS \
    --timeout 300 \
    > "$OUT/lifecycle.log" 2>&1 &
LIFECYCLE_ACTIVATOR_PID=$!

# 3) Optional: Foxglove WebSocket bridge (enabled by ROSIE_FOXGLOVE=1)
# Connect Foxglove Studio to ws://<pi-ip>:8765 to visualise /scan, /map, /pose,
# and the Robot Model display from /robot_description.
FOXGLOVE_PID=""
ROBOT_STATE_PUBLISHER_PID=""
VAC_MARKER_PID=""
ROBOT_URDF=${ROSIE_ROBOT_URDF:-/eval/robot/rosie_foxglove.urdf}
FOXGLOVE_TOPIC_WHITELIST=${ROSIE_FOXGLOVE_TOPIC_WHITELIST:-"['^/(tf|tf_static|map|map_metadata|scan|rosie_vac_marker)$']"}
if [ "${ROSIE_FOXGLOVE:-0}" = "1" ]; then
    if [ -f "$ROBOT_URDF" ]; then
        echo "--- starting robot_state_publisher ($ROBOT_URDF) ---"
        ros2 run robot_state_publisher robot_state_publisher "$ROBOT_URDF" \
            > "$OUT/robot_state_publisher.log" 2>&1 &
        ROBOT_STATE_PUBLISHER_PID=$!
    else
        echo "WARNING: robot URDF not found at $ROBOT_URDF; Foxglove Robot Model will not show the vac image"
    fi

    echo "--- starting foxglove_bridge (ws://0.0.0.0:8765) ---"
    ros2 run foxglove_bridge foxglove_bridge \
        --ros-args -p port:=8765 -p address:=0.0.0.0 \
        -p topic_whitelist:="$FOXGLOVE_TOPIC_WHITELIST" \
        > "$OUT/foxglove.log" 2>&1 &
    FOXGLOVE_PID=$!

    if [ -f /eval/rosie_vac_marker.py ]; then
        echo "--- starting Foxglove vac marker (/rosie_vac_marker) ---"
        python3 /eval/rosie_vac_marker.py > "$OUT/vac_marker.log" 2>&1 &
        VAC_MARKER_PID=$!
    fi
fi

save_map() {
    # Serialize the posegraph to disk so the next run uses localization mode.
    # Only save when we were in mapping mode and actually processed some scans.
    if [ "$SLAM_MODE" != "mapping" ]; then
        echo "--- skip map save (mode=${SLAM_MODE}) ---"
        write_save_status "skipped" "mode=${SLAM_MODE}" "$(last_scan_count || echo 0)"
        return
    fi
    if [ "$CONFIGURED" != "1" ]; then
        echo "--- skip map save (slam never activated) ---"
        write_save_status "failed" "slam never activated" "$(last_scan_count || echo 0)"
        return 1
    fi
    # Safety guard: refuse to overwrite an existing map with a result that
    # received too few scans (e.g. broker bounced mid-cycle and bridge was
    # disconnected for most of the run).  Threshold = 100 scans
    # (~30 s at 3-5 Hz).  Override with MIN_SAVE_SCANS=0 for forced saves.
    MIN_SAVE_SCANS=${MIN_SAVE_SCANS:-100}
    last_scan=$(last_scan_count || echo 0)
    last_scan=${last_scan:-0}
    if [ "$last_scan" -lt "$MIN_SAVE_SCANS" ]; then
        echo "--- skip map save: only ${last_scan} scans received (< ${MIN_SAVE_SCANS}); refusing to clobber existing map ---"
        write_save_status "failed" "only ${last_scan} scans received (< ${MIN_SAVE_SCANS})" "$last_scan"
        return 1
    fi
    echo "--- save_map: bridge processed ${last_scan} scans ---"
    mkdir -p "$SLAM_MAPS_DIR"
    MAP_PATH="${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}"
    echo "--- saving posegraph to ${MAP_PATH} ---"
    SERIALIZE_TIMEOUT=${SERIALIZE_MAP_TIMEOUT_SECS:-90}
    MAP_SAVER_TIMEOUT=${MAP_SAVER_TIMEOUT_SECS:-120}
    # serialize_map service: saves .posegraph + .data files
    posegraph_ok=0
    if timeout "${SERIALIZE_TIMEOUT}s" ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '${MAP_PATH}'}"; then
        echo "  posegraph saved OK"
        posegraph_ok=1
    else
        echo "  WARNING: posegraph save failed or timed out after ${SERIALIZE_TIMEOUT}s"
    fi

    # Save occupancy grid as home.pgm + home.yaml for clean_map.py
    # map_saver_cli subscribes to /map (published by slam_toolbox) and
    # writes a nav2-format PGM that clean_map.py already understands.
    echo "--- saving pgm to ${SLAM_MAPS_DIR}/home ---"
    pgm_ok=0
    if timeout "${MAP_SAVER_TIMEOUT}s" ros2 run nav2_map_server map_saver_cli \
        -f "${SLAM_MAPS_DIR}/home" \
        --ros-args -p save_map_timeout:=10000.0; then
        echo "  pgm saved OK"
        pgm_ok=1
    else
        echo "  WARNING: pgm save failed or timed out after ${MAP_SAVER_TIMEOUT}s"
    fi

    missing=()
    for required in \
        "${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}.posegraph" \
        "${SLAM_MAPS_DIR}/${SLAM_MAP_NAME}.data" \
        "${SLAM_MAPS_DIR}/home.pgm" \
        "${SLAM_MAPS_DIR}/home.yaml"; do
        if [ ! -s "$required" ]; then
            missing+=("$required")
        fi
    done

    if [ "$posegraph_ok" != "1" ] || [ "$pgm_ok" != "1" ] || [ "${#missing[@]}" -gt 0 ]; then
        detail="posegraph_ok=${posegraph_ok} pgm_ok=${pgm_ok} missing=${missing[*]:-none}"
        echo "  ERROR: map save incomplete: ${detail}"
        write_save_status "failed" "$detail" "$last_scan"
        return 1
    fi

    write_save_status "success" "map save complete" "$last_scan"
}

CLEANED_UP=0
SAVE_RC=0

cleanup() {
    if [ "${CLEANED_UP:-0}" = "1" ]; then
        return "${SAVE_RC:-0}"
    fi
    CLEANED_UP=1
    echo "--- cleanup ---"
    SAVE_RC=0
    save_map || SAVE_RC=$?
    kill -INT "$SLAM_PID" "$BRIDGE_PID" ${FOXGLOVE_PID:+"$FOXGLOVE_PID"} ${ROBOT_STATE_PUBLISHER_PID:+"$ROBOT_STATE_PUBLISHER_PID"} ${VAC_MARKER_PID:+"$VAC_MARKER_PID"} ${LIFECYCLE_ACTIVATOR_PID:+"$LIFECYCLE_ACTIVATOR_PID"} 2>/dev/null || true
    # Also kill by name — guards against PID reuse where the monitored $SLAM_PID
    # gets recycled to a different process and the real slam_toolbox child
    # escapes the kill above, surviving container cleanup.
    pkill -INT -x sync_slam_toolbox_node 2>/dev/null || true
    pkill -INT -f "mqtt_ros_bridge" 2>/dev/null || true
    # Wait each separately so SIGKILL can be applied if needed
    wait "$SLAM_PID" 2>/dev/null || true
    wait "$BRIDGE_PID" 2>/dev/null || true
    [ -n "$FOXGLOVE_PID" ] && wait "$FOXGLOVE_PID" 2>/dev/null || true
    [ -n "$ROBOT_STATE_PUBLISHER_PID" ] && wait "$ROBOT_STATE_PUBLISHER_PID" 2>/dev/null || true
    [ -n "$VAC_MARKER_PID" ] && wait "$VAC_MARKER_PID" 2>/dev/null || true
    [ -n "$LIFECYCLE_ACTIVATOR_PID" ] && wait "$LIFECYCLE_ACTIVATOR_PID" 2>/dev/null || true
    return "$SAVE_RC"
}

handle_shutdown_signal() {
    cleanup
    exit "$?"
}

trap cleanup EXIT
trap handle_shutdown_signal INT TERM

# Seed slam_toolbox with a known starting pose (localization mode only).
# Without this, slam_toolbox hunts for its initial position on the saved map
# for the first few seconds, producing visible bouncing in Foxglove.
# Defaults to dock position (0,0,0) — the robot always starts at the dock.
seed_initial_pose() {
    [ "$SLAM_MODE" = "localization" ] || return 0
    local px=${INITIAL_POSE_X:-0.0}
    local py=${INITIAL_POSE_Y:-0.0}
    local pth=${INITIAL_POSE_THETA:-0.0}
    local qz qw
    qz=$(awk -v th="$pth" 'BEGIN { printf "%.17g", sin(th / 2.0) }')
    qw=$(awk -v th="$pth" 'BEGIN { printf "%.17g", cos(th / 2.0) }')
    echo "--- seeding initial pose: x=${px} y=${py} th=${pth} (qz=${qz} qw=${qw}) ---"
    timeout 10s ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, \
pose: {pose: {position: {x: ${px}, y: ${py}, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: ${qz}, w: ${qw}}}, \
covariance: [0.0025,0.0,0.0,0.0,0.0,0.0, \
             0.0,0.0025,0.0,0.0,0.0,0.0, \
             0.0,0.0,0.0,0.0,0.0,0.0, \
             0.0,0.0,0.0,0.0,0.0,0.0, \
             0.0,0.0,0.0,0.0,0.0,0.0, \
             0.0,0.0,0.0,0.0,0.0,0.0076]}}" \
        2>/dev/null \
        && echo "  initial pose seeded OK" \
        || echo "  WARNING: initial pose seed failed (non-fatal)"
}



ELAPSED=0
LIFECYCLE_ATTEMPT=0
CONFIGURED=0
HEARTBEAT=60   # log every 60 s without ros2 CLI

while true; do
    sleep 10
    ELAPSED=$((ELAPSED + 10))

    # ---- health checks (no ros2 CLI) ----
    if ! ps -p "$SLAM_PID" > /dev/null 2>&1; then
        echo "!! slam_toolbox DIED at t=${ELAPSED}s"
        break
    fi
    if ! ps -p "$BRIDGE_PID" > /dev/null 2>&1; then
        echo "!! bridge DIED at t=${ELAPSED}s"
        break
    fi
    if [ -n "$FOXGLOVE_PID" ] && ! ps -p "$FOXGLOVE_PID" > /dev/null 2>&1; then
        echo "!! foxglove_bridge DIED at t=${ELAPSED}s — restarting"
        ros2 run foxglove_bridge foxglove_bridge \
            --ros-args -p port:=8765 -p address:=0.0.0.0 \
            -p topic_whitelist:="$FOXGLOVE_TOPIC_WHITELIST" \
            > "$OUT/foxglove.log" 2>&1 &
        FOXGLOVE_PID=$!
    fi
    if [ -n "$ROBOT_STATE_PUBLISHER_PID" ] && ! ps -p "$ROBOT_STATE_PUBLISHER_PID" > /dev/null 2>&1; then
        echo "!! robot_state_publisher DIED at t=${ELAPSED}s — restarting"
        ros2 run robot_state_publisher robot_state_publisher "$ROBOT_URDF" \
            > "$OUT/robot_state_publisher.log" 2>&1 &
        ROBOT_STATE_PUBLISHER_PID=$!
    fi
    if [ -n "$VAC_MARKER_PID" ] && ! ps -p "$VAC_MARKER_PID" > /dev/null 2>&1; then
        echo "!! vac marker publisher DIED at t=${ELAPSED}s — restarting"
        python3 /eval/rosie_vac_marker.py > "$OUT/vac_marker.log" 2>&1 &
        VAC_MARKER_PID=$!
    fi
    # ---- heartbeat every HEARTBEAT seconds ----
    if [ $((ELAPSED % HEARTBEAT)) -eq 0 ]; then
        rss_slam=$(awk '/^VmRSS:/ {print $2}' /proc/$SLAM_PID/status 2>/dev/null || echo 0)
        rss_brg=$(awk '/^VmRSS:/ {print $2}' /proc/$BRIDGE_PID/status 2>/dev/null || echo 0)
        rss_fox=$([ -n "$FOXGLOVE_PID" ] && awk '/^VmRSS:/ {print $2}' /proc/$FOXGLOVE_PID/status 2>/dev/null || echo 0)
        rss_rsp=$([ -n "$ROBOT_STATE_PUBLISHER_PID" ] && awk '/^VmRSS:/ {print $2}' /proc/$ROBOT_STATE_PUBLISHER_PID/status 2>/dev/null || echo 0)
        rss_marker=$([ -n "$VAC_MARKER_PID" ] && awk '/^VmRSS:/ {print $2}' /proc/$VAC_MARKER_PID/status 2>/dev/null || echo 0)
        free_mb=$(awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo)
        swap_free=$(awk '/^SwapFree:/ {print int($2/1024)}' /proc/meminfo)
        echo "t=${ELAPSED}s  slam=${rss_slam}KB  bridge=${rss_brg}KB  fox=${rss_fox}KB  rsp=${rss_rsp}KB  marker=${rss_marker}KB  avail=${free_mb}MB  swap_free=${swap_free}MB"
    fi

    # ---- lifecycle: check if lifecycle_activate.py has succeeded ----
    if [ "$CONFIGURED" = "0" ] && [ -f "$LIFECYCLE_STATUS_FILE" ]; then
        lc_status=$(cat "$LIFECYCLE_STATUS_FILE" 2>/dev/null || echo "")
        if [ "$lc_status" = "active" ]; then
            echo "--- slam_toolbox ACTIVE (t=${ELAPSED}s, via lifecycle activator) ---"
            CONFIGURED=1
        elif echo "$lc_status" | grep -q "^failed"; then
            echo "--- lifecycle activator reported: $lc_status ---"
            # Status file consumed; activator has already exited — no retry
        fi
    fi
done

echo "--- final state ---"
free -m
echo "--- last 30 slam log ---"
tail -30 "$OUT/slam.log" || true
echo "--- last 30 bridge log ---"
tail -30 "$OUT/bridge.log" || true
cleanup
exit "$?"
