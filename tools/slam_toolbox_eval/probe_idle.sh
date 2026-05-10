#!/bin/bash
# probe_idle.sh — measure idle RSS of slam_toolbox on the Pi.
# Brings up async_slam_toolbox_node, configures + activates the lifecycle,
# then samples VmRSS once per second for 60 s. No /scan, no /tf — just the
# baseline footprint of an active-but-idle slam_toolbox.

set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u

OUT=/out
mkdir -p "$OUT"
LOG="$OUT/probe.log"
RSS_CSV="$OUT/probe_rss.csv"
exec > >(tee -a "$LOG") 2>&1

echo "=== slam_toolbox idle probe ==="
date
echo "--- system before ---"
free -m
nproc

echo "--- launching slam_toolbox ---"
ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file /eval/slam_params.yaml \
    > "$OUT/slam.log" 2>&1 &
SLAM_PID=$!
trap "echo cleanup; kill $SLAM_PID 2>/dev/null || true" EXIT
echo "PID=$SLAM_PID"
sleep 8
ps -p $SLAM_PID -o stat= 2>/dev/null || { echo "DEAD"; exit 1; }

echo "--- lifecycle: configure + activate ---"
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
sleep 2
echo "state: $(ros2 lifecycle get /slam_toolbox 2>&1)"

echo "--- sampling RSS for 60 s ---"
echo "elapsed_s,vm_rss_kb,vm_size_kb,sys_mem_free_mb,sys_mem_avail_mb" > "$RSS_CSV"
for i in $(seq 1 60); do
  if ! ps -p $SLAM_PID > /dev/null; then
    echo "PROCESS DIED at t=$i"
    break
  fi
  rss=$(awk '/^VmRSS:/ {print $2}' /proc/$SLAM_PID/status)
  vsz=$(awk '/^VmSize:/ {print $2}' /proc/$SLAM_PID/status)
  free_mb=$(awk '/^MemFree:/ {print int($2/1024)}' /proc/meminfo)
  avail_mb=$(awk '/^MemAvailable:/ {print int($2/1024)}' /proc/meminfo)
  echo "$i,$rss,$vsz,$free_mb,$avail_mb" >> "$RSS_CSV"
  sleep 1
done

echo "--- final memory ---"
free -m
echo "--- slam_toolbox proc status ---"
grep -E '^Vm(RSS|Size|Peak|HWM)' /proc/$SLAM_PID/status || true

echo "--- RSS samples (head/tail) ---"
head -5 "$RSS_CSV"
echo "..."
tail -5 "$RSS_CSV"

echo "=== probe done ==="
