#!/usr/bin/env python3
"""replay_node.py — feed a recorded scan log into ROS 2 for slam_toolbox eval.

Reads the JSONL produced by pi/rosie_driver/scan_recorder.py and publishes:

  /scan   sensor_msgs/LaserScan   — 360-beam Neato LDS, 1° step, bin 0 = forward
  /odom   nav_msgs/Odometry        — from log odom (x, y, theta)
  /tf     odom -> base_link        — from same odom
  /tf_static  base_link -> laser   — identity (LDS centred on robot)

Pacing: replays at ~5x real-time by default (offline eval, no timing constraint).

When the log is exhausted, sleeps a few seconds (so slam_toolbox processes the
last batch + any pending loop closures), then writes /out/done.flag.

Frame conventions match slam_params.yaml:
    map (slam_toolbox) -> odom (us) -> base_link (us) -> laser (us)
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

LOG_PATH = Path(os.environ.get("ROSIE_REPLAY_LOG", "/data/scan.jsonl"))
DONE_FLAG = Path(os.environ.get("ROSIE_REPLAY_DONE", "/out/done.flag"))
SPEED = float(os.environ.get("ROSIE_REPLAY_SPEED", "5.0"))   # x real-time
SETTLE_SEC = float(os.environ.get("ROSIE_REPLAY_SETTLE", "10.0"))


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Replay(Node):
    def __init__(self):
        super().__init__("rosie_replay")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.tf      = TransformBroadcaster(self)
        self.tf_st   = StaticTransformBroadcaster(self)
        self._publish_static_tf()

    def _publish_static_tf(self):
        # base_link -> laser : identity (Neato LDS is centred on the robot)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.rotation.w = 1.0
        self.tf_st.sendTransform(t)

    def publish_row(self, row: dict, stamp):
        scan = row.get("scan") or {}
        ranges_in = scan.get("ranges") or []
        # 360 beams, 0..359 deg CCW, bin 0 = forward
        ranges = [float("inf") if (r is None or not math.isfinite(r) or r <= 0)
                  else float(r) for r in ranges_in]

        ls = LaserScan()
        ls.header.stamp = stamp
        ls.header.frame_id = "laser"
        ls.angle_min = 0.0
        ls.angle_max = math.radians(359.0)
        ls.angle_increment = math.radians(1.0)
        ls.time_increment = 0.0
        ls.scan_time = 0.2
        ls.range_min = 0.05
        ls.range_max = 5.0
        ls.ranges = ranges
        ls.intensities = []  # not needed for slam
        self.scan_pub.publish(ls)

        odom = row.get("odom")
        if not odom:
            return
        x, y, th = float(odom["x"]), float(odom["y"]), float(odom["theta"])
        qx, qy, qz, qw = yaw_to_quat(th)

        # Odometry msg
        om = Odometry()
        om.header.stamp = stamp
        om.header.frame_id = "odom"
        om.child_frame_id  = "base_link"
        om.pose.pose.position.x = x
        om.pose.pose.position.y = y
        om.pose.pose.orientation.x = qx
        om.pose.pose.orientation.y = qy
        om.pose.pose.orientation.z = qz
        om.pose.pose.orientation.w = qw
        om.twist.twist.linear.x  = float(odom.get("lin", 0.0))
        om.twist.twist.angular.z = float(odom.get("ang", 0.0))
        self.odom_pub.publish(om)

        # tf odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    if not LOG_PATH.exists():
        print(f"replay: log not found: {LOG_PATH}", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = Replay()

    rows = []
    with LOG_PATH.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    if not rows:
        print("replay: empty log", file=sys.stderr)
        sys.exit(3)

    t0_log  = float(rows[0].get("t", 0.0))
    t0_real = time.monotonic()
    print(f"replay: {len(rows)} rows, speed={SPEED}x", flush=True)

    accepted = 0
    for i, row in enumerate(rows):
        if not rclpy.ok():
            break
        # Pace
        target_real = t0_real + (float(row.get("t", t0_log)) - t0_log) / SPEED
        delay = target_real - time.monotonic()
        if delay > 0:
            time.sleep(delay)

        # Use ROS clock so slam_toolbox sees consistent timestamps.
        stamp = node.get_clock().now().to_msg()
        node.publish_row(row, stamp)
        rclpy.spin_once(node, timeout_sec=0.0)
        accepted += 1
        if accepted % 200 == 0:
            print(f"replay: {accepted}/{len(rows)}", flush=True)

    print(f"replay: published {accepted} rows; settling for {SETTLE_SEC} s", flush=True)
    # Let slam_toolbox finish processing + run final loop closures.
    settle_end = time.monotonic() + SETTLE_SEC
    while rclpy.ok() and time.monotonic() < settle_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    DONE_FLAG.parent.mkdir(parents=True, exist_ok=True)
    DONE_FLAG.write_text("done\n")
    print(f"replay: wrote {DONE_FLAG}", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
