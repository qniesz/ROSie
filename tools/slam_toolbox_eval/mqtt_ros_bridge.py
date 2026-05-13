#!/usr/bin/env python3
"""
mqtt_ros_bridge.py — bidirectional bridge between Pi MQTT topics and ROS 2.

Designed to run inside the slam_toolbox container on the Pi (or server).
Subscribes (existing Pi driver topics):
    MQTT  rosie/scan   →  publishes ROS  /scan   (sensor_msgs/LaserScan)
    MQTT  rosie/odom   →  publishes TF   odom -> base_link
                            + static TF  base_link -> laser_link
Subscribes ROS map -> base_link transform (after slam_toolbox composes
map -> odom -> base_link), publishes back to MQTT  rosie/pose_slam
at 5 Hz.

MQTT scan payload (existing rosie/scan format):
    {"ranges": [...360 floats; 0 = invalid],
     "intensities": [...], "angle_min": ..., "angle_max": ...,
     "angle_increment": ..., "range_min": ..., "range_max": ...,
     "rpm": ..., "stamp": ...}

MQTT odom payload (existing rosie/odom format):
    {"x": 1.2, "y": 0.3, "theta": 1.57,
     "linear_vel": ..., "angular_vel": ..., "stamp": ...}

MQTT pose payload (published by this bridge to existing rosie/pose contract):
    {"t": 12345.678, "x": 1.2, "y": 0.3, "theta": 1.57, "src": "slam_toolbox"}

Env:
    MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_PREFIX (default rosie)
    BRIDGE_LOG_LEVEL (default INFO)
    BRIDGE_POSE_RATE_HZ (default 5)
"""
from __future__ import annotations
import json
import logging
import math
import os
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


LOG = logging.getLogger("mqtt_ros_bridge")


def wall_to_time_msg(t: float) -> TimeMsg:
    """Convert a time.time() wall-clock float to a ROS builtin_interfaces/Time."""
    msg = TimeMsg()
    msg.sec = int(t)
    msg.nanosec = int((t % 1.0) * 1_000_000_000)
    return msg


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class BridgeNode(Node):
    def __init__(self):
        super().__init__("rosie_mqtt_bridge")

        self.prefix = os.environ.get("MQTT_PREFIX", "rosie")
        self.pose_rate = float(os.environ.get("BRIDGE_POSE_RATE_HZ", "5"))
        self.slam_mode = os.environ.get("SLAM_MODE", "unknown")
        self.slam_map_name = os.environ.get("SLAM_MAP_NAME", "")
        self.slam_map_id = os.environ.get("SLAM_MAP_ID", "")
        # When rf2o is active, the bridge hands off odom→base_link TF ownership
        # to rf2o_laser_odometry_node once scans start flowing.  During the
        # initial scan holdoff window the bridge still publishes wheel-encoder
        # TF so slam_toolbox has a valid chain at activation time.
        self._rf2o_mode = os.environ.get("ROSIE_RF2O", "0") == "1"

        # ── ROS publishers / TF ──────────────────────────────────────
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.scan_pub = self.create_publisher(LaserScan, "scan", scan_qos)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._last_odom_stamp = None  # ROS stamp of the most recent odom TF
        self._last_odom_transform = None  # geometry_msgs/Transform of last odom (cached)
        self._odom_origin: tuple[float, float, float] | None = None  # first msg locked as (0,0,0)
        # Hold off scan publishing for a brief window after node start so that
        # slam_toolbox has time to fully activate before it receives the first
        # scan.  If scans arrive before slam_toolbox's internal CorrelationGrid
        # is initialised it crashes with "unable to get pointer in probability
        # search".  Typical slam activation on Pi Zero 2W is ~5-10s.
        _holdoff_secs = float(os.environ.get("BRIDGE_SCAN_HOLDOFF_SECS", "15"))
        self._scan_ready_time = time.time() + _holdoff_secs  # wall-clock ready time

        # static base_link -> laser_link
        # LDS is at the back-centre of the 13 in (330 mm) robot, so the laser
        # sits 165 mm behind base_link's centre (negative X).  Without this
        # offset, rotation in place sweeps an arc that scan-matching reads
        # as translation, causing pose jumps on every turn.
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "base_link"
        st.child_frame_id = "laser_link"
        st.transform.translation.x = -0.100  # 100 mm aft (measured: axle centre to LDS turret centre)
        st.transform.rotation.w = 1.0
        self.static_br.sendTransform(st)

        # ── ROS /map subscriber (for live HA preview during mapping) ──
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self._last_map_pub: float = 0.0

        # ── ROS TF listener for map -> base_link (slam_toolbox output) ──
        # 2 s cache (default is 10 s) — at 5 Hz publish rate this is plenty
        self.tf_buf = Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = TransformListener(self.tf_buf, self)

        # ── MQTT ─────────────────────────────────────────────────────
        host = os.environ["MQTT_HOST"]
        port = int(os.environ.get("MQTT_PORT", "1883"))
        user = os.environ["MQTT_USER"]
        pw = os.environ["MQTT_PASS"]

        self.mq = mqtt.Client(client_id=f"rosie_bridge_{os.getpid()}",
                              clean_session=True)
        self.mq.username_pw_set(user, pw)
        self.mq.on_connect = self._on_mqtt_connect
        self.mq.on_message = self._on_mqtt_message

        # Retry initial connect — WiFi flaps on the Pi can cause a one-shot
        # connect() to time out, which would crash the bridge and prevent the
        # slam container from activating.
        _max_attempts = int(os.environ.get("MQTT_CONNECT_RETRIES", "10"))
        _retry_delay = 5  # seconds between attempts
        for _attempt in range(1, _max_attempts + 1):
            try:
                self.mq.connect(host, port, keepalive=30)
                break
            except (OSError, TimeoutError) as _e:
                if _attempt >= _max_attempts:
                    raise RuntimeError(
                        f"MQTT connect to {host}:{port} failed after "
                        f"{_max_attempts} attempts: {_e}"
                    ) from _e
                logging.getLogger(__name__).warning(
                    "MQTT connect attempt %d/%d failed (%s), retrying in %ds…",
                    _attempt, _max_attempts, _e, _retry_delay,
                )
                time.sleep(_retry_delay)

        self.mq.loop_start()

        # Pose publishing timer
        self.create_timer(1.0 / self.pose_rate, self._tick_pose)

        # Stats
        self.scan_count = 0
        self.odom_count = 0
        self.pose_count = 0
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f"Bridge up: MQTT={host}:{port} prefix={self.prefix} "
            f"pose_rate={self.pose_rate}Hz"
        )

    # ── MQTT side ────────────────────────────────────────────────────
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        topics = [
            (f"{self.prefix}/scan", 0),
            (f"{self.prefix}/odom", 0),
        ]
        client.subscribe(topics)
        self.get_logger().info(f"MQTT connected rc={rc}; subscribed: {topics}")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload)
        except Exception as e:
            self.get_logger().warning(f"bad payload on {msg.topic}: {e}")
            return
        if msg.topic.endswith("/scan"):
            self._handle_scan(payload)
        elif msg.topic.endswith("/odom"):
            self._handle_odom(payload)

    def _handle_scan(self, p: dict):
        # Hold off until slam_toolbox has had time to activate.
        if time.time() < self._scan_ready_time:
            return
        try:
            ranges = p["ranges"]
            ls = LaserScan()
            # Use the wall-clock stamp from the Pi driver (time.time() at capture).
            # This preserves the true capture time so slam_toolbox's motion model
            # is correct regardless of MQTT delivery jitter.
            raw_stamp = p.get("stamp")
            ls.header.stamp = (
                wall_to_time_msg(float(raw_stamp))
                if raw_stamp is not None
                else self.get_clock().now().to_msg()
            )
            ls.header.frame_id = "laser_link"
            ls.angle_min = float(p.get("angle_min", 0.0))
            ls.angle_increment = float(
                p.get("angle_increment", math.pi / 180.0))
            ls.angle_max = ls.angle_min + ls.angle_increment * max(0, len(ranges) - 1)
            ls.range_min = float(p.get("range_min", 0.02))
            ls.range_max = float(p.get("range_max", 5.0))
            # 0 -> inf so slam_toolbox treats them as no-return
            ls.ranges = [float(r) if r > 0 else float("inf") for r in ranges]
            # In rf2o mode, rf2o owns the odom→base_link TF once scans flow;
            # the bridge only re-stamps wheel-odom TF during the holdoff window.
            if not self._rf2o_mode:
                self._republish_odom_at(ls.header.stamp)
            self.scan_pub.publish(ls)
            self.scan_count += 1
        except Exception as e:
            self.get_logger().warning(f"scan publish failed: {e}")

    def _republish_odom_at(self, stamp: TimeMsg):
        if self._last_odom_transform is None:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform = self._last_odom_transform
        self.tf_br.sendTransform(t)

    def _handle_odom(self, p: dict):
        try:
            x = float(p["x"]); y = float(p["y"]); th = float(p["theta"])
            # Normalise to origin: lock the first received odom as (0,0,0) so
            # slam_toolbox always starts near map origin regardless of how much
            # accumulated driver odometry exists (e.g. after a previous clean
            # cycle).  Without this, mapping mode crashes with
            # "unable to get pointer in probability search" when the initial
            # map is too small to cover the robot's odom position.
            if self._odom_origin is None:
                self._odom_origin = (x, y, th)
                self.get_logger().info(
                    f"odom origin locked: x={x:.3f} y={y:.3f} th={math.degrees(th):.1f}°"
                )
            x0, y0, th0 = self._odom_origin
            dx = x - x0
            dy = y - y0
            cos_th0 = math.cos(th0)
            sin_th0 = math.sin(th0)
            x =  dx * cos_th0 + dy * sin_th0
            y = -dx * sin_th0 + dy * cos_th0
            th = th - th0
            t = TransformStamped()
            # Use the wall-clock stamp from the Pi driver so odom TF timestamps
            # align with scan timestamps as captured, not as received over MQTT.
            raw_stamp = p.get("stamp")
            if raw_stamp is not None:
                t.header.stamp = wall_to_time_msg(float(raw_stamp))
            else:
                t.header.stamp = self.get_clock().now().to_msg()
            self._last_odom_stamp = t.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            qx, qy, qz, qw = quat_from_yaw(th)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            # In rf2o mode, only publish wheel-encoder odom TF during the scan
            # holdoff window.  Once scans start flowing rf2o takes ownership of
            # odom→base_link and publishing both would cause TF conflicts.
            if not self._rf2o_mode or time.time() < self._scan_ready_time:
                self.tf_br.sendTransform(t)
            # Cache the transform so _handle_scan can republish it at scan time
            self._last_odom_transform = t.transform
            self.odom_count += 1
        except Exception as e:
            self.get_logger().warning(f"odom publish failed: {e}")

    # ── ROS -> MQTT (/map live preview) ───────────────────────────
    def _on_map(self, msg: OccupancyGrid):
        """Render an OccupancyGrid to a base64 JPEG and publish to map_image.

        Only fires during MAPPING mode and at most every 5 s to keep CPU
        and MQTT payload load manageable on the Pi Zero 2 W.
        """
        if self.slam_mode != "mapping":
            return
        now = time.time()
        if now - self._last_map_pub < 5.0:
            return
        self._last_map_pub = now

        try:
            import numpy as np
            from PIL import Image as PILImage, ImageDraw as PILImageDraw, ImageFont as PILImageFont
            import base64
            import io as _io
        except ImportError as exc:
            self.get_logger().warning(f"_on_map: missing dep {exc}")
            return

        try:
            w = msg.info.width
            h = msg.info.height
            if w == 0 or h == 0:
                return
            res   = msg.info.resolution          # m/px
            ox    = msg.info.origin.position.x   # map-frame x at pixel col 0
            oy    = msg.info.origin.position.y   # map-frame y at pixel row 0

            # OccupancyGrid: -1=unknown, 0=free, 100=occupied
            # ROS convention: row 0 = bottom of map (min y)
            data = np.array(msg.data, dtype=np.int8).reshape((h, w))

            # Build RGB image (row 0 of PIL = top → flip vertically)
            rgb = np.empty((h, w, 3), dtype=np.uint8)
            rgb[:] = (210, 215, 222)              # default: unknown gray
            rgb[data == 0]   = (255, 255, 255)    # free: white
            rgb[data == 100] = (50,  55,  65)     # occupied: dark
            # flip so north is up in the image
            rgb = np.flipud(rgb)

            img = PILImage.fromarray(rgb, "RGB")
            # Scale up so the longer dimension is at least 400 px — keeps the
            # image sharp in HA without the card having to stretch tiny pixels.
            scale = max(4, 400 // max(w, h, 1))
            img = img.resize((w * scale, h * scale), PILImage.NEAREST)
            draw = PILImageDraw.Draw(img)

            def world_to_px(wx: float, wy: float):
                """Map-frame metres → pixel coords in the upscaled image."""
                col = int((wx - ox) / res) * scale
                row = int((h - 1 - (wy - oy) / res)) * scale
                return col, row

            # Dock marker at map origin (robot starts at dock = (0,0) in map frame).
            dx, dy = world_to_px(0.0, 0.0)
            r = max(6, scale * 3)
            draw.ellipse([dx - r, dy - r, dx + r, dy + r],
                         fill=(34, 170, 85), outline=(20, 120, 60), width=max(1, scale // 2))

            # Robot marker from latest map→base_link TF.
            try:
                tf_ = self.tf_buf.lookup_transform(
                    "map", "base_link", rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05))
                rx = tf_.transform.translation.x
                ry = tf_.transform.translation.y
                q  = tf_.transform.rotation
                rth = yaw_from_quat(q.x, q.y, q.z, q.w)
                px, py = world_to_px(rx, ry)
                r = max(5, scale * 2)
                draw.ellipse([px - r, py - r, px + r, py + r],
                             fill=(41, 121, 255), outline=(20, 70, 200), width=max(1, scale // 2))
                tip = r * 1.8
                tx = px + tip * math.cos(rth)
                ty = py - tip * math.sin(rth)
                lx = px + r * 0.6 * math.cos(rth + 2.4)
                ly = py - r * 0.6 * math.sin(rth + 2.4)
                rx2 = px + r * 0.6 * math.cos(rth - 2.4)
                ry2 = py - r * 0.6 * math.sin(rth - 2.4)
                draw.polygon([(tx, ty), (lx, ly), (rx2, ry2)], fill=(255, 255, 255))
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass

            # Small corner label — doesn't obscure the map.
            try:
                font = PILImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 14)
            except (OSError, IOError):
                font = PILImageFont.load_default()
            label = "MAPPING"
            bb = draw.textbbox((0, 0), label, font=font)
            lw, lh = bb[2] - bb[0], bb[3] - bb[1]
            pad = 4
            x0 = img.width - lw - pad * 2 - 2
            y0 = 2
            draw.rectangle([x0, y0, x0 + lw + pad * 2, y0 + lh + pad * 2],
                           fill=(0, 0, 0, 160))
            draw.text((x0 + pad, y0 + pad), label, fill=(255, 200, 50), font=font)

            buf = _io.BytesIO()
            img.save(buf, "JPEG", quality=70, optimize=True)
            b64 = base64.b64encode(buf.getvalue()).decode("ascii")
            self.mq.publish(f"{self.prefix}/map_image", b64, qos=0, retain=True)
        except Exception as exc:
            self.get_logger().warning(f"_on_map render failed: {exc}")

    # ── ROS -> MQTT (pose) ───────────────────────────────────────────
    def _tick_pose(self):
        try:
            tf_ = self.tf_buf.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        x = tf_.transform.translation.x
        y = tf_.transform.translation.y
        q = tf_.transform.rotation
        th = yaw_from_quat(q.x, q.y, q.z, q.w)
        payload = json.dumps({
            "t": time.time(),
            "x": x, "y": y, "theta": th,
            "src": "slam_toolbox",
            "mode": self.slam_mode,
            "map_name": self.slam_map_name,
            "map_id": self.slam_map_id,
        })
        self.mq.publish(f"{self.prefix}/pose", payload, qos=0, retain=False)
        self.pose_count += 1

    def _log_stats(self):
        self.get_logger().info(
            f"stats: scan={self.scan_count} odom={self.odom_count} "
            f"pose_pub={self.pose_count}"
        )

    def shutdown(self):
        try:
            self.mq.loop_stop()
            self.mq.disconnect()
        except Exception:
            pass


def main():
    logging.basicConfig(
        level=os.environ.get("BRIDGE_LOG_LEVEL", "INFO"),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
