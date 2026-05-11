"""
driver_node.py — ROSie Neato D6 ROS 2 driver node (Pi-side).

Replaces the old mqtt_bridge.py + main.py + mqtt_bridge_node.py pipeline.

Architecture
------------
Everything runs in a single process on the Pi inside a Docker container.
Serial, GPIO, DDS and MQTT are all local — no network in the critical path.

DDS (ROS 2 topics/TF) — critical path, time-sensitive:
  Publish:
    /scan           (sensor_msgs/LaserScan)       — 5 Hz, BEST_EFFORT
    /odom           (nav_msgs/Odometry)            — ~10 Hz, BEST_EFFORT
    TF odom→base_footprint                         — with every odom update
                                                     + 20 Hz heartbeat
  Subscribe:
    /cmd_vel        (geometry_msgs/Twist)          — Nav2 velocity commands

MQTT (paho) — HA integration, non-critical:
  Publish:
    rosie/battery, rosie/state, rosie/settings, rosie/neato_sensors,
    rosie/bumpers, rosie/nogo_lines, rosie/nogo_status,
    rosie/availability, rosie/odom, rosie/vacuum_state, rosie/spot_config
    rosie/pose      ← SLAM-corrected pose (5 Hz, from TF lookup)
  Subscribe:
    rosie/command, rosie/cmd_vel, rosie/settings/+/set,
    rosie/spot_width/set, rosie/spot_height/set, rosie/nav_mode/set,
    rosie/vacuum_motor/set, rosie/vacuum_speed/set,
    rosie/nogo_lines/set
"""

import json
import logging
import math
import os
import signal
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

import paho.mqtt.client as mqtt

# ---------------------------------------------------------------------------
# Conditional GPIO / hardware imports (gracefully absent on non-Pi)
# ---------------------------------------------------------------------------
try:
    from .serial_handler import NeatoSerial, BASE_WIDTH_MM
    from .odometry import OdomState, get_motors, update_odometry
    from .lidar import get_lidar_scan
    from .sensors import (
        get_battery, get_robot_state, get_user_settings,
        get_version, get_analog_sensors, get_bumpers,
    )
    from .commands import handle_command, handle_cmd_vel
    from . import no_go_guard
    from . import bumper_sensors
except ImportError:
    # Allow the package to be imported on non-Pi for linting
    NeatoSerial = None  # type: ignore[assignment,misc]
    OdomState = None  # type: ignore[assignment,misc]
    BASE_WIDTH_MM = 245  # type: ignore[assignment]

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# HA discovery device block
# ---------------------------------------------------------------------------
_DEVICE = {
    "identifiers": ["rosie_neato_d6"],
    "name": "ROSie",
    "manufacturer": "Neato Robotics",
    "model": "BotVac D6 Connected",
    "sw_version": "rosie-driver 0.3.0",
}

# Timing intervals (seconds)
_SCAN_INTERVAL     = 0.20   # 5 Hz
_STATE_INTERVAL    = 10.0
_CHARGER_INTERVAL  = 10.0
_SETTINGS_INTERVAL = 60.0
_ANALOG_INTERVAL   = 0.5
_BUMPER_INTERVAL   = 0.1   # 10 Hz bumper poll
_SERIAL_FAIL_LIMIT = 20
_NOGO_LINES_FILE   = os.environ.get("ROSIE_NOGO_LINES_FILE", "/data/nogo-lines.json")


def _env_int(name: str, default: int, minimum: int = 0) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        value = int(raw)
    except ValueError:
        return default
    return max(minimum, value)


def _env_flag(name: str, default: bool = True) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val.strip().lower() not in {"0", "false", "no", "off", "disabled"}


_NOGO_PULSE_MS                = _env_int("ROSIE_NOGO_PULSE_MS", 80, minimum=1)
_NOGO_PULSE_INTERVAL_MS       = _env_int("ROSIE_NOGO_PULSE_INTERVAL_MS", 160, minimum=1)
_NOGO_SIDE_PULSES_BEFORE_FRONT = _env_int("ROSIE_NOGO_SIDE_PULSES_BEFORE_FRONT", 3, minimum=1)
_NOGO_VIRTUAL_STOP_ENABLED    = _env_flag("ROSIE_NOGO_VIRTUAL_STOP_ENABLED", default=False)


class NeaDriver(Node):
    """
    ROSie driver node — Neato serial ↔ ROS 2 + HA MQTT bridge.
    """

    def __init__(self, serial_port: str, baudrate: int,
                 mqtt_host: str, mqtt_port: int,
                 mqtt_user: str, mqtt_pass: str, mqtt_prefix: str):
        super().__init__('rosie_driver')

        self._prefix = mqtt_prefix

        # ------------------------------------------------------------------
        # ROS 2 publishers, subscribers, TF
        # ------------------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._scan_pub  = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self._odom_pub  = self.create_publisher(Odometry,  '/odom', sensor_qos)
        self._joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self._on_ros_cmd_vel, 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._on_ros_cmd_vel, 10)

        # TF buffer for SLAM-corrected pose (map→base_footprint) → rosie/pose
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Latest odom for TF heartbeat
        self._latest_odom: Optional[tuple] = None      # (x, y, theta)
        self._latest_odom_enc: Optional[tuple] = None  # (left_mm, right_mm) raw encoders
        self._odom_lock   = threading.Lock()

        # Wheel encoder positions for JointState
        self._wheel_radius_mm = 30.0
        self._left_wheel_rad  = 0.0
        self._right_wheel_rad = 0.0

        # SLAM-anchored dead-reckoning for no-go guard.
        # Each time SLAM gives a corrected pose we snapshot both the SLAM
        # (x,y,theta) and the raw encoder counts at that moment.  Between
        # corrections the guard integrates only the encoder *delta* from
        # that snapshot, keeping it in the correct SLAM coordinate frame
        # without waiting for the next 5 Hz TF update.
        self._slam_anchor: Optional[tuple] = None          # (x, y, theta)
        self._slam_enc_snapshot: Optional[tuple] = None    # (left_mm, right_mm)
        self._slam_anchor_lock = threading.Lock()

        # ------------------------------------------------------------------
        # Hardware state
        # ------------------------------------------------------------------
        self._serial: Optional[NeatoSerial] = None
        self._serial_port  = serial_port
        self._baudrate     = baudrate
        self._skey         = ""
        self._lds_active   = False
        self._was_scan_active = False
        self._odom         = OdomState() if OdomState is not None else None
        self._serial_fail_count = 0
        self._robot_state  = None   # last GetRobotState result
        self._shutdown_event = threading.Event()

        # Settings override cache (prevent poll clobbering recent MQTT toggles)
        self._settings_overrides: dict = {}
        self._SETTINGS_OVERRIDE_TTL = 10.0
        self._nav_mode  = "Normal"
        self._spot_w    = 200
        self._spot_h    = 200
        self._vac_on    = False
        self._vac_spd   = 65
        self._vac_rpm   = 0
        self._vac_ma    = 0
        self._last_settings: dict = {}

        # ------------------------------------------------------------------
        # MQTT client (runs on its own paho thread via loop_start)
        # ------------------------------------------------------------------
        self._mqtt = mqtt.Client(
            client_id='rosie-pi-driver',
            protocol=mqtt.MQTTv311,
        )
        if mqtt_user:
            self._mqtt.username_pw_set(mqtt_user, mqtt_pass)
        self._mqtt.will_set(
            f"{self._prefix}/availability",
            payload="offline", qos=1, retain=True,
        )
        self._mqtt.on_connect = self._mqtt_on_connect
        self._mqtt.on_message = self._mqtt_on_message

        self.get_logger().info(f'Connecting to MQTT {mqtt_host}:{mqtt_port}')
        self._mqtt.connect(mqtt_host, mqtt_port, keepalive=60)
        self._mqtt.loop_start()

        # ------------------------------------------------------------------
        # Timers (millisecond precision via rclpy)
        # ------------------------------------------------------------------
        # 20 Hz TF heartbeat — keeps odom→base_footprint alive for consumers
        self.create_timer(0.05,  self._tf_heartbeat)
        # 5 Hz SLAM pose → rosie/pose MQTT
        self.create_timer(0.2,   self._pose_to_mqtt)

        # ------------------------------------------------------------------
        # Serial + main poll loop in a background thread
        # (blocking serial IO cannot live on the rclpy executor thread)
        # ------------------------------------------------------------------
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name='neato-poll', daemon=True)

    def start_poll_thread(self):
        """Call after node is added to executor."""
        self._poll_thread.start()

    def destroy_node(self):
        self._shutdown_event.set()
        self._poll_thread.join(timeout=10)
        self._cleanup_hardware()
        self._mqtt.publish(
            f"{self._prefix}/availability", "offline", qos=1, retain=True)
        self._mqtt.loop_stop()
        self._mqtt.disconnect()
        super().destroy_node()

    # ==========================================================================
    # TF / Pose callbacks
    # ==========================================================================

    def _tf_heartbeat(self):
        """Republish latest odom TF at 20 Hz so SLAM always has a fresh transform."""
        with self._odom_lock:
            odom = self._latest_odom
        if odom is None:
            return
        x, y, theta = odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _pose_to_mqtt(self):
        """Look up map→base_link TF, anchor the dead-reckoning guard, and publish to MQTT."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        tx = t.transform.translation
        rz = t.transform.rotation.z
        rw = t.transform.rotation.w
        theta = 2.0 * math.atan2(rz, rw)
        # Snapshot SLAM pose + current raw encoder counts together so
        # _get_best_pose() can dead-reckon forward from this anchor.
        with self._odom_lock:
            odom_now = self._latest_odom_enc   # (left_mm, right_mm) or None
        with self._slam_anchor_lock:
            self._slam_anchor = (tx.x, tx.y, theta)
            self._slam_enc_snapshot = odom_now
        self._mqtt_pub('pose', {
            'x': round(tx.x, 4),
            'y': round(tx.y, 4),
            'theta': round(theta, 4),
        })

    def _get_best_pose(self) -> tuple:
        """Return the best available (x, y, theta) for no-go enforcement.

        Uses SLAM-anchored dead-reckoning: applies encoder delta since the
        last SLAM fix, so the position is always in the SLAM map frame and
        updated at odom rate (~10 Hz) rather than waiting for 5 Hz SLAM.
        Falls back to raw odom if no SLAM fix has been received yet.
        """
        with self._slam_anchor_lock:
            anchor = self._slam_anchor
            enc_snap = self._slam_enc_snapshot
        if anchor is None or enc_snap is None:
            # No SLAM fix yet — use raw odometry as fallback
            return (self._odom.x, self._odom.y, self._odom.theta)
        ax, ay, atheta = anchor
        snap_left, snap_right = enc_snap
        cur_left  = self._odom._prev_left_mm
        cur_right = self._odom._prev_right_mm
        d_left  = (cur_left  - snap_left)  / 1000.0   # mm → m
        d_right = (cur_right - snap_right) / 1000.0
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / (BASE_WIDTH_MM / 1000.0)
        mid_theta = atheta + d_theta / 2.0
        x = ax + d_center * math.cos(mid_theta)
        y = ay + d_center * math.sin(mid_theta)
        theta = math.atan2(math.sin(atheta + d_theta), math.cos(atheta + d_theta))
        return (x, y, theta)

    # ==========================================================================
    # ROS 2 subscriber callbacks
    # ==========================================================================

    def _on_ros_cmd_vel(self, msg: Twist):
        """Forward Nav2 velocity commands to Neato serial."""
        if not self._lds_active:
            return
        try:
            handle_cmd_vel(self._serial, msg.linear.x, msg.angular.z)
        except Exception as exc:
            self.get_logger().warn(f'cmd_vel serial error: {exc}')

    # ==========================================================================
    # DDS publish helpers
    # ==========================================================================

    def _publish_scan(self, scan):
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min       = scan.angle_min
        msg.angle_max       = scan.angle_max
        msg.angle_increment = scan.angle_increment
        msg.time_increment  = 0.0
        msg.scan_time       = _SCAN_INTERVAL
        msg.range_min       = scan.range_min
        msg.range_max       = scan.range_max
        msg.ranges          = [
            float(r) if math.isfinite(r) and scan.range_min <= r <= scan.range_max
            else float('inf')
            for r in scan.ranges
        ]
        msg.intensities = [float(i) for i in scan.intensities]
        self._scan_pub.publish(msg)

    def _publish_odom(self, odom):
        stamp = self.get_clock().now().to_msg()

        with self._odom_lock:
            self._latest_odom = (odom.x, odom.y, odom.theta)
            self._latest_odom_enc = (odom._prev_left_mm, odom._prev_right_mm)

        # Odometry message
        omsg = Odometry()
        omsg.header.stamp          = stamp
        omsg.header.frame_id       = 'odom'
        omsg.child_frame_id        = 'base_footprint'
        omsg.pose.pose.position.x  = odom.x
        omsg.pose.pose.position.y  = odom.y
        omsg.pose.pose.position.z  = 0.0
        omsg.pose.pose.orientation.z = math.sin(odom.theta / 2.0)
        omsg.pose.pose.orientation.w = math.cos(odom.theta / 2.0)
        omsg.twist.twist.linear.x  = odom.linear_vel
        omsg.twist.twist.angular.z = odom.angular_vel
        # Covariance — Neato D6 wheel odometry is approximate.
        # Sets diagonal XY=0.01 (1 cm²), Z-rotation=0.03 so AMCL
        # trusts laser more than odometry.
        omsg.pose.covariance[0]  = 0.01  # x
        omsg.pose.covariance[7]  = 0.01  # y
        omsg.pose.covariance[35] = 0.03  # yaw
        omsg.twist.covariance[0]  = 0.01
        omsg.twist.covariance[35] = 0.03
        self._odom_pub.publish(omsg)

        # Immediate TF broadcast at capture time
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = odom.x
        t.transform.translation.y = odom.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(odom.theta / 2.0)
        t.transform.rotation.w = math.cos(odom.theta / 2.0)
        self._tf_broadcaster.sendTransform(t)

    def _publish_joint_states(self, motors: dict):
        """Publish wheel joint angles from encoder positions so Foxglove
        can render the continuous joints without warnings."""
        left_mm  = motors.get('LeftWheel_PositionInMM',  self._left_wheel_rad  * self._wheel_radius_mm)
        right_mm = motors.get('RightWheel_PositionInMM', self._right_wheel_rad * self._wheel_radius_mm)
        self._left_wheel_rad  = left_mm  / self._wheel_radius_mm
        self._right_wheel_rad = right_mm / self._wheel_radius_mm
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self._left_wheel_rad, self._right_wheel_rad]
        msg.velocity = []
        msg.effort   = []
        self._joint_state_pub.publish(msg)

    # ==========================================================================
    # Serial / hardware poll loop  (background thread)
    # ==========================================================================

    def _poll_loop(self):
        """Main hardware poll loop — runs in its own thread so serial IO
        never blocks the rclpy executor."""
        self._init_hardware()

        last_state    = 0.0
        last_charger  = 0.0
        last_settings = 0.0
        last_analog   = 0.0
        last_bumper   = 0.0
        _prev_bumper  = (False, False, False, False)  # lf, rf, ls, rs
        last_scan     = 0.0
        robot_state_cache = None

        while not self._shutdown_event.is_set():
            now = time.monotonic()

            # ----------------------------------------------------------------
            # Serial reconnect if too many consecutive failures
            # ----------------------------------------------------------------
            if self._serial_fail_count >= _SERIAL_FAIL_LIMIT:
                self.get_logger().warn(
                    'Too many serial failures — reconnecting to Neato')
                self._serial_fail_count = 0
                self._lds_active = False
                try:
                    self._serial.disconnect()
                except Exception:
                    pass
                time.sleep(5)
                while not self._shutdown_event.is_set():
                    try:
                        self._serial.connect()
                        time.sleep(1.0)
                        self._try_compute_skey()
                        self.get_logger().info('Serial reconnected')
                        break
                    except Exception as exc:
                        self.get_logger().warn(f'Reconnect failed: {exc}')
                        time.sleep(5)

            # ----------------------------------------------------------------
            # Always-on polling (works without TestMode)
            # ----------------------------------------------------------------
            if now - last_state >= _STATE_INTERVAL:
                last_state = now
                try:
                    robot_state_cache = get_robot_state(self._serial)
                    self._robot_state = robot_state_cache
                    self._mqtt_publish_state(robot_state_cache)
                except Exception:
                    logger.debug('State poll failed', exc_info=True)

            if now - last_charger >= _CHARGER_INTERVAL:
                last_charger = now
                try:
                    batt = get_battery(self._serial)
                    self._mqtt_publish_battery(batt)
                except Exception:
                    logger.debug('Charger poll failed', exc_info=True)

            if now - last_analog >= _ANALOG_INTERVAL:
                last_analog = now
                try:
                    analog = get_analog_sensors(self._serial)
                    self._mqtt_pub('neato_sensors', {
                        'wall_mm':        analog.wall_mm,
                        'drop_left_mm':   analog.drop_left_mm,
                        'drop_right_mm':  analog.drop_right_mm,
                    }, retain=True)
                except Exception:
                    logger.debug('Analog poll failed', exc_info=True)

            if now - last_bumper >= _BUMPER_INTERVAL:
                last_bumper = now
                if bumper_sensors.virtual_active():
                    pass  # virtual trigger owns bumper state, skip serial
                else:
                    try:
                        b = get_bumpers(self._serial)
                        cur = (b.left_front, b.right_front, b.left_side, b.right_side)
                        if cur != _prev_bumper:
                            logger.info('Serial bumpers: LF=%s RF=%s LS=%s RS=%s', *cur)
                            _prev_bumper = cur
                            self._pub_bumpers(*cur)
                            # NOTE: do NOT send pause_cleaning here — the Neato firmware
                            # already responds to physical bumper hits autonomously during
                            # a cleaning run.  Sending an extra pause caused phantom
                            # "picked up" / pause events when the host poll race-conditioned
                            # the firmware's own bumper handling.
                    except Exception:
                        logger.debug('Bumper poll failed', exc_info=True)

            if now - last_settings >= _SETTINGS_INTERVAL:
                last_settings = now
                try:
                    s = get_user_settings(self._serial)
                    self._mqtt_publish_settings(
                        s.eco_mode, s.wall_enable, s.intense_clean,
                        s.click_sounds, s.melody_sounds, s.warning_sounds,
                        s.bin_full_detect, s.led,
                    )
                except Exception:
                    logger.debug('Settings poll failed', exc_info=True)

            # ----------------------------------------------------------------
            # Determine if scan is active
            # ----------------------------------------------------------------
            robot_cleaning = False
            if robot_state_cache is not None:
                ui = robot_state_cache.ui_state.upper()
                robot_cleaning = any(k in ui for k in (
                    'CLEANINGRUNNING', 'CLEANINGPAUSED',
                    'STARTHOUSECLEAN', 'STARTSPOTCLEAN',
                ))
            scan_active = self._lds_active or robot_cleaning

            # Tick no-go pulse sequencer in the main loop
            if hasattr(self, '_tick_nogo_pulses'):
                if scan_active:
                    self._tick_nogo_pulses(now)
                elif hasattr(self, '_nogo_seq') and any(
                    st['touching'] for st in self._nogo_seq.values()
                ):
                    self.get_logger().info('No-go pulse reset (scan inactive)')
                    for side in ('left', 'right'):
                        self._reset_nogo_side(side)
                    bumper_sensors.release_virtual()

            # Reset odometry on idle→active transition
            if scan_active and not self._was_scan_active:
                self.get_logger().info('Scan active — resetting odometry')
                self._odom = OdomState()
            self._was_scan_active = scan_active

            # ----------------------------------------------------------------
            # Odometry + no-go guard (every loop iteration, ~10 Hz)
            # ----------------------------------------------------------------
            if scan_active:
                try:
                    motors = get_motors(self._serial)
                    if motors:
                        self._serial_fail_count = 0
                        self._odom = update_odometry(self._odom, motors)
                        self._publish_odom(self._odom)
                        self._mqtt_pub_odom(self._odom)
                        self._publish_joint_states(motors)

                        # Vacuum telemetry
                        vacuum_rpm = int(motors.get('Vacuum_RPM', 0))
                        vacuum_ma  = int(motors.get('Vacuum_mA', 0))
                        if vacuum_rpm != self._vac_rpm or vacuum_ma != self._vac_ma:
                            self._vac_rpm = vacuum_rpm
                            self._vac_ma  = vacuum_ma
                            self._mqtt_pub_vacuum_state()

                        gx, gy, gt = self._get_best_pose()
                        no_go_guard.check(
                            gx, gy, gt,
                            self._odom.angular_vel, self._odom.linear_vel,
                        )
                    else:
                        self._serial_fail_count += 1
                except Exception:
                    self._serial_fail_count += 1
                    logger.debug('Odom poll failed', exc_info=True)

            # ----------------------------------------------------------------
            # LIDAR scan (~5 Hz)
            # ----------------------------------------------------------------
            if scan_active and now - last_scan >= _SCAN_INTERVAL:
                last_scan = now
                try:
                    scan = get_lidar_scan(self._serial)
                    if scan:
                        self._publish_scan(scan)
                except Exception:
                    logger.debug('Scan failed', exc_info=True)

                # Re-poll odom after blocking LDS read (~300 ms)
                if scan_active:
                    try:
                        motors2 = get_motors(self._serial)
                        if motors2:
                            self._odom = update_odometry(self._odom, motors2)
                            gx, gy, gt = self._get_best_pose()
                            no_go_guard.check(
                                gx, gy, gt,
                                self._odom.angular_vel, self._odom.linear_vel,
                            )
                    except Exception:
                        logger.debug('Post-LDS odom failed', exc_info=True)

            time.sleep(0.005 if scan_active else 0.5)

    # ==========================================================================
    # Hardware init / cleanup
    # ==========================================================================

    def _init_hardware(self):
        """Connect serial, compute SKey, init GPIO."""
        no_go_guard.init()
        bumper_sensors.start(mqtt_bridge=None)

        loaded, msg = no_go_guard.load_lines_file(_NOGO_LINES_FILE)
        self.get_logger().info(f'No-go lines: {msg}')

        self._serial = NeatoSerial(
            port=self._serial_port, baudrate=self._baudrate)

        while not self._shutdown_event.is_set():
            try:
                self._serial.connect()
                break
            except Exception as exc:
                self.get_logger().warn(f'Serial connect failed: {exc} — retrying')
                time.sleep(5)

        time.sleep(1.0)
        self._try_compute_skey()

        # No-go pulse sequencer state — per-side
        nogo_pulse_hold_secs     = _NOGO_PULSE_MS / 1000.0
        nogo_pulse_interval_secs = _NOGO_PULSE_INTERVAL_MS / 1000.0
        # Tunables for the new "smart" pulse rules
        NOGO_RELEASE_GRACE_S = 1.5    # keep side_pulses counter for this long after a brief release
        NOGO_BUSY_AVEL_RPS   = 0.5    # |angular_vel| above this means firmware is actively responding -> skip pulse
        NOGO_STUCK_DIST_M    = 0.01   # if robot moved less than this since last pulse, treat as stuck
        NOGO_STUCK_BACKOFF_S = 0.6    # back off pulse rate this much when stuck
        nogo_seq = {
            'left':  {
                'touching': False, 'is_front': False, 'side_pulses': 0,
                'next_pulse_at': 0.0, 'phase': '',
                'released_at': 0.0, 'last_pulse_pos': None,
            },
            'right': {
                'touching': False, 'is_front': False, 'side_pulses': 0,
                'next_pulse_at': 0.0, 'phase': '',
                'released_at': 0.0, 'last_pulse_pos': None,
            },
        }

        def _reset_nogo_side(side: str) -> None:
            st = nogo_seq[side]
            st['touching'] = False
            st['is_front'] = False
            st['side_pulses'] = 0
            st['next_pulse_at'] = 0.0
            st['phase'] = ''
            st['released_at'] = 0.0
            st['last_pulse_pos'] = None

        def _fire_nogo_pulse(side: str, now_ts: float) -> None:
            st = nogo_seq[side]
            if not st['touching']:
                return
            try:
                gx, gy, gt = self._get_best_pose()
                lv = self._odom.linear_vel if self._odom is not None else 0.0
                av = self._odom.angular_vel if self._odom is not None else 0.0
            except Exception:
                gx = gy = gt = lv = av = 0.0

            # Pulse type follows contact type — never escalate side -> front.
            # Side approach (parallel to line)  -> only side pulses, forever.
            # Head-on approach                  -> front pulses, immediately.
            use_front = bool(st['is_front'])
            phase = 'front' if use_front else 'side'

            # The busy/stuck skip rules ONLY apply to FRONT pulses, because a
            # front pulse triggers the firmware's long back-up-and-rotate
            # response. SIDE pulses are short nudges that should keep firing
            # while the robot is following the line (parallel motion).
            if use_front:
                # Skip rule 1: firmware is actively responding (turning fast).
                if abs(av) > NOGO_BUSY_AVEL_RPS:
                    self.get_logger().info(
                        f'NOGO PULSE SKIP side={side} phase=front reason=busy av={av:+.3f}r/s'
                    )
                    st['next_pulse_at'] = now_ts + nogo_pulse_interval_secs
                    return
                # Skip rule 2: robot hasn't moved since last pulse -> stuck.
                last_pos = st.get('last_pulse_pos')
                if last_pos is not None:
                    dx = gx - last_pos[0]
                    dy = gy - last_pos[1]
                    if math.hypot(dx, dy) < NOGO_STUCK_DIST_M and abs(lv) < 0.02:
                        self.get_logger().info(
                            f'NOGO PULSE SKIP side={side} phase=front reason=stuck '
                            f'dpos={math.hypot(dx, dy):.4f}m lv={lv:+.3f}m/s'
                        )
                        st['next_pulse_at'] = now_ts + NOGO_STUCK_BACKOFF_S
                        return

            if phase != st['phase']:
                self.get_logger().info(f'No-go pulse phase={phase} side={side}')
                st['phase'] = phase
            # trigger_virtual() drives the bumper GPIO pin LOW, faking a real
            # bumper switch closure to the Neato firmware which then performs
            # its built-in back-up-and-turn response.
            if use_front:
                self.get_logger().info(
                    f'NOGO PULSE #{st["side_pulses"]+1} side={side} phase=front '
                    f'pos=({gx:.3f},{gy:.3f}) hdg={math.degrees(gt):.1f}° '
                    f'vel=({lv:+.3f}m/s,{av:+.3f}r/s) hold={int(nogo_pulse_hold_secs*1000)}ms'
                )
                bumper_sensors.trigger_virtual(
                    front_left=(side == 'left'),
                    front_right=(side == 'right'),
                    hold_secs=nogo_pulse_hold_secs,
                    stop_on_trigger=False,
                )
            else:
                self.get_logger().info(
                    f'NOGO PULSE #{st["side_pulses"]+1}/{_NOGO_SIDE_PULSES_BEFORE_FRONT} side={side} phase=side '
                    f'pos=({gx:.3f},{gy:.3f}) hdg={math.degrees(gt):.1f}° '
                    f'vel=({lv:+.3f}m/s,{av:+.3f}r/s) hold={int(nogo_pulse_hold_secs*1000)}ms'
                )
                bumper_sensors.trigger_virtual(
                    side_left=(side == 'left'),
                    side_right=(side == 'right'),
                    hold_secs=nogo_pulse_hold_secs,
                    stop_on_trigger=False,
                )
                st['side_pulses'] += 1
            st['next_pulse_at'] = now_ts + nogo_pulse_interval_secs
            st['last_pulse_pos'] = (gx, gy)

        def _tick_nogo_pulses(now_ts: float) -> None:
            for side, st in nogo_seq.items():
                if st['touching'] and now_ts >= st['next_pulse_at']:
                    _fire_nogo_pulse(side, now_ts)

        self._tick_nogo_pulses = _tick_nogo_pulses
        self._nogo_seq = nogo_seq
        self._reset_nogo_side = _reset_nogo_side

        # Wire no-go touch callback — pulse sequencer with side-first escalation
        def _on_nogo_touch(side: str, is_front: bool, touching: bool):
            if side not in nogo_seq:
                return
            st = nogo_seq[side]
            now_ts = time.monotonic()
            if touching:
                # Preserve escalation state if this touch comes shortly after a release
                # (the robot was rotating across the line and briefly lost contact).
                grace_active = (
                    st['released_at'] > 0.0
                    and (now_ts - st['released_at']) < NOGO_RELEASE_GRACE_S
                )
                if grace_active:
                    self.get_logger().info(
                        f'No-go touch resume side={side} front={is_front} '
                        f'(keeping side_pulses={st["side_pulses"]}, '
                        f'gap={(now_ts - st["released_at"]) * 1000:.0f}ms)'
                    )
                    # Update is_front in case the approach angle changed; keep counters.
                    st['is_front'] = is_front
                else:
                    st['is_front'] = is_front
                    st['side_pulses'] = 0
                    st['phase'] = ''
                    st['last_pulse_pos'] = None
                    self.get_logger().info(
                        f'No-go touch start side={side} front={is_front}'
                    )
                st['touching'] = True
                st['released_at'] = 0.0
                st['next_pulse_at'] = 0.0
                _fire_nogo_pulse(side, now_ts)
            else:
                if st['touching']:
                    self.get_logger().info(f'No-go touch release side={side}')
                # Mark released but keep side_pulses + is_front for the grace window.
                st['touching'] = False
                st['released_at'] = now_ts
                st['next_pulse_at'] = 0.0
        no_go_guard.set_touch_callback(_on_nogo_touch)

        # Stop callback for bumper_sensors — pauses cleaning
        def _do_stop():
            if self._serial and self._skey:
                try:
                    handle_command(self._serial, 'pause_cleaning', self._skey)
                except Exception as exc:
                    self.get_logger().warn(f'pause on bumper failed: {exc}')
        bumper_sensors.set_stop_callback(_do_stop)

        # Wire bumper_sensors to MQTT (shim so module publishes bumper state)
        class _BumperShim:
            def __init__(self_, pub_fn):  # noqa: N805
                self_._pub_fn = pub_fn
            def publish_bumpers(self_, left_front, right_front, left_side, right_side):  # noqa: N805
                self_._pub_fn(left_front, right_front, left_side, right_side)
        bumper_sensors._mqtt = _BumperShim(self._pub_bumpers)

        self.get_logger().info('Hardware initialised')

    def _pub_bumpers(self, left_front: bool, right_front: bool,
                     left_side: bool, right_side: bool):
        """Callback for bumper_sensors to publish bump state."""
        self._mqtt_pub('bumpers', {
            'left_front':  left_front,
            'right_front': right_front,
            'left_side':   left_side,
            'right_side':  right_side,
        })

    def _cleanup_hardware(self):
        try:
            bumper_sensors.stop()
        except Exception:
            pass
        try:
            no_go_guard.cleanup()
        except Exception:
            pass
        if self._serial and self._lds_active:
            try:
                self._serial.set_motors(0, 0, 0)
                time.sleep(0.1)
                self._serial.set_lds_rotation(False)
                time.sleep(0.1)
                self._serial.set_test_mode(False)
            except Exception:
                pass
        if self._serial:
            try:
                self._serial.disconnect()
            except Exception:
                pass

    def _try_compute_skey(self):
        try:
            version = get_version(self._serial)
            if version.serial_number:
                self._skey = NeatoSerial.compute_skey(version.serial_number)
                self.get_logger().info(
                    f'SKey computed from S/N: '
                    f'{version.serial_number.split(",")[0]}')
        except Exception as exc:
            self.get_logger().warn(f'SKey computation failed: {exc}')

    def _activate_lds(self):
        if self._lds_active:
            return
        self.get_logger().info('Activating LDS')
        self._serial.set_test_mode(True)
        self._serial.set_lds_rotation(True)
        time.sleep(3.0)
        self._serial.flush()
        get_lidar_scan(self._serial)   # discard first scan
        self._lds_active = True

    def _deactivate_lds(self):
        if not self._lds_active:
            return
        self.get_logger().info('Deactivating LDS')
        self._serial.set_motors(0, 0, 0)
        time.sleep(0.1)
        self._serial.set_lds_rotation(False)
        time.sleep(0.1)
        self._serial.set_test_mode(False)
        self._lds_active = False

    # ==========================================================================
    # MQTT helpers
    # ==========================================================================

    def _mqtt_pub(self, suffix: str, payload, qos: int = 0, retain: bool = False):
        topic = f"{self._prefix}/{suffix}"
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        self._mqtt.publish(topic, payload, qos=qos, retain=retain)

    def _mqtt_pub_odom(self, odom):
        """Publish odom to MQTT for rosie_server map overlay (non-critical)."""
        self._mqtt_pub('odom', {
            'x': round(odom.x, 4),
            'y': round(odom.y, 4),
            'theta': round(odom.theta, 4),
            'linear_vel': round(odom.linear_vel, 4),
            'angular_vel': round(odom.angular_vel, 4),
            'stamp': odom.timestamp,
        })

    def _mqtt_pub_vacuum_state(self):
        self._mqtt_pub('vacuum_state', {
            'vacuum_on':  'ON' if self._vac_on else 'OFF',
            'vacuum_speed': self._vac_spd,
            'vacuum_rpm': self._vac_rpm,
            'vacuum_ma':  self._vac_ma,
        }, retain=True)

    def _mqtt_publish_battery(self, batt):
        self._mqtt_pub('battery', {
            'fuel_percent': batt.fuel_percent,
            'voltage':      round(batt.voltage, 2),
            'charging':     batt.charging_active,
            'ext_power':    batt.ext_power_present,
            'temperature':  round(batt.battery_temp_c, 1),
        }, retain=True)

    def _mqtt_publish_state(self, state):
        error  = state.error
        alert  = state.alert
        if 'UI_ALERT_INVALID' in error or 'UI_ALERT_INVALID' in alert:
            error = 'none'
            alert = 'none'
        ha_state = self._map_vacuum_state(state.ui_state, error)
        self._mqtt_pub('state', {
            'state':        ha_state,
            'ui_state':     state.ui_state,
            'robot_state':  state.robot_state,
            'error':        error,
            'alert':        alert,
        }, retain=True)

    def _mqtt_publish_settings(self, eco_mode, wall_enable, intense_clean,
                                click_sounds, melody_sounds, warning_sounds,
                                bin_full_detect, led):
        now = time.monotonic()
        expired = [k for k, (_, ts) in self._settings_overrides.items()
                   if now - ts > self._SETTINGS_OVERRIDE_TTL]
        for k in expired:
            del self._settings_overrides[k]

        vals = dict(
            eco_mode=eco_mode, wall_enable=wall_enable,
            intense_clean=intense_clean, click_sounds=click_sounds,
            melody_sounds=melody_sounds, warning_sounds=warning_sounds,
            bin_full_detect=bin_full_detect, led=led,
        )
        for k, (v, _) in self._settings_overrides.items():
            if k in vals:
                vals[k] = v

        payload = {k: ('ON' if v else 'OFF') for k, v in vals.items()}
        payload['nav_mode'] = self._nav_mode
        self._last_settings = payload
        self._mqtt_pub('settings', payload, retain=True)

    @staticmethod
    def _map_vacuum_state(ui_state: str, error: str) -> str:
        if error and error != 'none':
            return 'error'
        ui = ui_state.upper()
        if 'PAUSED' in ui:
            return 'paused'
        if any(k in ui for k in ('CLEANINGRUNNING', 'STARTCLEAN',
                                  'STARTHOUSECLEAN', 'STARTSPOTCLEAN')):
            return 'cleaning'
        if any(k in ui for k in ('GOTOBASE', 'SENDTOBASE', 'DOCKINGRUNNING')):
            return 'returning'
        if any(k in ui for k in ('DOCKED', 'CHARGING')):
            return 'docked'
        return 'idle'

    # ==========================================================================
    # HA MQTT Discovery
    # ==========================================================================

    def _availability(self):
        return [{'topic': f'{self._prefix}/availability',
                 'payload_available': 'online',
                 'payload_not_available': 'offline'}]

    def _pub_discovery(self, component: str, object_id: str, config: dict):
        config.setdefault('device', _DEVICE)
        config.setdefault('availability', self._availability())
        self._mqtt.publish(
            f"homeassistant/{component}/rosie_{object_id}/config",
            json.dumps(config), qos=1, retain=True,
        )

    def _publish_ha_discovery(self):
        pfx = self._prefix

        # Vacuum entity
        self._pub_discovery('vacuum', 'vacuum', {
            'name': None,
            'unique_id': 'rosie_vacuum_v2',
            'object_id': 'rosie',
            'command_topic': f'{pfx}/command',
            'payload_start': 'start',
            'payload_stop': 'stop',
            'payload_pause': 'pause',
            'payload_return_to_base': 'return_to_base',
            'payload_locate': 'locate',
            'payload_clean_spot': 'clean_spot',
            'state_topic': f'{pfx}/state',
            'value_template': '{{ value_json.state }}',
            'battery_level_topic': f'{pfx}/battery',
            'battery_level_template': '{{ value_json.fuel_percent | int }}',
            'charging_topic': f'{pfx}/battery',
            'charging_template': '{{ value_json.charging }}',
            'fan_speed_list': ['eco', 'normal'],
            'set_fan_speed_topic': f'{pfx}/command',
            'json_attributes_topic': f'{pfx}/state',
            'icon': 'mdi:robot-vacuum',
        })

        # Buttons
        buttons = [
            ('house_clean', 'House Clean', 'mdi:home'),
            ('spot_clean', 'Spot Clean', 'mdi:target'),
            ('spot_clean_hw', 'Spot Clean (H×W)', 'mdi:target'),
            ('stop_cleaning', 'Stop Cleaning', 'mdi:stop'),
            ('pause_cleaning', 'Pause Cleaning', 'mdi:pause'),
            ('resume_cleaning', 'Resume Cleaning', 'mdi:play'),
            ('send_to_base', 'Send to Base', 'mdi:home-import-outline'),
            ('locate', 'Locate Robot', 'mdi:volume-high'),
            ('manual_forward_down', 'Drive Forward', 'mdi:arrow-up-bold'),
            ('manual_forward_up', 'Drive Forward Stop', 'mdi:arrow-up'),
            ('manual_backwards_down', 'Drive Backward', 'mdi:arrow-down-bold'),
            ('manual_backwards_up', 'Drive Backward Stop', 'mdi:arrow-down'),
            ('manual_turn_left_down', 'Turn Left', 'mdi:arrow-left-bold'),
            ('manual_turn_left_up', 'Turn Left Stop', 'mdi:arrow-left'),
            ('manual_turn_right_down', 'Turn Right', 'mdi:arrow-right-bold'),
            ('manual_turn_right_up', 'Turn Right Stop', 'mdi:arrow-right'),
            ('manual_arc_left_down', 'Arc Left', 'mdi:rotate-left'),
            ('manual_arc_left_up', 'Arc Left Stop', 'mdi:rotate-left'),
            ('manual_arc_right_down', 'Arc Right', 'mdi:rotate-right'),
            ('manual_arc_right_up', 'Arc Right Stop', 'mdi:rotate-right'),
            ('manual_btn_timeout', 'Drive Timeout', 'mdi:timer-off'),
            ('start_manual_cleaning', 'Start Manual Cleaning', 'mdi:play-circle'),
            ('update_status', 'Update Status', 'mdi:refresh'),
            ('clear_errors', 'Clear Errors', 'mdi:notification-clear-all'),
            ('activate', 'Activate LDS', 'mdi:power'),
            ('deactivate', 'Deactivate LDS', 'mdi:power-off'),
            ('shutdown', 'Shutdown Robot', 'mdi:power'),
            ('powercycle', 'Reboot Robot', 'mdi:restart'),
            ('test_bumper_fl', 'Bumper Test: Front Left',  'mdi:gesture-tap'),
            ('test_bumper_fr', 'Bumper Test: Front Right', 'mdi:gesture-tap'),
            ('test_bumper_sl', 'Bumper Test: Side Left',   'mdi:gesture-tap'),
            ('test_bumper_sr', 'Bumper Test: Side Right',  'mdi:gesture-tap'),
        ]
        for btn_id, label, icon in buttons:
            payload = btn_id
            if btn_id == 'spot_clean_hw':
                payload = f'spot_clean_hw:{self._spot_w},{self._spot_h}'
            cat = 'diagnostic' if btn_id in (
                'update_status', 'clear_errors', 'activate', 'deactivate',
                'shutdown', 'powercycle',
            ) else None
            cfg = {
                'name': label,
                'unique_id': f'rosie_{btn_id}',
                'command_topic': f'{pfx}/command',
                'payload_press': payload,
                'icon': icon,
            }
            if cat:
                cfg['entity_category'] = cat
            self._pub_discovery('button', btn_id, cfg)

        # Switches (settings)
        for sw_id, label, icon in [
            ('eco_mode', 'Eco Mode', 'mdi:leaf'),
            ('wall_enable', 'Wall Follower', 'mdi:wall'),
            ('intense_clean', 'Intense Clean', 'mdi:broom'),
            ('click_sounds', 'Click Sounds', 'mdi:volume-medium'),
            ('melody_sounds', 'Melody Sounds', 'mdi:music'),
            ('warning_sounds', 'Warning Sounds', 'mdi:alert'),
            ('bin_full_detect', 'Bin Full Detect', 'mdi:delete-variant'),
            ('led', 'LED', 'mdi:led-on'),
        ]:
            self._pub_discovery('switch', sw_id, {
                'name': label, 'unique_id': f'rosie_{sw_id}',
                'command_topic': f'{pfx}/settings/{sw_id}/set',
                'state_topic': f'{pfx}/settings',
                'value_template': '{{ value_json.' + sw_id + ' }}',
                'payload_on': 'ON', 'payload_off': 'OFF',
                'state_on': 'ON', 'state_off': 'OFF',
                'icon': icon, 'entity_category': 'config',
            })

        # Spot clean dimensions
        for dim_id, label in [('spot_width', 'Spot Clean Width'),
                               ('spot_height', 'Spot Clean Height')]:
            self._pub_discovery('number', dim_id, {
                'name': label, 'unique_id': f'rosie_{dim_id}',
                'command_topic': f'{pfx}/{dim_id}/set',
                'state_topic': f'{pfx}/spot_config',
                'value_template': '{{ value_json.' + dim_id.split('_')[1] + ' }}',
                'min': 100, 'max': 400, 'step': 1,
                'unit_of_measurement': 'cm', 'mode': 'slider',
                'icon': 'mdi:arrow-left-right' if 'width' in dim_id else 'mdi:arrow-up-down',
            })

        # Vacuum motor
        self._pub_discovery('switch', 'vacuum_motor', {
            'name': 'Vacuum Motor', 'unique_id': 'rosie_vacuum_motor',
            'command_topic': f'{pfx}/vacuum_motor/set',
            'state_topic': f'{pfx}/vacuum_state',
            'value_template': '{{ value_json.vacuum_on }}',
            'payload_on': 'ON', 'payload_off': 'OFF',
            'state_on': 'ON', 'state_off': 'OFF',
            'icon': 'mdi:fan',
        })

        # Vacuum speed
        self._pub_discovery('number', 'vacuum_speed', {
            'name': 'Vacuum Speed', 'unique_id': 'rosie_vacuum_speed',
            'command_topic': f'{pfx}/vacuum_speed/set',
            'state_topic': f'{pfx}/vacuum_state',
            'value_template': '{{ value_json.vacuum_speed }}',
            'min': 1, 'max': 100, 'step': 5,
            'unit_of_measurement': '%', 'mode': 'slider',
            'icon': 'mdi:speedometer',
        })

        # Nav mode
        self._pub_discovery('select', 'nav_mode', {
            'name': 'Navigation Mode', 'unique_id': 'rosie_nav_mode',
            'command_topic': f'{pfx}/nav_mode/set',
            'state_topic': f'{pfx}/settings',
            'value_template': '{{ value_json.nav_mode }}',
            'options': ['Normal', 'Gentle', 'Deep', 'Quick'],
            'icon': 'mdi:robot-vacuum', 'entity_category': 'config',
        })

        # Sensors
        sensors = [
            ('fuel_percent', 'Battery', '%', 'battery', f'{pfx}/battery',
             '{{ value_json.fuel_percent | int }}', 'measurement'),
            ('battery_voltage', 'Battery Voltage', 'V', None, f'{pfx}/battery',
             '{{ value_json.voltage }}', 'measurement'),
            ('battery_temp', 'Battery Temperature', '°C', 'temperature',
             f'{pfx}/battery', '{{ value_json.temperature }}', 'measurement'),
            ('ui_state', 'UI State', None, None, f'{pfx}/state',
             '{{ value_json.ui_state }}', None),
            ('robot_state', 'Robot State', None, None, f'{pfx}/state',
             '{{ value_json.robot_state }}', None),
            ('robot_error', 'Robot Error', None, None, f'{pfx}/state',
             '{{ value_json.error }}', None),
            ('robot_alert', 'Robot Alert', None, None, f'{pfx}/state',
             '{{ value_json.alert }}', None),
            ('nogo_line_count', 'No-Go Line Count', None, None, f'{pfx}/nogo_lines',
             '{{ (value_json.lines | default([])) | count }}', 'measurement'),
            ('nogo_status', 'No-Go Status', None, None, f'{pfx}/nogo_status',
             '{{ value_json.status }}', None),
            ('nogo_message', 'No-Go Message', None, None, f'{pfx}/nogo_status',
             '{{ value_json.message }}', None),
            ('vacuum_rpm', 'Vacuum RPM', 'RPM', None, f'{pfx}/vacuum_state',
             '{{ value_json.vacuum_rpm }}', 'measurement'),
            ('vacuum_current', 'Vacuum Current', 'mA', None, f'{pfx}/vacuum_state',
             '{{ value_json.vacuum_ma }}', 'measurement'),
        ]
        for s_id, label, unit, dev_class, topic, tmpl, state_class in sensors:
            cfg = {'name': label, 'unique_id': f'rosie_{s_id}',
                   'state_topic': topic, 'value_template': tmpl}
            if unit:
                cfg['unit_of_measurement'] = unit
            if dev_class:
                cfg['device_class'] = dev_class
            if state_class:
                cfg['state_class'] = state_class
            if s_id in ('battery_voltage', 'battery_temp',
                        'nogo_status', 'nogo_message'):
                cfg['entity_category'] = 'diagnostic'
            if s_id == 'nogo_line_count':
                cfg['json_attributes_topic'] = topic
            self._pub_discovery('sensor', s_id, cfg)

        # Binary sensors
        for bs_id, label, dev_class, topic, tmpl in [
            ('charging', 'Charging', 'battery_charging', f'{pfx}/battery',
             "{{ 'ON' if value_json.charging else 'OFF' }}"),
            ('ext_power', 'Docked', 'plug', f'{pfx}/battery',
             "{{ 'ON' if value_json.ext_power else 'OFF' }}"),
        ]:
            self._pub_discovery('binary_sensor', bs_id, {
                'name': label, 'unique_id': f'rosie_{bs_id}',
                'state_topic': topic, 'value_template': tmpl,
                'payload_on': 'ON', 'payload_off': 'OFF',
                'device_class': dev_class,
            })

        # Physical bump switches
        for bs_id, label, tmpl in [
            ('bumper_front_left',  'Bumper Front Left',
             "{{ 'ON' if value_json.left_front  else 'OFF' }}"),
            ('bumper_front_right', 'Bumper Front Right',
             "{{ 'ON' if value_json.right_front else 'OFF' }}"),
            ('bumper_side_left',   'Bumper Side Left',
             "{{ 'ON' if value_json.left_side   else 'OFF' }}"),
            ('bumper_side_right',  'Bumper Side Right',
             "{{ 'ON' if value_json.right_side  else 'OFF' }}"),
        ]:
            self._pub_discovery('binary_sensor', bs_id, {
                'name': label, 'unique_id': f'rosie_{bs_id}',
                'state_topic': f'{pfx}/bumpers',
                'value_template': tmpl,
                'payload_on': 'ON', 'payload_off': 'OFF',
            })

    # ==========================================================================
    # MQTT callbacks
    # ==========================================================================

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f'MQTT connect failed (rc={rc})')
            return
        self.get_logger().info('Connected to MQTT broker')
        self._publish_ha_discovery()
        # Clear stale HAL discovery topics left over from before HAL removal
        for stale in ('hal_left_approaching', 'hal_left_on_boundary',
                      'hal_right_approaching', 'hal_right_on_boundary',
                      'hal_sensor_left', 'hal_sensor_right'):
            for comp in ('button', 'sensor', 'binary_sensor'):
                self._mqtt.publish(
                    f'homeassistant/{comp}/rosie_{stale}/config',
                    payload='', retain=True)
        self._mqtt_pub('availability', 'online', qos=1, retain=True)
        # Publish spot config and vacuum state
        self._mqtt_pub('spot_config',
                       {'width': self._spot_w, 'height': self._spot_h},
                       retain=True)
        self._mqtt_pub_vacuum_state()

        # Publish no-go lines
        lines = no_go_guard.export_lines()
        self._mqtt_pub('nogo_lines', {'lines': lines}, retain=True)
        if no_go_guard.is_enabled():
            self._mqtt_pub('nogo_status',
                           {'status': 'ok',
                            'message': f'{no_go_guard.line_count()} no-go line(s) active'},
                           retain=True)
        else:
            self._mqtt_pub('nogo_status',
                           {'status': 'disabled',
                            'message': 'disabled by ROSIE_NOGO_ENABLED'},
                           retain=True)
        self._mqtt_pub('bumpers',
                       {'left_front': False, 'right_front': False,
                        'left_side': False, 'right_side': False})

        pfx = self._prefix
        subs = [
            (f'{pfx}/command',             1),
            (f'{pfx}/cmd_vel',             0),
            (f'{pfx}/settings/+/set',      1),
            (f'{pfx}/spot_width/set',      1),
            (f'{pfx}/spot_height/set',     1),
            (f'{pfx}/nav_mode/set',        1),
            (f'{pfx}/vacuum_motor/set',    1),
            (f'{pfx}/vacuum_speed/set',    1),
            (f'{pfx}/nogo_lines/set',      1),
        ]
        for topic, qos in subs:
            client.subscribe(topic, qos=qos)

    def _mqtt_on_message(self, client, userdata, msg):
        topic   = msg.topic
        payload = msg.payload.decode('utf-8', errors='replace').strip()
        pfx     = self._prefix

        try:
            if topic == f'{pfx}/command':
                self._handle_command(payload)

            elif topic == f'{pfx}/cmd_vel':
                data = json.loads(payload)
                lx = float(data.get('linear_x', 0.0))
                az = float(data.get('angular_z', 0.0))
                if self._lds_active:
                    handle_cmd_vel(self._serial, lx, az)

            elif topic.startswith(f'{pfx}/settings/') and topic.endswith('/set'):
                key = topic.split('/')[-2]
                val = payload.upper() == 'ON'
                self._settings_overrides[key] = (val, time.monotonic())
                if self._serial:
                    try:
                        from .commands import SETTINGS_MAP
                        if key in SETTINGS_MAP:
                            neato_name = SETTINGS_MAP[key][0]
                            self._serial.set_user_setting(neato_name, 'ON' if val else 'OFF')
                    except Exception as exc:
                        logger.debug('set_user_setting failed: %s', exc)
                if self._last_settings:
                    updated = dict(self._last_settings)
                    updated[key] = 'ON' if val else 'OFF'
                    self._mqtt_pub('settings', updated, retain=True)

            elif topic == f'{pfx}/spot_width/set':
                try:
                    self._spot_w = max(100, min(400, int(payload)))
                    self._mqtt_pub('spot_config',
                                   {'width': self._spot_w,
                                    'height': self._spot_h}, retain=True)
                except ValueError:
                    pass

            elif topic == f'{pfx}/spot_height/set':
                try:
                    self._spot_h = max(100, min(400, int(payload)))
                    self._mqtt_pub('spot_config',
                                   {'width': self._spot_w,
                                    'height': self._spot_h}, retain=True)
                except ValueError:
                    pass

            elif topic == f'{pfx}/nav_mode/set':
                if payload in ('Normal', 'Gentle', 'Deep', 'Quick'):
                    self._nav_mode = payload
                    if self._last_settings:
                        updated = dict(self._last_settings)
                        updated['nav_mode'] = self._nav_mode
                        self._mqtt_pub('settings', updated, retain=True)

            elif topic == f'{pfx}/vacuum_motor/set':
                self._vac_on = (payload.upper() == 'ON')
                if self._serial:
                    try:
                        self._serial.set_vacuum(self._vac_on, self._vac_spd)
                    except Exception as exc:
                        logger.debug('set_vacuum failed: %s', exc)
                self._mqtt_pub_vacuum_state()

            elif topic == f'{pfx}/vacuum_speed/set':
                try:
                    self._vac_spd = max(1, min(100, int(float(payload))))
                    self._mqtt_pub_vacuum_state()
                except ValueError:
                    pass

            elif topic == f'{pfx}/nogo_lines/set':
                try:
                    data  = json.loads(payload)
                    lines = data.get('lines', data) if isinstance(data, dict) else data
                    if isinstance(lines, list):
                        ok, msg_txt, exported = no_go_guard.set_lines(lines)
                        if ok:
                            no_go_guard.save_lines_file(_NOGO_LINES_FILE)
                            self._mqtt_pub('nogo_lines',
                                           {'lines': exported}, retain=True)
                            self._mqtt_pub('nogo_status',
                                           {'status': 'ok',
                                            'message': msg_txt}, retain=True)
                        else:
                            self._mqtt_pub('nogo_status',
                                           {'status': 'error',
                                            'message': msg_txt}, retain=True)
                except (json.JSONDecodeError, TypeError):
                    pass

        except Exception as exc:
            logger.debug('MQTT message error (%s): %s', topic, exc, exc_info=True)

    def _handle_command(self, cmd: str):
        cmd_lower = cmd.strip().lower()

        # Lazy SKey retry
        if not self._skey and self._serial:
            self._try_compute_skey()

        if cmd_lower == 'activate':
            threading.Thread(target=self._activate_lds, daemon=True).start()
        elif cmd_lower == 'deactivate':
            threading.Thread(target=self._deactivate_lds, daemon=True).start()
        elif cmd_lower == 'update_status':
            pass   # next poll cycle will update
        elif cmd_lower == 'test_bumper_fl':
            bumper_sensors.trigger_virtual(front_left=True, hold_secs=0.6)
            threading.Timer(0.5, bumper_sensors.release_virtual).start()
        elif cmd_lower == 'test_bumper_fr':
            bumper_sensors.trigger_virtual(front_right=True, hold_secs=0.6)
            threading.Timer(0.5, bumper_sensors.release_virtual).start()
        elif cmd_lower == 'test_bumper_sl':
            bumper_sensors.trigger_virtual(side_left=True, hold_secs=0.6)
            threading.Timer(0.5, bumper_sensors.release_virtual).start()
        elif cmd_lower == 'test_bumper_sr':
            bumper_sensors.trigger_virtual(side_right=True, hold_secs=0.6)
            threading.Timer(0.5, bumper_sensors.release_virtual).start()
        elif cmd_lower == 'create_map':
            ok, msg_txt, exported = no_go_guard.set_lines([])
            no_go_guard.save_lines_file(_NOGO_LINES_FILE)
            self._mqtt_pub('nogo_lines', {'lines': exported}, retain=True)
            self._mqtt_pub('nogo_status',
                           {'status': 'ok',
                            'message': 'cleared for new map'}, retain=True)
            if self._serial:
                handle_command(self._serial, cmd, self._skey)
        else:
            if self._serial:
                handle_command(self._serial, cmd, self._skey)


def main(args=None):
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
    )

    rclpy.init(args=args)

    serial_port = os.environ.get('NEATO_PORT', '/dev/ttyACM0')
    baudrate    = int(os.environ.get('NEATO_BAUD', '115200'))
    mqtt_host   = os.environ.get('MQTT_HOST', 'localhost')
    mqtt_port   = int(os.environ.get('MQTT_PORT', '1883'))
    mqtt_user   = os.environ.get('MQTT_USER', '')
    mqtt_pass   = os.environ.get('MQTT_PASS', '')
    mqtt_prefix = os.environ.get('MQTT_PREFIX', 'rosie')

    node = NeaDriver(
        serial_port=serial_port,
        baudrate=baudrate,
        mqtt_host=mqtt_host,
        mqtt_port=mqtt_port,
        mqtt_user=mqtt_user,
        mqtt_pass=mqtt_pass,
        mqtt_prefix=mqtt_prefix,
    )

    node.start_poll_thread()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
