#!/usr/bin/env python3
"""
ROSie Server — unified supervisor for the ROS 2 stack + map pipeline.

Single container runs:
  - Bridge (MQTT↔ROS2 + robot_state_publisher + Foxglove) — always
  - Nav2 (autonomous navigation) — default mode
  - SLAM Toolbox (mapping) — during create_map pipeline

Map pipeline (triggered by 'create_map' on rosie/command):
  stop Nav2 → start SLAM → activate LDS → start cleaning →
  wait for dock → save map → clean map → publish to MQTT camera →
  stop SLAM → start Nav2

HA Discovery entities published by this server:
  - button.rosie_create_new_map   — triggers pipeline
  - sensor.rosie_map_pipeline     — pipeline status
  - camera.rosie_map              — cleaned map image via MQTT
"""

import base64
import io
import json
import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil

import paho.mqtt.client as mqtt
from PIL import Image, ImageDraw, ImageFont

# ── Pipeline States ───────────────────────────────────────────────
IDLE = "idle"
SWITCHING_TO_SLAM = "switching_to_slam"
CLEANING = "cleaning"
WAITING_FOR_DOCK = "waiting_for_dock"
SAVING_MAP = "saving_map"
PROCESSING = "processing"
UPLOADING = "uploading"
SWITCHING_TO_NAV = "switching_to_nav"
DONE = "done"
ERROR = "error"

# HA Discovery device block (matches Pi driver for entity grouping)
_DEVICE = {
    "identifiers": ["rosie_neato_d6"],
    "name": "ROSie",
    "manufacturer": "Neato Robotics",
    "model": "BotVac D6 Connected",
}

MAP_YAML = Path("/ros2_ws/maps/home.yaml")
MAP_CLEAN_PNG = Path("/ros2_ws/maps/home_clean.png")
MAP_META_JSON = Path("/ros2_ws/maps/home_meta.json")
DOCK_JSON = Path("/ros2_ws/maps/dock.json")
PIPELINE_TIMEOUT = 7200  # 2 hours max cleaning

# Marker colours (must match clean_map.py)
DOCK_COLOUR = (34, 170, 85)
ROBOT_COLOUR = (41, 121, 255)
NOGO_COLOUR = (220, 40, 40)
BG_COLOUR = (240, 245, 250)

MAP_REFRESH_INTERVAL = 2  # seconds between map overlay refreshes

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    stream=sys.stdout,
)
log = logging.getLogger("rosie_server")


class RosieServer:
    def __init__(self):
        self.mqtt_host = os.environ["MQTT_HOST"]
        self.mqtt_port = int(os.environ.get("MQTT_PORT", "1883"))
        self.mqtt_user = os.environ["MQTT_USER"]
        self.mqtt_pass = os.environ["MQTT_PASS"]
        self.prefix = os.environ.get("MQTT_PREFIX", "rosie")

        # Process handles
        self.bridge_proc = None
        self.mode_proc = None
        self.current_mode = None  # "nav", "slam", or None

        # Pipeline state
        self.pipeline_status = IDLE
        self.pipeline_lock = threading.Lock()

        # Robot state (updated via MQTT)
        self.robot_ui_state = ""
        self.robot_ext_power = False

        # Pose tracking
        self.robot_pose = None   # (x, y, theta) in map frame from SLAM
        self._pose_stamp = 0.0   # monotonic time of last pose update
        self._odom_pose = None   # (x, y, theta) raw odometry fallback
        self._odom_stamp = 0.0   # monotonic time of last odom update
        self.dock_pose = None    # (x, y) in map frame, captured when docked
        self._last_dock_save = 0.0   # debounce dock pose writes
        self._was_docked = True      # assume docked at startup
        self._load_dock_pose()

        # Map overlay
        self._base_map_img = None   # PIL Image of the clean map (no markers)
        self._map_meta = None       # dict from home_meta.json
        self._last_map_refresh = 0
        self._last_overlay_pose = None  # avoid republish if nothing moved

        # No-go lines (list of {"p1": [x,y], "p2": [x,y]})
        self.nogo_lines = []

        # MQTT client
        self.client = mqtt.Client("rosie_server")

        # Diagnostics timing
        self._last_diag_publish = 0.0

        # Shutdown flag
        self._shutdown = threading.Event()

    # ── Process Management ────────────────────────────────────────

    def _start_bridge(self):
        """Launch the core driver + robot_state_publisher + Foxglove."""
        cmd = [
            "ros2", "launch", "rosie_bringup", "rosie_bringup.launch.py",
        ]
        self.bridge_proc = subprocess.Popen(cmd, start_new_session=True)
        log.info("Bridge started (PID %d)", self.bridge_proc.pid)

    def _start_nav(self):
        """Launch Nav2 for autonomous navigation."""
        self._stop_mode()
        cmd = [
            "ros2", "launch", "rosie_bringup", "navigation.launch.py",
            "map:=/ros2_ws/maps/home.yaml",
        ]
        self.mode_proc = subprocess.Popen(cmd, start_new_session=True)
        self.current_mode = "nav"
        log.info("Nav2 started (PID %d)", self.mode_proc.pid)
        if self.dock_pose:
            self._publish_initial_pose(self.dock_pose[0], self.dock_pose[1], delay=12)
        else:
            log.warning("No dock pose saved — AMCL will start at (0, 0)")

    def _start_slam(self):
        """Launch SLAM Toolbox for mapping."""
        self._stop_mode()
        cmd = [
            "ros2", "launch", "rosie_bringup", "slam.launch.py",
        ]
        self.mode_proc = subprocess.Popen(cmd, start_new_session=True)
        self.current_mode = "slam"
        log.info("SLAM started (PID %d)", self.mode_proc.pid)

    def _stop_mode(self):
        """Stop the current SLAM or Nav2 process group."""
        if self.mode_proc and self.mode_proc.poll() is None:
            log.info("Stopping %s (PID %d)", self.current_mode, self.mode_proc.pid)
            try:
                pgid = os.getpgid(self.mode_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                self.mode_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                self.mode_proc.wait()
            except ProcessLookupError:
                pass
        self.mode_proc = None
        self.current_mode = None

    # ── MQTT ──────────────────────────────────────────────────────

    def _load_dock_pose(self):
        """Load saved dock position from disk."""
        if DOCK_JSON.exists():
            try:
                data = json.loads(DOCK_JSON.read_text())
                self.dock_pose = (data["x"], data["y"])
                log.info("Loaded dock pose: (%.3f, %.3f)", *self.dock_pose)
            except Exception as e:
                log.warning("Failed to load dock pose: %s", e)

    def _save_dock_pose(self, x, y):
        """Persist dock position to disk."""
        self.dock_pose = (x, y)
        self._last_dock_save = time.monotonic()
        DOCK_JSON.parent.mkdir(parents=True, exist_ok=True)
        DOCK_JSON.write_text(json.dumps({"x": x, "y": y}))
        log.info("Saved dock pose: (%.3f, %.3f)", x, y)
        # Re-publish base map so hal_tester gets updated dock position
        self._publish_map_base()

    def _publish_initial_pose(self, x, y, yaw=0.0, delay=0):
        """Publish initial pose to AMCL. Use delay>0 when Nav2 is still starting up."""
        def _do_publish():
            if delay:
                time.sleep(delay)
            if self.current_mode != "nav":
                log.info("Skipping initial pose publish — mode changed to %s", self.current_mode)
                return
            # Encode yaw as quaternion (rotation around Z axis only)
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose_yaml = (
                f"{{header: {{frame_id: map}}, "
                f"pose: {{pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
                f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}, "
                f"covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
                f"0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
                f"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                f"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                f"0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
                f"0.0, 0.0, 0.0, 0.0, 0.0, 0.068]}}}}"
            )
            cmd = [
                "ros2", "topic", "pub", "--times", "3",
                "/initialpose",
                "geometry_msgs/msg/PoseWithCovarianceStamped",
                pose_yaml,
            ]
            try:
                subprocess.run(cmd, timeout=30, check=False)
                log.info("Initial pose published to AMCL: (%.3f, %.3f, yaw=%.3f)", x, y, yaw)
            except Exception as e:
                log.warning("Failed to publish initial pose: %s", e)
        threading.Thread(target=_do_publish, daemon=True).start()

    def _load_map_overlay_data(self):
        """Load the base map image and metadata for marker overlay."""
        if MAP_CLEAN_PNG.exists() and MAP_META_JSON.exists():
            try:
                self._base_map_img = Image.open(MAP_CLEAN_PNG).convert("RGB")
                self._map_meta = json.loads(MAP_META_JSON.read_text())
                log.info("Loaded base map (%dx%d) + meta for overlay",
                         self._base_map_img.width, self._base_map_img.height)
                self._publish_map_base()
            except Exception as e:
                log.warning("Failed to load map overlay data: %s", e)
                self._base_map_img = None
                self._map_meta = None

    def _publish_map_meta(self):
        """Publish map metadata JSON to MQTT for client coordinate transforms."""
        if self._map_meta is None:
            return
        payload = dict(self._map_meta)
        payload["nogo_lines"] = self.nogo_lines
        self.client.publish(
            f"{self.prefix}/map_meta",
            json.dumps(payload), qos=1, retain=True,
        )
        log.info("Map metadata published to MQTT")

    def _connect_mqtt(self):
        self.client.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.connect(self.mqtt_host, self.mqtt_port, 60)

    def _on_connect(self, client, userdata, flags, rc):
        log.info("MQTT connected (rc=%d)", rc)
        client.subscribe(f"{self.prefix}/command", qos=1)
        client.subscribe(f"{self.prefix}/state", qos=0)
        client.subscribe(f"{self.prefix}/battery", qos=0)
        client.subscribe(f"{self.prefix}/pose", qos=0)
        client.subscribe(f"{self.prefix}/odom", qos=0)
        client.subscribe(f"{self.prefix}/nogo_lines", qos=1)
        # Clear any old broken camera discovery config
        client.publish("homeassistant/camera/rosie_map/config", "", qos=1, retain=True)
        self._publish_ha_discovery()
        self._publish_status(self.pipeline_status)
        # Load base map + meta for live overlay
        self._load_map_overlay_data()
        self._publish_map_meta()
        # Publish map with markers if available
        self._publish_map_with_markers()

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace")

        if topic == f"{self.prefix}/command":
            if payload == "create_map":
                threading.Thread(
                    target=self._run_pipeline, daemon=True
                ).start()
            elif payload == "reboot":
                log.warning("Reboot requested from HA — rebooting Pi in 3 seconds")
                subprocess.Popen(["sudo", "reboot"])
            elif payload == "start":
                # Robot is definitely at the dock when a clean is commanded —
                # publish initial pose immediately so AMCL is correct from the start
                if self.dock_pose and self.current_mode == "nav":
                    log.info("Clean start command — publishing initial pose to AMCL")
                    self._publish_initial_pose(self.dock_pose[0], self.dock_pose[1])
        elif topic == f"{self.prefix}/state":
            try:
                data = json.loads(payload)
                self.robot_ui_state = data.get("ui_state", "")
            except (json.JSONDecodeError, TypeError):
                pass
        elif topic == f"{self.prefix}/battery":
            try:
                data = json.loads(payload)
                self.robot_ext_power = data.get("ext_power", False)
            except (json.JSONDecodeError, TypeError):
                pass
        elif topic == f"{self.prefix}/pose":
            try:
                data = json.loads(payload)
                x = float(data["x"])
                y = float(data["y"])
                theta = float(data.get("theta", 0))
                self.robot_pose = (x, y, theta)
                self._pose_stamp = time.monotonic()
                docked_now = self._is_docked()
                self._was_docked = docked_now
                # Update dock position whenever robot is docked (debounced to 60s)
                if docked_now and time.monotonic() - self._last_dock_save > 60:
                    self._save_dock_pose(x, y)
            except (json.JSONDecodeError, TypeError, KeyError, ValueError):
                pass
        elif topic == f"{self.prefix}/odom":
            try:
                data = json.loads(payload)
                x = float(data["x"])
                y = float(data["y"])
                theta = float(data.get("theta", 0))
                self._odom_pose = (x, y, theta)
                self._odom_stamp = time.monotonic()
                # Use odom as robot_pose when SLAM pose is stale (>15s)
                if time.monotonic() - self._pose_stamp > 15:
                    self.robot_pose = self._odom_pose
            except (json.JSONDecodeError, TypeError, KeyError, ValueError):
                pass
        elif topic == f"{self.prefix}/nogo_lines":
            try:
                data = json.loads(payload)
                lines = data.get("lines", data) if isinstance(data, dict) else data
                if isinstance(lines, list):
                    self.nogo_lines = lines
                    log.info("No-go lines updated: %d line(s)", len(lines))
                    self._last_overlay_pose = None  # force map refresh
                    self._publish_map_meta()        # update retained meta with new lines
            except (json.JSONDecodeError, TypeError, ValueError):
                pass

    def _publish_status(self, status, detail=""):
        self.pipeline_status = status
        payload = json.dumps({"status": status, "detail": detail})
        self.client.publish(
            f"{self.prefix}/map_pipeline/status",
            payload, qos=1, retain=True,
        )
        log.info("Pipeline: %s %s", status, detail)

    # ── HA Discovery ──────────────────────────────────────────────

    def _publish_ha_discovery(self):
        """Register pipeline entities with HA via MQTT discovery."""
        # Button: Create New Map
        self._pub_discovery("button", "create_map", {
            "name": "Create New Map",
            "unique_id": "rosie_create_map",
            "command_topic": f"{self.prefix}/command",
            "payload_press": "create_map",
            "icon": "mdi:map-plus",
        })

        # Button: Reboot Pi
        self._pub_discovery("button", "reboot", {
            "name": "Reboot Pi",
            "unique_id": "rosie_reboot",
            "command_topic": f"{self.prefix}/command",
            "payload_press": "reboot",
            "icon": "mdi:restart",
            "entity_category": "config",
        })

        # Sensor: Pipeline Status
        self._pub_discovery("sensor", "map_pipeline_status", {
            "name": "Map Pipeline",
            "unique_id": "rosie_map_pipeline_status",
            "state_topic": f"{self.prefix}/map_pipeline/status",
            "value_template": "{{ value_json.status }}",
            "json_attributes_topic": f"{self.prefix}/map_pipeline/status",
            "icon": "mdi:map-clock",
        })

        # Camera: Map Image (base64 PNG on MQTT)
        self._pub_discovery("camera", "map", {
            "name": "ROSie Map",
            "unique_id": "rosie_map",
            "topic": f"{self.prefix}/map_image",
            "image_encoding": "b64",
            "icon": "mdi:floor-plan",
        })

        # Sensor: Pi CPU Load
        self._pub_discovery("sensor", "cpu_load", {
            "name": "Pi CPU Load",
            "unique_id": "rosie_cpu_load",
            "state_topic": f"{self.prefix}/diagnostics",
            "value_template": "{{ value_json.cpu_percent }}",
            "unit_of_measurement": "%",
            "state_class": "measurement",
            "icon": "mdi:cpu-64-bit",
        })

        # Sensor: Pi RAM (combined — "868 MB (25.6%)")
        self._pub_discovery("sensor", "ram", {
            "name": "Pi RAM",
            "unique_id": "rosie_ram",
            "state_topic": f"{self.prefix}/diagnostics",
            "value_template": "{{ value_json.ram_used_mb }} MB used ({{ value_json.ram_percent }}%)",
            "icon": "mdi:memory",
        })

        # Remove old split RAM sensors from HA (publish empty retain to clear)
        for old_id in ("ram_used", "ram_percent"):
            self.client.publish(
                f"homeassistant/sensor/rosie_{old_id}/config",
                "", qos=1, retain=True,
            )

        # Sensor: Map Metadata (JSON for coordinate transforms)
        self._pub_discovery("sensor", "map_meta", {
            "name": "Map Metadata",
            "unique_id": "rosie_map_meta",
            "state_topic": f"{self.prefix}/map_meta",
            "value_template": "{{ 'available' if value_json is defined else 'unavailable' }}",
            "json_attributes_topic": f"{self.prefix}/map_meta",
            "icon": "mdi:map-legend",
        })

    def _pub_discovery(self, component, object_id, config):
        config.setdefault("device", _DEVICE)
        config.setdefault("availability", [{
            "topic": f"{self.prefix}/availability",
            "payload_available": "online",
            "payload_not_available": "offline",
        }])
        self.client.publish(
            f"homeassistant/{component}/rosie_{object_id}/config",
            json.dumps(config), qos=1, retain=True,
        )

    # ── Map Pipeline ──────────────────────────────────────────────

    def _run_pipeline(self):
        """Entry point for the create-new-map pipeline (runs in thread)."""
        with self.pipeline_lock:
            if self.pipeline_status not in (IDLE, DONE, ERROR):
                log.warning("Pipeline already running, ignoring create_map")
                return
            try:
                self._pipeline_impl()
            except Exception as e:
                log.exception("Pipeline failed")
                self._publish_status(ERROR, str(e))
                # Restore nav mode on failure
                time.sleep(2)
                self._start_nav()

    def _pipeline_impl(self):
        # 1. Clear old map — remove stale files and blank the camera
        self._publish_status(SWITCHING_TO_SLAM)
        self._clear_old_map()

        # 2. Switch to SLAM
        self._stop_mode()
        time.sleep(2)
        self._start_slam()
        time.sleep(5)  # let SLAM initialise

        # SLAM starts with robot at (0,0) — that's the dock position
        self._save_dock_pose(0.0, 0.0)

        # 3. Start cleaning (robot auto-activates LDS during cleaning)
        self._publish_status(CLEANING)
        # Clear stale UI state so was_cleaning doesn't trigger from old
        # retained MQTT value.  Do NOT clear robot_ext_power — that races
        # with the MQTT callback and causes false undock detection.
        self.robot_ui_state = ""
        self.client.publish(f"{self.prefix}/command", "start", qos=1)

        # 3a. Wait for robot to leave the dock before monitoring return
        log.info("Waiting for robot to undock...")
        undock_timeout = 120  # 2 minutes to leave dock
        undock_start = time.time()
        while time.time() - undock_start < undock_timeout:
            if self._shutdown.is_set():
                raise RuntimeError("Shutdown requested during pipeline")
            if not self._is_docked():
                log.info("Robot has left the dock")
                break
            time.sleep(2)
        else:
            log.warning("Robot did not undock within %ds, continuing anyway",
                        undock_timeout)

        # 4. Wait for robot to finish cleaning and return to dock
        self._publish_status(WAITING_FOR_DOCK)
        start_time = time.time()
        was_cleaning = False
        last_snapshot = 0  # timestamp of last map snapshot
        first_snapshot_delay = 10   # first snapshot after 10s
        snapshot_interval = 10      # subsequent snapshots every 10s
        # Require at least 60s after undock before accepting "docked" to
        # prevent a stale MQTT battery update from causing false dock-return.
        min_dock_time = time.time() + 60

        while time.time() - start_time < PIPELINE_TIMEOUT:
            if self._shutdown.is_set():
                raise RuntimeError("Shutdown requested during pipeline")

            if "CLEANING" in self.robot_ui_state.upper():
                was_cleaning = True

            if was_cleaning and self._is_docked() and time.time() > min_dock_time:
                log.info("Robot returned to dock after cleaning (elapsed %.0fs)",
                         time.time() - start_time)
                time.sleep(10)  # let final scans arrive for SLAM
                break

            # Publish live map snapshots while cleaning
            now = time.time()
            delay = first_snapshot_delay if last_snapshot == 0 else snapshot_interval
            if was_cleaning and now - (last_snapshot or start_time) >= delay:
                last_snapshot = now
                self._publish_live_snapshot()

            time.sleep(2)
        else:
            raise RuntimeError("Cleaning timeout (2h)")

        # 4. Save map
        self._publish_status(SAVING_MAP)
        result = subprocess.run(
            [
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", "/ros2_ws/maps/home",
                "--ros-args", "-p", "save_map_timeout:=10000.0",
            ],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode != 0:
            raise RuntimeError(f"Map save failed: {result.stderr}")
        log.info("Map saved: %s", result.stdout.strip())

        # 5. Process map (clean up for display)
        self._publish_status(PROCESSING)
        from clean_map import clean_map
        clean_map(
            yaml_path=MAP_YAML,
            out_path=MAP_CLEAN_PNG,
        )

        # 6. Publish cleaned map image to MQTT (with markers)
        self._publish_status(UPLOADING)
        self._load_map_overlay_data()
        self._publish_map_meta()
        self._publish_map_with_markers()

        # 7. Switch back to Nav2
        self._publish_status(SWITCHING_TO_NAV)
        self._stop_mode()
        time.sleep(2)
        self._start_nav()

        self._publish_status(DONE, "Map updated successfully")

    def _clear_old_map(self):
        """Delete old map files and publish a blank camera image."""
        # Clear no-go lines and publish updated meta before nulling it,
        # so the retained rosie/map_meta message on the broker gets
        # updated with an empty nogo_lines list.
        self.nogo_lines = []
        if self._map_meta is not None:
            self._publish_map_meta()
        for f in (MAP_CLEAN_PNG, MAP_META_JSON):
            if f.exists():
                f.unlink()
                log.info("Deleted %s", f)
        # Clear in-memory overlay data
        self._base_map_img = None
        self._map_meta = None
        self._last_overlay_pose = None
        # Publish a small blank image to clear the camera entity
        blank = Image.new("RGB", (200, 200), BG_COLOUR)
        draw = ImageDraw.Draw(blank)
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
        except (OSError, IOError):
            font = ImageFont.load_default()
        draw.text((30, 85), "Mapping...", fill=(120, 130, 140), font=font)
        buf = io.BytesIO()
        blank.save(buf, "PNG")
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        self.client.publish(
            f"{self.prefix}/map_image", b64, qos=1, retain=True)
        self.client.publish(
            f"{self.prefix}/map_image_base", b64, qos=1, retain=True)
        log.info("Old map cleared, blank placeholder published")

    def _is_docked(self):
        """Check if robot is on the charger."""
        if self.robot_ext_power:
            return True
        ui_upper = self.robot_ui_state.upper()
        for kw in ("DOCKED", "CHARGING", "BASE"):
            if kw in ui_upper:
                return True
        return False

    def _publish_live_snapshot(self):
        """Save + clean + publish the current SLAM map as a live preview."""
        try:
            result = subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", "/ros2_ws/maps/home",
                    "--ros-args", "-p", "save_map_timeout:=10000.0",
                ],
                capture_output=True, text=True, timeout=30,
            )
            if result.returncode != 0:
                log.warning("Live snapshot save failed: %s", result.stderr.strip())
                return
            from clean_map import clean_map
            clean_map(
                yaml_path=MAP_YAML,
                out_path=MAP_CLEAN_PNG,
            )
            self._load_map_overlay_data()
            self._publish_map_with_markers()
            log.info("Live map snapshot published")
        except Exception as e:
            log.warning("Live snapshot failed: %s", e)

    def _publish_map_image(self, path):
        """Publish a PNG image as base64 to MQTT for HA camera entity."""
        data = Path(path).read_bytes()
        b64 = base64.b64encode(data).decode("ascii")
        self.client.publish(
            f"{self.prefix}/map_image",
            b64, qos=1, retain=True,
        )
        log.info("Map image published to MQTT (%d bytes, %d b64)", len(data), len(b64))

    def _publish_map_base(self):
        """Publish dock-only base map (no robot marker) to rosie/map_image_base.

        This is the background image for real-time overlays like the HAL tester.
        It only updates when the base map or dock position changes.
        """
        if self._base_map_img is None or self._map_meta is None:
            return
        try:
            from clean_map import map_to_pixel

            meta = self._map_meta
            img = self._base_map_img.copy()
            draw = ImageDraw.Draw(img)
            scale = meta["scale"]

            try:
                icon_font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                    int(scale * 3 * 1.2))
            except (OSError, IOError):
                icon_font = ImageFont.load_default()
            try:
                label_font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                    max(10, scale * 2))
            except (OSError, IOError):
                label_font = ImageFont.load_default()

            conv_args = (
                meta["origin"], meta["resolution"], meta["map_h"],
                meta["angle"], meta["scale"],
                meta["pre_rot_w"], meta["pre_rot_h"],
                meta["post_rot_w"], meta["post_rot_h"],
                meta["crop_r"], meta["crop_c"], meta["border"],
            )

            if self.dock_pose:
                dx, dy = map_to_pixel(self.dock_pose[0], self.dock_pose[1],
                                      *conv_args)
                r = scale * 3
                if 0 <= dx < img.width and 0 <= dy < img.height:
                    draw.ellipse([dx - r, dy - r, dx + r, dy + r],
                                 fill=DOCK_COLOUR, outline=(20, 120, 60),
                                 width=max(2, scale // 2))
                    icon = "\u2302"
                    bbox = draw.textbbox((0, 0), icon, font=icon_font)
                    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                    draw.text((dx - tw // 2, dy - th // 2 - bbox[1]),
                              icon, fill=(255, 255, 255), font=icon_font)
                    label = "Base"
                    lbox = draw.textbbox((0, 0), label, font=label_font)
                    lw = lbox[2] - lbox[0]
                    draw.text((dx - lw // 2, dy + r + 2),
                              label, fill=DOCK_COLOUR, font=label_font)

            buf = io.BytesIO()
            img.save(buf, "PNG", optimize=True)
            b64 = base64.b64encode(buf.getvalue()).decode("ascii")
            self.client.publish(
                f"{self.prefix}/map_image_base",
                b64, qos=1, retain=True,
            )
            log.info("Base map published to map_image_base (%d b64)", len(b64))
        except Exception:
            log.exception("Failed to publish base map")

    def _publish_map_with_markers(self):
        """Overlay dock + robot markers on the base map and publish."""
        if self._base_map_img is None or self._map_meta is None:
            # No base map yet — try publishing raw image if it exists
            if MAP_CLEAN_PNG.exists():
                self._publish_map_image(MAP_CLEAN_PNG)
            return

        try:
            from clean_map import map_to_pixel

            meta = self._map_meta
            img = self._base_map_img.copy()
            draw = ImageDraw.Draw(img)
            scale = meta["scale"]

            # Load fonts
            try:
                icon_font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                    int(scale * 3 * 1.2))
            except (OSError, IOError):
                icon_font = ImageFont.load_default()
            try:
                label_font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                    max(10, scale * 2))
            except (OSError, IOError):
                label_font = ImageFont.load_default()

            conv_args = (
                meta["origin"], meta["resolution"], meta["map_h"],
                meta["angle"], meta["scale"],
                meta["pre_rot_w"], meta["pre_rot_h"],
                meta["post_rot_w"], meta["post_rot_h"],
                meta["crop_r"], meta["crop_c"], meta["border"],
            )

            # Draw dock marker
            if self.dock_pose:
                dx, dy = map_to_pixel(self.dock_pose[0], self.dock_pose[1],
                                      *conv_args)
                r = scale * 3
                if 0 <= dx < img.width and 0 <= dy < img.height:
                    draw.ellipse([dx - r, dy - r, dx + r, dy + r],
                                 fill=DOCK_COLOUR, outline=(20, 120, 60),
                                 width=max(2, scale // 2))
                    icon = "\u2302"
                    bbox = draw.textbbox((0, 0), icon, font=icon_font)
                    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
                    draw.text((dx - tw // 2, dy - th // 2 - bbox[1]),
                              icon, fill=(255, 255, 255), font=icon_font)
                    label = "Base"
                    lbox = draw.textbbox((0, 0), label, font=label_font)
                    lw = lbox[2] - lbox[0]
                    draw.text((dx - lw // 2, dy + r + 2),
                              label, fill=DOCK_COLOUR, font=label_font)

            # No-go lines are rendered client-side by the HA card overlay
            # (rosie-nogo-editor-card.js) for instant visual feedback.

            # Draw robot marker (snap to dock when docked)
            draw_pose = self.robot_pose
            if self._is_docked() and self.dock_pose:
                draw_pose = (self.dock_pose[0], self.dock_pose[1], 0)
            if draw_pose:
                rx, ry = map_to_pixel(draw_pose[0], draw_pose[1],
                                      *conv_args)
                r = scale * 2
                if 0 <= rx < img.width and 0 <= ry < img.height:
                    draw.ellipse([rx - r, ry - r, rx + r, ry + r],
                                 fill=ROBOT_COLOUR, outline=(20, 70, 200),
                                 width=max(2, scale // 2))
                    theta = draw_pose[2]
                    adj_theta = theta + math.radians(meta["angle"])
                    tri_len = r * 1.8
                    tip_x = rx + tri_len * math.cos(adj_theta)
                    tip_y = ry - tri_len * math.sin(adj_theta)
                    left_x = rx + r * 0.6 * math.cos(adj_theta + 2.4)
                    left_y = ry - r * 0.6 * math.sin(adj_theta + 2.4)
                    right_x = rx + r * 0.6 * math.cos(adj_theta - 2.4)
                    right_y = ry - r * 0.6 * math.sin(adj_theta - 2.4)
                    draw.polygon([(tip_x, tip_y), (left_x, left_y),
                                  (right_x, right_y)],
                                 fill=(255, 255, 255))

            # Encode and publish
            buf = io.BytesIO()
            img.save(buf, "PNG", optimize=True)
            b64 = base64.b64encode(buf.getvalue()).decode("ascii")
            self.client.publish(
                f"{self.prefix}/map_image",
                b64, qos=1, retain=True,
            )
            log.info("Map with markers published (%d b64, dock=%s, robot=%s)",
                     len(b64), self.dock_pose is not None,
                     self.robot_pose is not None)
        except Exception:
            log.exception("Failed to publish map with markers")

    def _publish_diagnostics(self):
        """Publish CPU and RAM metrics to MQTT every 30 seconds."""
        now = time.monotonic()
        if now - self._last_diag_publish < 30.0:
            return
        self._last_diag_publish = now
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory()
        payload = {
            "cpu_percent": round(cpu, 1),
            "ram_used_mb": round(ram.used / 1024 / 1024),
            "ram_percent": round(ram.percent, 1),
            "ram_total_mb": round(ram.total / 1024 / 1024),
        }
        self.client.publish(
            f"{self.prefix}/diagnostics",
            json.dumps(payload), qos=0, retain=True,
        )

    def _maybe_refresh_map(self):
        """Called from the main loop — refresh map overlay periodically."""
        # Don't fight with the pipeline — coordinates won't match
        if self.pipeline_status not in (IDLE, DONE, ERROR):
            return

        now = time.time()
        if now - self._last_map_refresh < MAP_REFRESH_INTERVAL:
            return
        self._last_map_refresh = now

        if self._base_map_img is None:
            return

        # Only republish if pose, lines, or docked state changed
        docked = self._is_docked()
        current = (self.robot_pose, self.dock_pose, docked,
                   tuple(str(l) for l in self.nogo_lines))
        if current == self._last_overlay_pose:
            return
        self._last_overlay_pose = current
        self._publish_map_with_markers()

    # ── Main Loop ─────────────────────────────────────────────────

    def run(self):
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

        log.info("Starting ROSie Server")
        self._start_bridge()
        time.sleep(3)  # let bridge settle

        # Start Nav2 by default (if map exists)
        if MAP_YAML.exists():
            self._start_nav()
        else:
            log.warning("No map found at %s — starting in idle mode", MAP_YAML)

        self._connect_mqtt()

        # Run MQTT loop until shutdown
        while not self._shutdown.is_set():
            self.client.loop(timeout=1.0)
            self._maybe_refresh_map()
            self._publish_diagnostics()

        self._cleanup()

    def _handle_signal(self, signum, frame):
        log.info("Signal %d received, shutting down", signum)
        self._shutdown.set()

    def _cleanup(self):
        """Gracefully shut down all child processes."""
        self._stop_mode()
        if self.bridge_proc and self.bridge_proc.poll() is None:
            try:
                pgid = os.getpgid(self.bridge_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                self.bridge_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                self.bridge_proc.wait()
            except ProcessLookupError:
                pass
        try:
            self.client.disconnect()
        except Exception:
            pass
        log.info("Shutdown complete")


if __name__ == "__main__":
    RosieServer().run()
