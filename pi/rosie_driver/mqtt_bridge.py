"""
mqtt_bridge.py — Publish sensor data and subscribe to commands over MQTT.

Topic scheme (all under configurable prefix, default 'rosie'):
  Publish:
    {prefix}/scan         — LIDAR scan as JSON
    {prefix}/odom         — Odometry as JSON
    {prefix}/battery      — Battery state as JSON
    {prefix}/bumpers      — Bumper state as JSON
    {prefix}/state        — Robot state as JSON
    {prefix}/settings     — User settings as JSON
    {prefix}/spot_config  — Spot clean dimensions as JSON
    {prefix}/nogo_lines   — Active no-go lines as JSON
    {prefix}/nogo_status  — Last no-go update status as JSON
    {prefix}/availability — 'online' / LWT 'offline'

  Subscribe:
    {prefix}/command          — All commands (buttons, HA vacuum, etc.)
    {prefix}/cmd_vel          — Twist velocity commands as JSON
    {prefix}/settings/+/set   — Switch toggle commands
    {prefix}/spot_width/set   — Spot width number
    {prefix}/spot_height/set  — Spot height number
    {prefix}/nav_mode/set     — Navigation mode select
    {prefix}/nogo_lines/set   — Update no-go lines (JSON)
"""

import json
import logging
import math
import threading
import time
from typing import Any, Callable, Optional

import paho.mqtt.client as mqtt

logger = logging.getLogger(__name__)

# Device block shared by all entities. Model and sw_version are placeholders
# until update_device_info() is called with real values from GetVersion.
_DEFAULT_DEVICE = {
    "identifiers": ["rosie_neato_d6"],
    "name": "ROSie",
    "manufacturer": "Neato Robotics",
    "model": "BotVac (detecting...)",
    "sw_version": "rosie-driver 0.2.0",
}


class MQTTBridge:
    """Bidirectional MQTT bridge for ROSie."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 1883,
        username: Optional[str] = None,
        password: Optional[str] = None,
        prefix: str = "rosie",
        name: Optional[str] = None,
    ):
        self._host = host
        self._port = port
        self._prefix = prefix

        # Per-instance device dict so it can be updated from robot version info
        self._device = dict(_DEFAULT_DEVICE)
        if name:
            self._device["name"] = name

        self._client = mqtt.Client(
            client_id="rosie-pi-driver",
            protocol=mqtt.MQTTv311,
        )

        if username:
            self._client.username_pw_set(username, password)

        # Last-will: mark offline on ungraceful disconnect
        self._client.will_set(
            f"{self._prefix}/availability",
            payload="offline",
            qos=1,
            retain=True,
        )

        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message

        # Callbacks set by the main loop
        self._command_callback: Optional[Callable[[str], None]] = None
        self._cmd_vel_callback: Optional[Callable[[float, float], None]] = None
        self._nogo_lines_callback: Optional[
            Callable[[list], tuple[bool, str, list[dict[str, list[float]]]]]
        ] = None
        self._nogo_tuning_callback: Optional[Callable[[str, float], None]] = None
        self._slam_pose_callback: Optional[Callable] = None

        # Latest SLAM-corrected pose from ROS 2 (map frame)
        # Stored as (x, y, theta, receive_stamp, meta_dict)
        self._slam_pose: Optional[tuple] = None
        self._slam_lock = threading.Lock()

        # Local state for spot config & settings (for HA number/select entities)
        self._spot_width = 200
        self._spot_height = 200
        self._nav_mode = "Normal"
        self._vacuum_on = False
        self._vacuum_speed = 65
        self._vacuum_rpm = 0
        self._vacuum_ma = 0

        # Pending setting overrides: {setting_key: (value_bool, timestamp)}
        # Prevents poll from clobbering a recently-sent toggle.
        self._settings_overrides: dict[str, tuple[bool, float]] = {}
        self._SETTINGS_OVERRIDE_TTL = 10.0  # seconds

        # Last published settings dict (for immediate republish on toggle)
        self._last_settings: dict[str, str] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_command_callback(self, cb: Callable[[str], None]) -> None:
        self._command_callback = cb

    def set_cmd_vel_callback(self, cb: Callable[[float, float], None]) -> None:
        self._cmd_vel_callback = cb

    def set_nogo_lines_callback(
        self,
        cb: Callable[[list], tuple[bool, str, list[dict[str, list[float]]]]],
    ) -> None:
        self._nogo_lines_callback = cb

    def set_nogo_tuning_callback(
        self,
        cb: Callable[[str, float], None],
    ) -> None:
        self._nogo_tuning_callback = cb

    def set_slam_pose_callback(
        self,
        cb: Callable,
    ) -> None:
        self._slam_pose_callback = cb

    def publish_nogo_tuning(self, tuning: dict) -> None:
        self.publish("nogo_tuning", tuning, retain=True)

    def publish_nogo_debug(self, snapshot: dict) -> None:
        self.publish("nogo_debug", snapshot, retain=False)

    def publish_bumper_event(self, record: dict) -> None:
        self.publish("bumper_event", record, retain=False)

    def get_slam_pose(self) -> Optional[tuple]:
        """Return (x, y, theta, age_s, meta) or None if not yet available."""
        with self._slam_lock:
            if self._slam_pose is None:
                return None
            sx, sy, sth, stamp, meta = self._slam_pose
        return (sx, sy, sth, time.monotonic() - stamp, meta)

    def connect(self) -> None:
        logger.info("Connecting to MQTT broker %s:%d", self._host, self._port)
        self._client.connect(self._host, self._port, keepalive=60)
        self._client.loop_start()

    def disconnect(self) -> None:
        self.publish("availability", "offline", retain=True)
        self._client.loop_stop()
        self._client.disconnect()
        logger.info("Disconnected from MQTT broker")

    def publish(self, topic_suffix: str, payload, qos: int = 0,
                retain: bool = False) -> None:
        full_topic = f"{self._prefix}/{topic_suffix}"
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload)
        self._client.publish(full_topic, payload, qos=qos, retain=retain)

    # ------------------------------------------------------------------
    # Typed publish helpers
    # ------------------------------------------------------------------

    def publish_scan(self, ranges: list[float], intensities: list[float],
                     angle_min: float, angle_max: float,
                     angle_increment: float,
                     range_min: float, range_max: float,
                     rpm: float, stamp: float) -> None:
        safe_ranges = [r if math.isfinite(r) else 0.0 for r in ranges]
        safe_intensities = [i if math.isfinite(i) else 0.0 for i in intensities]
        self.publish("scan", {
            "ranges": safe_ranges,
            "intensities": safe_intensities,
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_increment": angle_increment,
            "range_min": range_min,
            "range_max": range_max,
            "rpm": rpm,
            "stamp": stamp,
        })

    def publish_odom(self, x: float, y: float, theta: float,
                     linear_vel: float, angular_vel: float,
                     stamp: float) -> None:
        self.publish("odom", {
            "x": x, "y": y, "theta": theta,
            "linear_vel": linear_vel, "angular_vel": angular_vel,
            "stamp": stamp,
        })

    def publish_battery(self, fuel_percent: float, voltage: float,
                        charging: bool, ext_power: bool,
                        temperature: float) -> None:
        self.publish("battery", {
            "fuel_percent": fuel_percent,
            "voltage": round(voltage, 2),
            "charging": charging,
            "ext_power": ext_power,
            "temperature": round(temperature, 1),
        }, retain=True)

    def publish_bumpers(self, left_front: bool, right_front: bool,
                        left_side: bool, right_side: bool) -> None:
        self.publish("bumpers", {
            "left_front":  left_front,  "right_front": right_front,
            "left_side":   left_side,   "right_side":  right_side,
        })

    def publish_state(self, ui_state: str, robot_state: str,
                      error: str, alert: str) -> None:
        if "UI_ALERT_INVALID" in error or "UI_ALERT_INVALID" in alert:
            error = "none"
            alert = "none"

        ha_state = self._map_vacuum_state(ui_state, error)
        self.publish("state", {
            "state": ha_state,
            "ui_state": ui_state,
            "robot_state": robot_state,
            "error": error,
            "alert": alert,
        }, retain=True)

    def _apply_overrides(self, **polled: bool) -> dict[str, bool]:
        """Apply pending overrides to polled settings values."""
        now = time.monotonic()
        expired = [k for k, (_, ts) in self._settings_overrides.items()
                   if now - ts > self._SETTINGS_OVERRIDE_TTL]
        for k in expired:
            del self._settings_overrides[k]
        result = dict(polled)
        for key, (val, _ts) in self._settings_overrides.items():
            if key in result:
                result[key] = val
        return result

    def publish_settings(self, eco_mode: bool, wall_enable: bool,
                         intense_clean: bool, click_sounds: bool,
                         melody_sounds: bool, warning_sounds: bool,
                         bin_full_detect: bool, led: bool,
                         nav_mode: str | None = None) -> None:
        s = self._apply_overrides(
            eco_mode=eco_mode, wall_enable=wall_enable,
            intense_clean=intense_clean, click_sounds=click_sounds,
            melody_sounds=melody_sounds, warning_sounds=warning_sounds,
            bin_full_detect=bin_full_detect, led=led,
        )
        payload = {
            "eco_mode": "ON" if s["eco_mode"] else "OFF",
            "wall_enable": "ON" if s["wall_enable"] else "OFF",
            "intense_clean": "ON" if s["intense_clean"] else "OFF",
            "click_sounds": "ON" if s["click_sounds"] else "OFF",
            "melody_sounds": "ON" if s["melody_sounds"] else "OFF",
            "warning_sounds": "ON" if s["warning_sounds"] else "OFF",
            "bin_full_detect": "ON" if s["bin_full_detect"] else "OFF",
            "led": "ON" if s["led"] else "OFF",
            "nav_mode": nav_mode if nav_mode is not None else self._nav_mode,
        }
        self._last_settings = payload
        self.publish("settings", payload, retain=True)

    def publish_spot_config(self) -> None:
        self.publish("spot_config", {
            "width": self._spot_width,
            "height": self._spot_height,
        }, retain=True)

    def _publish_vacuum_state(self) -> None:
        self.publish("vacuum_state", {
            "vacuum_on": "ON" if self._vacuum_on else "OFF",
            "vacuum_speed": self._vacuum_speed,
            "vacuum_rpm": self._vacuum_rpm,
            "vacuum_ma": self._vacuum_ma,
        }, retain=True)

    def publish_vacuum_motors(self, vacuum_rpm: int, vacuum_ma: int) -> None:
        """Update vacuum motor telemetry from GetMotors poll."""
        self._vacuum_rpm = vacuum_rpm
        self._vacuum_ma = vacuum_ma
        self._publish_vacuum_state()

    def publish_neato_sensors(
        self,
        wall_mm: float, drop_left_mm: float, drop_right_mm: float,
    ) -> None:
        self.publish("neato_sensors", {
            "wall_mm": wall_mm,
            "drop_left_mm": drop_left_mm,
            "drop_right_mm": drop_right_mm,
        }, retain=True)

    def publish_nogo_lines(self, lines: list[dict[str, list[float]]]) -> None:
        self.publish("nogo_lines", {"lines": lines}, retain=True)

    def publish_nogo_status(self, status: str, message: str) -> None:
        self.publish(
            "nogo_status",
            {"status": status, "message": message},
            retain=True,
        )

    @staticmethod
    def _map_vacuum_state(ui_state: str, error: str) -> str:
        if error and error != "none" and error != "":
            return "error"
        ui = ui_state.upper()
        if "PAUSED" in ui:
            return "paused"
        if "CLEANINGRUNNING" in ui or "STARTCLEAN" in ui or "STARTHOUSECLEAN" in ui or "STARTSPOTCLEAN" in ui:
            return "cleaning"
        if "GOTOBASE" in ui or "SENDTOBASE" in ui or "DOCKINGRUNNING" in ui:
            return "returning"
        if "DOCKED" in ui or "CHARGING" in ui:
            return "docked"
        return "idle"

    # ------------------------------------------------------------------
    # HA MQTT Discovery
    # ------------------------------------------------------------------

    def _availability(self):
        return [{"topic": f"{self._prefix}/availability",
                 "payload_available": "online",
                 "payload_not_available": "offline"}]

    def _pub_discovery(self, component: str, object_id: str, config: dict):
        """Publish a single HA MQTT discovery config."""
        config.setdefault("device", self._device)
        config.setdefault("availability", self._availability())
        self._client.publish(
            f"homeassistant/{component}/rosie_{object_id}/config",
            json.dumps(config), qos=1, retain=True,
        )

    def update_device_info(self, model: Optional[str] = None,
                           hw_version: Optional[str] = None) -> None:
        """Update the HA device block with values read from the robot.

        Republishes all HA discovery configs so the new model/firmware
        appears in the HA Devices panel without restarting HA.
        """
        changed = False
        if model and model != self._device.get("model"):
            self._device["model"] = model
            changed = True
        if hw_version and hw_version != self._device.get("hw_version"):
            self._device["hw_version"] = hw_version
            changed = True
        if changed:
            logger.info("Updated HA device info: model=%s hw=%s",
                        self._device.get("model"), self._device.get("hw_version"))
            try:
                self._publish_ha_discovery()
            except Exception as exc:  # noqa: BLE001
                logger.warning("Re-publishing HA discovery failed: %s", exc)

    def _publish_ha_discovery(self) -> None:
        """Publish all HA MQTT Discovery configs."""
        pfx = self._prefix

        # --- Vacuum entity ---
        self._pub_discovery("vacuum", "vacuum", {
            "name": None,
            "unique_id": "rosie_vacuum_v2",
            "object_id": "rosie",
            "command_topic": f"{pfx}/command",
            "payload_start": "start",
            "payload_stop": "stop",
            "payload_pause": "pause",
            "payload_return_to_base": "return_to_base",
            "payload_locate": "locate",
            "payload_clean_spot": "clean_spot",
            "state_topic": f"{pfx}/state",
            "value_template": "{{ value_json.state }}",
            "battery_level_topic": f"{pfx}/battery",
            "battery_level_template": "{{ value_json.fuel_percent | int }}",
            "charging_topic": f"{pfx}/battery",
            "charging_template": "{{ value_json.charging }}",
            "fan_speed_list": ["eco", "normal"],
            "set_fan_speed_topic": f"{pfx}/command",
            "json_attributes_topic": f"{pfx}/state",
            "icon": "mdi:robot-vacuum",
        })

        # --- Buttons: Cleaning ---
        buttons = [
            ("house_clean", "House Clean", "mdi:home"),
            ("spot_clean", "Spot Clean", "mdi:target"),
            ("spot_clean_hw", "Spot Clean (H×W)", "mdi:target"),
            ("stop_cleaning", "Stop Cleaning", "mdi:stop"),
            ("pause_cleaning", "Pause Cleaning", "mdi:pause"),
            ("resume_cleaning", "Resume Cleaning", "mdi:play"),
            ("send_to_base", "Send to Base", "mdi:home-import-outline"),
            ("locate", "Locate Robot", "mdi:volume-high"),
            ("start_manual_cleaning", "Start Manual Cleaning", "mdi:play-circle"),
            # Utility
            ("update_status", "Update Status", "mdi:refresh"),
            ("clear_errors", "Clear Errors", "mdi:notification-clear-all"),
            ("activate", "Activate LDS", "mdi:power"),
            ("deactivate", "Deactivate LDS", "mdi:power-off"),
            ("shutdown", "Shutdown Robot", "mdi:power"),
            ("powercycle", "Reboot Robot", "mdi:restart"),
            # Bumper test (diagnostic) — pulls the corresponding bumper GPIO
            # pin LOW for ~5s, faking a real bumper switch closure.
            ("test_bumper_fl", "Bumper Test: Front Left",  "mdi:gesture-tap"),
            ("test_bumper_fr", "Bumper Test: Front Right", "mdi:gesture-tap"),
            ("test_bumper_sl", "Bumper Test: Side Left",   "mdi:gesture-tap"),
            ("test_bumper_sr", "Bumper Test: Side Right",  "mdi:gesture-tap"),
        ]
        for btn_id, label, icon in buttons:
            payload = btn_id
            if btn_id == "spot_clean_hw":
                payload = f"spot_clean_hw:{self._spot_width},{self._spot_height}"
            cat = "diagnostic" if btn_id in (
                "update_status", "clear_errors", "activate", "deactivate",
                "shutdown", "powercycle",
                "test_bumper_fl", "test_bumper_fr",
                "test_bumper_sl", "test_bumper_sr",
            ) else None
            cfg = {
                "name": label,
                "unique_id": f"rosie_{btn_id}",
                "command_topic": f"{pfx}/command",
                "payload_press": payload,
                "icon": icon,
            }
            if cat:
                cfg["entity_category"] = cat
            self._pub_discovery("button", btn_id, cfg)

        # --- Switches: Settings ---
        switches = [
            ("eco_mode", "Eco Mode", "mdi:leaf"),
            ("wall_enable", "Wall Follower", "mdi:wall"),
            ("intense_clean", "Intense Clean", "mdi:broom"),
            ("click_sounds", "Click Sounds", "mdi:volume-medium"),
            ("melody_sounds", "Melody Sounds", "mdi:music"),
            ("warning_sounds", "Warning Sounds", "mdi:alert"),
            ("bin_full_detect", "Bin Full Detect", "mdi:delete-variant"),
            ("led", "LED", "mdi:led-on"),
        ]
        for sw_id, label, icon in switches:
            self._pub_discovery("switch", sw_id, {
                "name": label,
                "unique_id": f"rosie_{sw_id}",
                "command_topic": f"{pfx}/settings/{sw_id}/set",
                "state_topic": f"{pfx}/settings",
                "value_template": "{{ value_json." + sw_id + " }}",
                "payload_on": "ON",
                "payload_off": "OFF",
                "state_on": "ON",
                "state_off": "OFF",
                "icon": icon,
                "entity_category": "config",
            })

        # --- Number: Spot clean dimensions ---
        for dim_id, label in [("spot_width", "Spot Clean Width"),
                               ("spot_height", "Spot Clean Height")]:
            self._pub_discovery("number", dim_id, {
                "name": label,
                "unique_id": f"rosie_{dim_id}",
                "command_topic": f"{pfx}/{dim_id}/set",
                "state_topic": f"{pfx}/spot_config",
                "value_template": "{{ value_json." + dim_id.split('_')[1] + " }}",
                "min": 100,
                "max": 400,
                "step": 1,
                "unit_of_measurement": "cm",
                "mode": "slider",
                "icon": "mdi:arrow-left-right" if "width" in dim_id else "mdi:arrow-up-down",
            })

        # --- Switch: Vacuum motor direct control ---
        self._pub_discovery("switch", "vacuum_motor", {
            "name": "Vacuum Motor",
            "unique_id": "rosie_vacuum_motor",
            "command_topic": f"{pfx}/vacuum_motor/set",
            "state_topic": f"{pfx}/vacuum_state",
            "value_template": "{{ value_json.vacuum_on }}",
            "payload_on": "ON",
            "payload_off": "OFF",
            "state_on": "ON",
            "state_off": "OFF",
            "icon": "mdi:fan",
        })

        # --- Number: Vacuum speed ---
        self._pub_discovery("number", "vacuum_speed", {
            "name": "Vacuum Speed",
            "unique_id": "rosie_vacuum_speed",
            "command_topic": f"{pfx}/vacuum_speed/set",
            "state_topic": f"{pfx}/vacuum_state",
            "value_template": "{{ value_json.vacuum_speed }}",
            "min": 1,
            "max": 100,
            "step": 5,
            "unit_of_measurement": "%",
            "mode": "slider",
            "icon": "mdi:speedometer",
        })

        # --- Select: Navigation mode ---
        self._pub_discovery("select", "nav_mode", {
            "name": "Navigation Mode",
            "unique_id": "rosie_nav_mode",
            "command_topic": f"{pfx}/nav_mode/set",
            "state_topic": f"{pfx}/settings",
            "value_template": "{{ value_json.nav_mode }}",
            "options": ["Normal", "Gentle", "Deep", "Quick"],
            "icon": "mdi:robot-vacuum",
            "entity_category": "config",
        })

        # --- Sensors ---
        sensors = [
            ("fuel_percent", "Battery", "%", "battery", f"{pfx}/battery",
             "{{ value_json.fuel_percent | int }}", "measurement"),
            ("battery_voltage", "Battery Voltage", "V", None, f"{pfx}/battery",
             "{{ value_json.voltage }}", "measurement"),
            ("battery_temp", "Battery Temperature", "°C", "temperature", f"{pfx}/battery",
             "{{ value_json.temperature }}", "measurement"),
            ("ui_state", "UI State", None, None, f"{pfx}/state",
             "{{ value_json.ui_state }}", None),
            ("robot_state", "Robot State", None, None, f"{pfx}/state",
             "{{ value_json.robot_state }}", None),
            ("robot_error", "Robot Error", None, None, f"{pfx}/state",
             "{{ value_json.error }}", None),
            ("robot_alert", "Robot Alert", None, None, f"{pfx}/state",
             "{{ value_json.alert }}", None),
            ("nogo_line_count", "No-Go Line Count", None, None, f"{pfx}/nogo_lines",
             "{{ (value_json.lines | default([])) | count }}", "measurement"),
            ("nogo_status", "No-Go Status", None, None, f"{pfx}/nogo_status",
             "{{ value_json.status }}", None),
            ("nogo_message", "No-Go Message", None, None, f"{pfx}/nogo_status",
             "{{ value_json.message }}", None),
            ("vacuum_rpm", "Vacuum RPM", "RPM", None, f"{pfx}/vacuum_state",
             "{{ value_json.vacuum_rpm }}", "measurement"),
            ("vacuum_current", "Vacuum Current", "mA", None, f"{pfx}/vacuum_state",
             "{{ value_json.vacuum_ma }}", "measurement"),
        ]
        for s_id, label, unit, dev_class, topic, tmpl, state_class in sensors:
            cfg = {
                "name": label,
                "unique_id": f"rosie_{s_id}",
                "state_topic": topic,
                "value_template": tmpl,
            }
            if unit:
                cfg["unit_of_measurement"] = unit
            if dev_class:
                cfg["device_class"] = dev_class
            if state_class:
                cfg["state_class"] = state_class
            if s_id in ("battery_voltage", "battery_temp", "nogo_status", "nogo_message"):
                cfg["entity_category"] = "diagnostic"
            if s_id == "nogo_line_count":
                # Expose full lines array as entity attributes so the HA
                # no-go editor card can read/edit them after a page reload
                cfg["json_attributes_topic"] = topic
            self._pub_discovery("sensor", s_id, cfg)

        # --- Binary sensors ---
        bin_sensors = [
            ("charging", "Charging", "battery_charging", f"{pfx}/battery",
             "{{ 'ON' if value_json.charging else 'OFF' }}"),
            ("ext_power", "Docked", "plug", f"{pfx}/battery",
             "{{ 'ON' if value_json.ext_power else 'OFF' }}"),
        ]
        for bs_id, label, dev_class, topic, tmpl in bin_sensors:
            self._pub_discovery("binary_sensor", bs_id, {
                "name": label,
                "unique_id": f"rosie_{bs_id}",
                "state_topic": topic,
                "value_template": tmpl,
                "payload_on": "ON",
                "payload_off": "OFF",
                "device_class": dev_class,
            })

        # --- Binary sensors: physical bump switches ---
        bumper_sensors = [
            ("bumper_front_left",  "Bumper Front Left",  f"{pfx}/bumpers",
             "{{ 'ON' if value_json.left_front  else 'OFF' }}"),
            ("bumper_front_right", "Bumper Front Right", f"{pfx}/bumpers",
             "{{ 'ON' if value_json.right_front else 'OFF' }}"),
            ("bumper_side_left",   "Bumper Side Left",   f"{pfx}/bumpers",
             "{{ 'ON' if value_json.left_side   else 'OFF' }}"),
            ("bumper_side_right",  "Bumper Side Right",  f"{pfx}/bumpers",
             "{{ 'ON' if value_json.right_side  else 'OFF' }}"),
        ]
        for bs_id, label, topic, tmpl in bumper_sensors:
            self._pub_discovery("binary_sensor", bs_id, {
                "name": label,
                "unique_id": f"rosie_{bs_id}",
                "state_topic": topic,
                "value_template": tmpl,
                "payload_on": "ON",
                "payload_off": "OFF",
            })

        logger.info("Published HA MQTT Discovery config (%d entities)",
                     1 + len(buttons) + len(switches) + 2 + 1 +
                     len(sensors) + len(bin_sensors) + len(bumper_sensors))

    # ------------------------------------------------------------------
    # MQTT callbacks
    # ------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            logger.info("Connected to MQTT broker")
            self._publish_ha_discovery()
            self.publish("availability", "online", qos=1, retain=True)
            self.publish_spot_config()
            self._publish_vacuum_state()

            # Subscribe to all command topics
            pfx = self._prefix
            subs = [
                (f"{pfx}/command", 1),
                (f"{pfx}/cmd_vel", 0),
                (f"{pfx}/pose", 0),
                (f"{pfx}/settings/+/set", 1),
                (f"{pfx}/spot_width/set", 1),
                (f"{pfx}/spot_height/set", 1),
                (f"{pfx}/nav_mode/set", 1),
                (f"{pfx}/vacuum_motor/set", 1),
                (f"{pfx}/vacuum_speed/set", 1),
                (f"{pfx}/nogo_lines/set", 1),
                (f"{pfx}/nogo_tuning/set", 1),
            ]
            for topic, qos in subs:
                client.subscribe(topic, qos=qos)
            logger.info("Subscribed to %d topics", len(subs))
        else:
            logger.error("MQTT connect failed with rc=%d", rc)

    def _on_disconnect(self, client, userdata, rc) -> None:
        if rc != 0:
            logger.warning("Unexpected MQTT disconnect (rc=%d)", rc)

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage) -> None:
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace").strip()
        logger.debug("MQTT RX: %s → %s", topic, payload[:120])

        pfx = self._prefix

        # --- Main command topic ---
        if topic == f"{pfx}/command":
            if self._command_callback:
                self._command_callback(payload)

        # --- Velocity commands ---
        elif topic == f"{pfx}/cmd_vel":
            if self._cmd_vel_callback:
                try:
                    data = json.loads(payload)
                    self._cmd_vel_callback(
                        float(data.get("linear_x", 0.0)),
                        float(data.get("angular_z", 0.0)),
                    )
                except (json.JSONDecodeError, ValueError, TypeError) as exc:
                    logger.warning("Invalid cmd_vel payload: %s — %s", payload, exc)

        # --- SLAM-corrected pose from ROS 2 ---
        elif topic == f"{pfx}/pose":
            try:
                data = json.loads(payload)
                sx = float(data['x'])
                sy = float(data['y'])
                sth = float(data['theta'])
                meta = {
                    "mode": data.get("mode", "unknown"),
                    "map_id": data.get("map_id", ""),
                    "map_name": data.get("map_name", ""),
                    "src": data.get("src", ""),
                }
                with self._slam_lock:
                    self._slam_pose = (sx, sy, sth, time.monotonic(), meta)
                if self._slam_pose_callback:
                    self._slam_pose_callback(sx, sy, sth, meta)
            except (json.JSONDecodeError, ValueError, KeyError) as exc:
                logger.warning("Invalid pose payload: %s — %s", payload, exc)

        # --- Settings switch toggles ---
        elif topic.startswith(f"{pfx}/settings/") and topic.endswith("/set"):
            # e.g., rosie/settings/eco_mode/set → payload "ON" or "OFF"
            setting = topic.split("/")[-2]
            is_on = payload.upper() == "ON"
            # Record override so the next poll doesn't clobber
            self._settings_overrides[setting] = (is_on, time.monotonic())
            cmd = f"set_{setting}_on" if is_on else f"set_{setting}_off"
            if self._command_callback:
                self._command_callback(cmd)
            # Immediately republish settings with the override applied
            if self._last_settings:
                updated = dict(self._last_settings)
                updated[setting] = "ON" if is_on else "OFF"
                self._last_settings = updated
                self.publish("settings", updated, retain=True)

        # --- Spot dimension changes ---
        elif topic == f"{pfx}/spot_width/set":
            try:
                self._spot_width = max(100, min(400, int(float(payload))))
                self.publish_spot_config()
            except ValueError:
                pass

        elif topic == f"{pfx}/spot_height/set":
            try:
                self._spot_height = max(100, min(400, int(float(payload))))
                self.publish_spot_config()
            except ValueError:
                pass

        # --- Navigation mode select ---
        elif topic == f"{pfx}/nav_mode/set":
            if self._command_callback:
                self._nav_mode = payload.strip()
                self._command_callback(f"set_nav_mode:{payload}")

        # --- Vacuum motor direct control ---
        elif topic == f"{pfx}/vacuum_motor/set":
            if self._command_callback:
                if payload.upper() == "ON":
                    self._vacuum_on = True
                    self._command_callback(f"set_vacuum_speed:{self._vacuum_speed}")
                else:
                    self._vacuum_on = False
                    self._command_callback("vacuum_off")
                self._publish_vacuum_state()

        elif topic == f"{pfx}/vacuum_speed/set":
            try:
                speed = max(1, min(100, int(float(payload))))
                self._vacuum_speed = speed
                if self._vacuum_on and self._command_callback:
                    self._command_callback(f"set_vacuum_speed:{speed}")
                self._publish_vacuum_state()
            except ValueError:
                pass

        # --- No-go line updates ---
        elif topic == f"{pfx}/nogo_lines/set":
            if self._nogo_lines_callback is None:
                self.publish_nogo_status("error", "nogo callback not configured")
                return

            try:
                parsed: Any = json.loads(payload)
                lines_raw = parsed.get("lines") if isinstance(parsed, dict) else parsed
                if not isinstance(lines_raw, list):
                    raise ValueError("payload must be a JSON list or {'lines': [...]} object")

                ok, msg, active_lines = self._nogo_lines_callback(lines_raw)
                if ok:
                    self.publish_nogo_lines(active_lines)
                    self.publish_nogo_status("ok", msg)
                else:
                    self.publish_nogo_status("error", msg)
            except (json.JSONDecodeError, ValueError, TypeError) as exc:
                self.publish_nogo_status("error", f"invalid no-go payload: {exc}")

        # --- No-go tuning number updates ---
        elif topic == f"{pfx}/nogo_tuning/set":
            if self._nogo_tuning_callback is None:
                return
            try:
                data = json.loads(payload)
                for key, value in data.items():
                    self._nogo_tuning_callback(key, float(value))
            except (json.JSONDecodeError, ValueError, TypeError) as exc:
                logger.warning("Invalid nogo_tuning payload: %s — %s", payload, exc)
