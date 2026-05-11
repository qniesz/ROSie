"""
main.py ? ROSie Pi-side driver entry point.

Orchestrates:
  - Serial connection to Neato D6
  - SKey computation for SetEvent commands
  - Always-on state/battery/error polling
  - SetEvent-based cleaning & manual driving (no TestMode needed)
  - Optional LIDAR scanning at ~5 Hz (requires 'activate')
  - Optional odometry at ~20 Hz (requires 'activate')
  - MQTT publish / subscribe
  - Graceful shutdown
"""

import argparse
import json
import logging
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

from .serial_handler import NeatoSerial
from .lidar import get_lidar_scan
from .odometry import OdomState, get_motors, update_odometry
from .sensors import get_battery, get_bumpers, get_robot_state, get_user_settings, get_version, format_model, format_firmware
from .commands import handle_command, handle_cmd_vel
from .mqtt_bridge import MQTTBridge
from . import no_go_guard
from . import bumper_sensors

# MapPipeline is optional ? enabled by ROSIE_MAP_PIPELINE_ENABLED (default on).
# When disabled (Pi 4 / legacy deploy), behaviour is completely unchanged.
try:
    from .map_pipeline import MapPipeline as _MapPipeline
except ImportError:
    _MapPipeline = None

logger = logging.getLogger("rosie")

# Timing intervals (seconds)
SCAN_INTERVAL     = 0.22     # ~4.5 Hz (active mode only) ? widened from 0.20 to give GetMotors better interleave windows
STATE_INTERVAL    = 10.0    # ~0.1 Hz ? poll GetErr + GetState
CHARGER_INTERVAL  = 10.0  # ~0.1 Hz ? poll GetCharger
SETTINGS_INTERVAL = 60.0 # ~once per minute ? poll GetUserSettings
SERIAL_STATS_INTERVAL = float(os.environ.get("ROSIE_SERIAL_STATS_INTERVAL", "10.0"))
NOGO_LINES_FILE = os.environ.get("ROSIE_NOGO_LINES_FILE", "/home/rosie/nogo-lines.json")
NOGO_TUNING_FILE = os.environ.get("ROSIE_NOGO_TUNING_FILE", "/home/rosie/nogo-tuning.json")


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
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off", "disabled"}


PIPELINE_ENABLED = _env_flag("ROSIE_MAP_PIPELINE_ENABLED", default=False)
NOGO_PULSE_MS = _env_int("ROSIE_NOGO_PULSE_MS", 80, minimum=1)
NOGO_PULSE_INTERVAL_MS = _env_int("ROSIE_NOGO_PULSE_INTERVAL_MS", 160, minimum=1)
NOGO_SIDE_PULSES_BEFORE_FRONT = _env_int("ROSIE_NOGO_SIDE_PULSES_BEFORE_FRONT", 3, minimum=1)
NOGO_VIRTUAL_STOP_ENABLED = _env_flag("ROSIE_NOGO_VIRTUAL_STOP_ENABLED", default=False)

NOGO_DEBUG_PUBLISH_HZ = 2.0  # rate limit for rosie/nogo_debug MQTT publish
BUMPER_EVENT_LOG_PATH = os.environ.get(
    "ROSIE_BUMPER_EVENT_LOG", "/home/rosie/logs/bumper_events.jsonl"
)
BUMPER_EVENT_WINDOW_SECS = 3.0      # how long to record after the first bump
BUMPER_EVENT_SAMPLE_SECS = 0.2      # sample period during the window

_shutdown = False


class BumpEventRecorder:
    """Capture robot pose for ~3 s after each bump (physical or virtual).

    On every bump, schedules a recorder thread (one at a time). Subsequent
    bumps within the active window are appended to ``additional_triggers``.
    On window expiry the record is published to MQTT and appended to a JSONL
    log file on disk for later analysis.
    """

    def __init__(self, mqtt_bridge, pose_provider, ui_state_provider,
                 motors_provider=None,
                 log_path: str = BUMPER_EVENT_LOG_PATH,
                 window_secs: float = BUMPER_EVENT_WINDOW_SECS,
                 sample_secs: float = BUMPER_EVENT_SAMPLE_SECS):
        self._mqtt = mqtt_bridge
        self._pose_provider = pose_provider
        self._ui_state_provider = ui_state_provider
        self._motors_provider = motors_provider
        self._log_path = log_path
        self._window_secs = window_secs
        self._sample_secs = sample_secs
        self._lock = threading.Lock()
        self._active_record: dict | None = None
        self._thread: threading.Thread | None = None

    # Called from bumper_sensors._emit_bump_event (any thread).
    def on_bump(self, side: str, is_front: bool, virtual: bool,
                pose: tuple | None) -> None:
        ts = time.time()
        with self._lock:
            if self._active_record is not None:
                # Append to in-flight record.
                self._active_record["additional_triggers"].append({
                    "ts_offset": round(ts - self._active_record["ts"], 3),
                    "side": side,
                    "is_front": is_front,
                    "virtual": virtual,
                })
                return

            # Start a new record.
            pre_pose = self._pose_to_dict(pose)
            pre_motors = None
            if self._motors_provider is not None:
                try:
                    pre_motors = self._wheels_to_dict(self._motors_provider())
                except Exception:
                    pass
            self._active_record = {
                "ts": ts,
                "ts_iso": time.strftime("%Y-%m-%dT%H:%M:%S",
                                        time.localtime(ts)),
                "trigger": {
                    "side": side,
                    "is_front": is_front,
                    "virtual": virtual,
                },
                "pre_pose": pre_pose,
                "pre_wheels": pre_motors,
                "pre_ui_state": self._ui_state_provider(),
                "samples": [],
                "additional_triggers": [],
            }
            t = threading.Thread(target=self._run, daemon=True,
                                 name="bump-recorder")
            self._thread = t
            t.start()

    def _pose_to_dict(self, pose) -> dict | None:
        if pose is None or len(pose) < 5:
            return None
        d = {
            "x": round(pose[0], 4),
            "y": round(pose[1], 4),
            "theta": round(pose[2], 4),
            "linear_vel": round(pose[3], 4),
            "angular_vel": round(pose[4], 4),
        }
        if len(pose) > 5:
            d["pose_source"] = pose[5]
        return d

    def _wheels_to_dict(self, motors: dict | None) -> dict | None:
        """Extract left/right wheel RPM, speed, and load from a GetMotors dict."""
        if not motors:
            return None
        keys = (
            "LeftWheel_RPM", "LeftWheel_Speed", "LeftWheel_Load",
            "RightWheel_RPM", "RightWheel_Speed", "RightWheel_Load",
        )
        out = {k: round(motors[k], 2) for k in keys if k in motors}
        return out if out else None

    def _run(self) -> None:
        try:
            start = time.monotonic()
            last_ui = None
            ui_changes = []
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= self._window_secs:
                    break
                pose = None
                try:
                    pose = self._pose_provider()
                except Exception:
                    pass
                ui = None
                try:
                    ui = self._ui_state_provider()
                except Exception:
                    pass

                sample = {"dt": round(elapsed, 3), "ui_state": ui}
                if pose is not None and len(pose) >= 5:
                    sample.update({
                        "x": round(pose[0], 4),
                        "y": round(pose[1], 4),
                        "theta": round(pose[2], 4),
                        "lin": round(pose[3], 4),
                        "ang": round(pose[4], 4),
                    })
                    if len(pose) > 5:
                        sample["pose_source"] = pose[5]
                if self._motors_provider is not None:
                    try:
                        w = self._wheels_to_dict(self._motors_provider())
                        if w:
                            sample["wheels"] = w
                    except Exception:
                        pass

                with self._lock:
                    if self._active_record is not None:
                        self._active_record["samples"].append(sample)

                if ui != last_ui:
                    ui_changes.append({"dt": round(elapsed, 3), "ui_state": ui})
                    last_ui = ui

                time.sleep(self._sample_secs)

            with self._lock:
                rec = self._active_record
                self._active_record = None
            if rec is None:
                return

            # Compute net pose delta if we have pre_pose + a final sample.
            pre = rec.get("pre_pose")
            samples = rec.get("samples", [])
            last = samples[-1] if samples else None
            if pre is not None and last is not None and "x" in last:
                rec["net_dx"] = round(last["x"] - pre["x"], 4)
                rec["net_dy"] = round(last["y"] - pre["y"], 4)
                rec["net_dtheta"] = round(last["theta"] - pre["theta"], 4)
            rec["ui_state_changes"] = ui_changes

            # Append to JSONL on disk.
            try:
                os.makedirs(os.path.dirname(self._log_path) or ".",
                            exist_ok=True)
                with open(self._log_path, "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec) + "\n")
            except Exception as exc:
                logger.warning("[bump_event] JSONL append failed: %s", exc)

            # Publish over MQTT for HA.
            try:
                self._mqtt.publish_bumper_event(rec)
            except Exception as exc:
                logger.warning("[bump_event] MQTT publish failed: %s", exc)

            logger.info(
                "[bump_event] DONE side=%s virtual=%s extras=%d net_dx=%.3f net_dy=%.3f net_dtheta=%.3f ui_changes=%d",
                rec["trigger"]["side"], rec["trigger"]["virtual"],
                len(rec["additional_triggers"]),
                rec.get("net_dx", 0.0), rec.get("net_dy", 0.0),
                rec.get("net_dtheta", 0.0),
                len(ui_changes),
            )
        except Exception:
            logger.exception("[bump_event] recorder thread crashed")


def _signal_handler(sig, frame):
    global _shutdown
    logger.info("Shutdown signal received (%s)", signal.Signals(sig).name)
    _shutdown = True


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ROSie Neato D6 Pi Driver")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--mqtt-host", default=os.environ.get("MQTT_HOST", "localhost"))
    p.add_argument("--mqtt-port", type=int,
                   default=int(os.environ.get("MQTT_PORT", "1883")))
    p.add_argument("--mqtt-user", default=os.environ.get("MQTT_USER"))
    p.add_argument("--mqtt-pass", default=os.environ.get("MQTT_PASS"))
    p.add_argument("--robot-name", default=os.environ.get("ROSIE_NAME", "ROSie"),
                   help="Human-readable robot name shown in HA (default: ROSie)")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p.parse_args()


def main() -> None:
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # ------------------------------------------------------------------
    # Initialize serial
    # ------------------------------------------------------------------
    # Initialize GPIO no-go guard early so pins settle before cleaning starts
    no_go_guard.init()
    bumper_sensors.start(mqtt_bridge=None)  # mqtt not connected yet; updated below
    loaded_lines, load_msg = no_go_guard.load_lines_file(NOGO_LINES_FILE)
    if loaded_lines:
        logger.info("No-go lines loaded: %s", load_msg)
    else:
        logger.info("No-go lines: %s", load_msg)

    serial = NeatoSerial(port=args.port, baudrate=args.baudrate)
    skey = ""

    # ------------------------------------------------------------------
    # Initialize MQTT early ? before serial ? so GPIO/bumper commands
    # work even when the Neato is not yet connected
    # ------------------------------------------------------------------
    # Derive MQTT topic prefix from the robot name: lowercase, collapse
    # non-alphanumeric runs to underscores ("ROSie" ? "rosie",
    # "Kitchen Bot" ? "kitchen_bot", "ROSie 2" ? "rosie_2").
    mqtt_prefix = re.sub(r'[^a-z0-9]+', '_',
                         args.robot_name.lower()).strip('_') or 'rosie'
    mqtt = MQTTBridge(
        host=args.mqtt_host,
        port=args.mqtt_port,
        username=args.mqtt_user,
        password=args.mqtt_pass,
        prefix=mqtt_prefix,
        name=args.robot_name,
    )

    # Track whether LDS/TestMode is active (for LIDAR + direct motor control)
    lds_active = False
    force_poll = False  # set True by "update_status" command

    # pipeline is set after MQTT connects; pre-declare so closures can reference it
    pipeline = None

    def activate_lds():
        """Enter TestMode, spin up LDS for LIDAR scans + motor control."""
        nonlocal lds_active
        if lds_active:
            logger.info("LDS already active")
            return
        logger.info("Activating LDS?")
        serial.set_test_mode(True)
        serial.set_lds_rotation(True)
        time.sleep(3.0)
        serial.flush()
        get_lidar_scan(serial)  # discard first scan
        lds_active = True
        logger.info("LDS active")

    def deactivate_lds():
        """Stop motors, LDS, exit TestMode."""
        nonlocal lds_active
        if not lds_active:
            logger.info("LDS already inactive")
            return
        logger.info("Deactivating LDS?")
        serial.set_motors(0, 0, 0)
        time.sleep(0.1)
        serial.set_lds_rotation(False)
        time.sleep(0.1)
        serial.set_test_mode(False)
        lds_active = False
        logger.info("LDS deactivated")

    def on_command(cmd: str):
        """Handle all commands."""
        nonlocal force_poll, skey
        cmd_lower = cmd.strip().lower()

        # Lazily retry SKey if it wasn't available at startup
        if not skey:
            try:
                version = get_version(serial)
                if version.serial_number:
                    skey = NeatoSerial.compute_skey(version.serial_number)
                    logger.info("SKey computed (lazy) from S/N: %s", version.serial_number.split(",")[0])
            except Exception as exc:
                logger.debug("Lazy SKey retry failed: %s", exc)

        if cmd_lower == "activate":
            activate_lds()
        elif cmd_lower == "deactivate":
            deactivate_lds()
        elif cmd_lower == "update_status":
            force_poll = True
        elif cmd_lower == "create_map":
            # Clear no-go lines so they don't interfere with SLAM
            ok, msg, exported = no_go_guard.set_lines([])
            logger.info("New map requested ? no-go lines cleared: %s", msg)
            no_go_guard.save_lines_file(NOGO_LINES_FILE)
            mqtt.publish_nogo_lines(exported)
            mqtt.publish_nogo_status("ok", "cleared for new map")
            # Delegate to map pipeline if enabled
            if pipeline is not None:
                pipeline.on_command("create_map")
        elif cmd_lower == "update":
            update_script = os.path.join(
                os.path.expanduser("~"), "rosie", "pi", "update.sh"
            )
            dev_marker = os.path.join(os.path.expanduser("~"), "rosie", ".rosie-dev-mode")
            if os.path.exists(dev_marker):
                msg = f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} BLOCKED: dev mode active ({dev_marker})"
                try:
                    with open(os.path.join(os.path.expanduser("~"), "last-update.txt"), "w", encoding="utf-8") as fh:
                        fh.write(msg + "\n")
                except Exception:
                    logger.debug("Could not write dev-mode update block status", exc_info=True)
                logger.warning("Software update blocked: dev mode active at %s", dev_marker)
                return
            try:
                subprocess.Popen(
                    ["bash", update_script, "--force"],
                    start_new_session=True,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    close_fds=True,
                )
                logger.info("Software update triggered via MQTT command")
            except Exception as exc:
                logger.warning("Failed to spawn software update: %s", exc)
        elif cmd_lower == "reboot":
            if pipeline is not None:
                pipeline.on_command("reboot")
            else:
                logger.warning("Reboot requested but no pipeline ? ignoring")
        elif cmd_lower == "test_bumper_fl":
            bumper_sensors.test_pin("front_left")
        elif cmd_lower == "test_bumper_fr":
            bumper_sensors.test_pin("front_right")
        elif cmd_lower == "test_bumper_sl":
            bumper_sensors.test_pin("side_left")
        elif cmd_lower == "test_bumper_sr":
            bumper_sensors.test_pin("side_right")
        elif cmd_lower in ("start", "house_clean", "spot_clean"):
            # Ensure slam_toolbox container is running before the robot undocks.
            # No-op if already up; starts/restarts it if it exited after the
            # previous cycle.  The container's 30 s warm-up window aligns with
            # the robot's undocking sequence so slam is ACTIVE by cleaning time.
            if pipeline is not None:
                try:
                    pipeline.ensure_online_slam()
                except Exception:
                    logger.debug("ensure_online_slam failed", exc_info=True)
            handle_command(serial, cmd, skey)
        else:
            handle_command(serial, cmd, skey)

    def on_cmd_vel(lx: float, az: float):
        if not lds_active:
            logger.debug("cmd_vel ignored ? LDS not active")
            return
        handle_cmd_vel(serial, lx, az)

    def on_nogo_lines(lines: list) -> tuple[bool, str, list[dict[str, list[float]]]]:
        ok, msg, exported = no_go_guard.set_lines(lines)
        if not ok:
            return ok, msg, exported

        saved, save_msg = no_go_guard.save_lines_file(NOGO_LINES_FILE)
        result_msg = f"{msg}; persisted" if saved else f"{msg}; persistence warning: {save_msg}"

        # Notify map pipeline so it redraws the no-go overlay on the map
        if pipeline is not None:
            pipeline.on_nogo_lines(exported)

        return True, result_msg, exported

    mqtt.set_command_callback(on_command)
    mqtt.set_cmd_vel_callback(on_cmd_vel)
    mqtt.set_nogo_lines_callback(on_nogo_lines)

    try:
        mqtt.connect()
    except Exception as exc:
        logger.critical("Cannot connect to MQTT broker: %s", exc)
        sys.exit(1)

    # Wire mqtt into bumper_sensors now that it is connected
    bumper_sensors._mqtt = mqtt

    # Start map pipeline (Pi Zero 2 W mode)
    pipeline = None
    if PIPELINE_ENABLED and _MapPipeline is not None:
        try:
            pipeline = _MapPipeline(mqtt)
            pipeline.start()
            # Seed pipeline with no-go lines already loaded from disk
            pipeline.on_nogo_lines(no_go_guard.export_lines())
        except Exception as exc:
            logger.warning("MapPipeline init failed (%s) ? running without it", exc)
            pipeline = None

    nogo_pulse_hold_secs = NOGO_PULSE_MS / 1000.0
    nogo_pulse_interval_secs = NOGO_PULSE_INTERVAL_MS / 1000.0

    # ?? Reaction-aware cooldown (suppresses repeat pulses on the same side
    #    while the firmware's first reaction is still in progress) ????????
    NOGO_COOLDOWN_SECS = float(os.environ.get("ROSIE_NOGO_COOLDOWN_SECS", "2.5"))
    NOGO_COOLDOWN_DTHETA = math.radians(
        float(os.environ.get("ROSIE_NOGO_COOLDOWN_DTHETA_DEG", "30"))
    )
    NOGO_COOLDOWN_DISP = float(os.environ.get("ROSIE_NOGO_COOLDOWN_DISP_M", "0.10"))

    # Mutable runtime tuning (overridable from HA via MQTT). Keys must match
    # the HA Number unique_ids minus the 'rosie_nogo_' prefix.
    half_l_env = float(os.environ.get("ROSIE_NOGO_HALF_LENGTH", "0.20"))
    half_w_env = float(os.environ.get("ROSIE_NOGO_HALF_WIDTH",  "0.20"))
    nogo_tuning = {
        "half_length":              half_l_env,
        "half_width":               half_w_env,
        # Front bumps: firmware reaction is strong (~50? turn). Generous
        # cooldown lets it complete instead of being interrupted.
        "front_cooldown_secs":      NOGO_COOLDOWN_SECS,
        "front_cooldown_dtheta_deg": math.degrees(NOGO_COOLDOWN_DTHETA),
        "front_cooldown_disp_m":    NOGO_COOLDOWN_DISP,
        # Side bumps: firmware reaction is small (~20? turn) and the robot
        # tends to track over the line. Aggressive cooldown re-pulses faster
        # for tighter line-following.
        "side_cooldown_secs":       1.0,
        "side_cooldown_dtheta_deg": 15.0,
        "side_cooldown_disp_m":     0.05,
    }

    # Overlay any persisted tuning from disk (HA changes survive restart).
    try:
        if os.path.exists(NOGO_TUNING_FILE):
            with open(NOGO_TUNING_FILE, "r", encoding="utf-8") as f:
                disk_tuning = json.load(f)
            # Migration: old single 'cooldown_*' keys map to BOTH front and side.
            for old, new_pair in (
                ("cooldown_secs",        ("front_cooldown_secs",        "side_cooldown_secs")),
                ("cooldown_dtheta_deg",  ("front_cooldown_dtheta_deg",  "side_cooldown_dtheta_deg")),
                ("cooldown_disp_m",      ("front_cooldown_disp_m",      "side_cooldown_disp_m")),
            ):
                if old in disk_tuning:
                    for nk in new_pair:
                        disk_tuning.setdefault(nk, disk_tuning[old])
            for k in nogo_tuning:
                if k in disk_tuning:
                    nogo_tuning[k] = float(disk_tuning[k])
            logger.info("[main] no-go tuning loaded from %s", NOGO_TUNING_FILE)
    except Exception as exc:  # noqa: BLE001
        logger.warning("[main] failed to load tuning file: %s", exc)

    # Apply loaded footprint to the guard immediately.
    no_go_guard.set_footprint(
        half_length=nogo_tuning["half_length"],
        half_width=nogo_tuning["half_width"],
    )

    def _save_nogo_tuning() -> None:
        try:
            os.makedirs(os.path.dirname(NOGO_TUNING_FILE) or ".", exist_ok=True)
            tmp = f"{NOGO_TUNING_FILE}.tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(nogo_tuning, f, indent=2)
            os.replace(tmp, NOGO_TUNING_FILE)
        except Exception as exc:  # noqa: BLE001
            logger.warning("[main] failed to save tuning file: %s", exc)

    def _on_nogo_tuning_set(key: str, value: float) -> None:
        if key not in nogo_tuning:
            logger.warning("[main] unknown tuning key from HA: %s", key)
            return
        # Clamp same as HA Number ranges.
        if key in ("half_length", "half_width"):
            value = max(0.10, min(0.30, value))
        elif key.endswith("_secs"):
            value = max(0.0, min(10.0, value))
        elif key.endswith("_dtheta_deg"):
            value = max(0.0, min(180.0, value))
        elif key.endswith("_disp_m"):
            value = max(0.0, min(0.50, value))
        nogo_tuning[key] = value
        if key == "half_length":
            no_go_guard.set_footprint(half_length=value)
        elif key == "half_width":
            no_go_guard.set_footprint(half_width=value)
        logger.info("[main] no-go tuning updated: %s=%.3f", key, value)
        _save_nogo_tuning()
        try:
            mqtt.publish_nogo_tuning(nogo_tuning)
        except Exception:
            logger.debug("publish_nogo_tuning failed", exc_info=True)

    mqtt.set_nogo_tuning_callback(_on_nogo_tuning_set)
    # Publish current tuning as retained so HA Number entities reflect actual values.
    try:
        mqtt.publish_nogo_tuning(nogo_tuning)
    except Exception:
        logger.debug("initial publish_nogo_tuning failed", exc_info=True)

    logger.info(
        "[main] no-go cooldown front: secs=%.2f dtheta=%.0f? disp=%.2fm | "
        "side: secs=%.2f dtheta=%.0f? disp=%.2fm",
        nogo_tuning["front_cooldown_secs"],
        nogo_tuning["front_cooldown_dtheta_deg"],
        nogo_tuning["front_cooldown_disp_m"],
        nogo_tuning["side_cooldown_secs"],
        nogo_tuning["side_cooldown_dtheta_deg"],
        nogo_tuning["side_cooldown_disp_m"],
    )

    nogo_seq = {
        "left": {
            "touching": False,
            "is_front": False,
            "side_pulses": 0,
            "next_pulse_at": 0.0,
            "phase": "",
            "last_pulse_ts": 0.0,
            "last_pulse_pose": None,   # (x, y, theta) at last fired pulse
        },
        "right": {
            "touching": False,
            "is_front": False,
            "side_pulses": 0,
            "next_pulse_at": 0.0,
            "phase": "",
            "last_pulse_ts": 0.0,
            "last_pulse_pose": None,
        },
    }

    def _reset_nogo_side(side: str) -> None:
        st = nogo_seq[side]
        st["touching"] = False
        st["is_front"] = False
        st["side_pulses"] = 0
        st["next_pulse_at"] = 0.0
        st["phase"] = ""
        st["last_pulse_ts"] = 0.0
        st["last_pulse_pose"] = None

    def _nogo_cooldown_active(side: str, now_ts: float,
                              is_front: bool) -> tuple[bool, str]:
        """Return (suppress?, reason) for the cooldown on this side.

        Front and side bumps use independent thresholds (front cooldown is
        generous so the firmware's strong reaction completes; side cooldown
        is aggressive so the robot re-pulses to track tightly along a line).
        """
        st = nogo_seq[side]
        last_ts = st["last_pulse_ts"]
        last_pose = st["last_pulse_pose"]
        if last_ts <= 0.0 or last_pose is None:
            return False, ""
        prefix = "front" if is_front else "side"
        cd_secs   = nogo_tuning[f"{prefix}_cooldown_secs"]
        cd_dtheta = math.radians(nogo_tuning[f"{prefix}_cooldown_dtheta_deg"])
        cd_disp   = nogo_tuning[f"{prefix}_cooldown_disp_m"]
        elapsed = now_ts - last_ts
        if elapsed >= cd_secs:
            return False, f"elapsed={elapsed:.2f}s"
        odom_now = _latest_odom["odom"]
        if odom_now is None:
            return False, ""
        lx, ly, ltheta = last_pose
        disp = math.hypot(odom_now.x - lx, odom_now.y - ly)
        dtheta = abs(math.atan2(
            math.sin(odom_now.theta - ltheta),
            math.cos(odom_now.theta - ltheta),
        ))
        if dtheta >= cd_dtheta:
            return False, f"d\u03b8={math.degrees(dtheta):.0f}\u00b0"
        if disp >= cd_disp:
            return False, f"disp={disp:.2f}m"
        return True, (
            f"{prefix} elapsed={elapsed:.2f}s "
            f"d\u03b8={math.degrees(dtheta):.0f}\u00b0 disp={disp:.2f}m"
        )

    def _fire_nogo_pulse(side: str, now_ts: float) -> None:
        st = nogo_seq[side]
        if not st["touching"]:
            return

        use_front = bool(st["is_front"]) or st["side_pulses"] >= NOGO_SIDE_PULSES_BEFORE_FRONT
        # Cooldown uses the GEOMETRY approach direction (is_front), not the bump
        # type.  Side-approach contacts (robot drifting parallel to a line) get
        # the tighter side cooldown so the guard re-fires sooner and keeps the
        # robot from slowly creeping over the boundary.  Front-approach contacts
        # (robot heading straight at the line) get the generous front cooldown
        # so the firmware's strong 50? reaction can complete undisturbed.
        suppress, reason = _nogo_cooldown_active(side, now_ts, st["is_front"])
        if suppress:
            logger.info("[main] no-go pulse SUPPRESSED side=%s %s", side, reason)
            # Re-check soon so we re-evaluate cooldown promptly.
            st["next_pulse_at"] = now_ts + nogo_pulse_interval_secs
            return

        phase = "front" if use_front else "side"
        if phase != st["phase"]:
            logger.info("[main] no-go pulse phase=%s side=%s", phase, side)
            st["phase"] = phase

        # trigger_virtual() drives the bumper GPIO pin LOW, faking a real
        # bumper switch closure to the Neato firmware which then performs
        # its built-in back-up-and-turn response.
        if use_front:
            bumper_sensors.trigger_virtual(
                front_left=(side == "left"),
                front_right=(side == "right"),
                hold_secs=nogo_pulse_hold_secs,
                stop_on_trigger=False,
            )
        else:
            bumper_sensors.trigger_virtual(
                side_left=(side == "left"),
                side_right=(side == "right"),
                hold_secs=nogo_pulse_hold_secs,
                stop_on_trigger=False,
            )
            st["side_pulses"] += 1

        st["next_pulse_at"] = now_ts + nogo_pulse_interval_secs
        st["last_pulse_ts"] = now_ts
        odom_now = _latest_odom["odom"]
        if odom_now is not None:
            st["last_pulse_pose"] = (odom_now.x, odom_now.y, odom_now.theta)

    def _tick_nogo_pulses(now_ts: float) -> None:
        for side, st in nogo_seq.items():
            if st["touching"] and now_ts >= st["next_pulse_at"]:
                _fire_nogo_pulse(side, now_ts)

    # Set up no-go guard touch callback ? fires the relevant virtual bumper
    def _on_nogo_touch(side: str, is_front: bool, touching: bool) -> None:
        if side not in nogo_seq:
            return

        st = nogo_seq[side]
        if touching:
            st["touching"] = True
            st["is_front"] = is_front
            st["side_pulses"] = 0
            st["next_pulse_at"] = 0.0
            st["phase"] = ""
            st["last_pulse_ts"] = 0.0
            st["last_pulse_pose"] = None
            logger.info("[main] no-go touch start side=%s front=%s", side, is_front)
            _fire_nogo_pulse(side, time.monotonic())
        else:
            if st["touching"]:
                logger.info("[main] no-go touch release side=%s", side)
            _reset_nogo_side(side)
    no_go_guard.set_touch_callback(_on_nogo_touch)

    # Stop callback for bumper_sensors ? sends stop-cleaning to Neato
    def _do_stop() -> None:
        handle_command(serial, "stop", skey)
    bumper_sensors.set_stop_callback(_do_stop)

    # ------------------------------------------------------------------
    # Bump-event recorder + pose/UI providers (data collection)
    # ------------------------------------------------------------------
    # Shared mutable box so recorder threads can read the latest odom and motor state.
    _latest_odom   = {"odom": None}
    _latest_motors = {"motors": None}
    _latest_ui     = {"state": ""}

    # ------------------------------------------------------------------
    # SLAM + odom fusion for the no-go guard
    # ------------------------------------------------------------------
    # slam_toolbox publishes drift-corrected pose at ~2.5 Hz with up to
    # ~400 ms of latency.  At 0.8 m/s that means the raw slam pose lags the
    # robot by ~320 mm, which is unsafe for no-go-line geometry checks.
    #
    # Fix: when a slam pose arrives, snapshot the wheel-odom pose at the
    # same instant.  Whenever the guard asks for a pose, we add the odom
    # delta (now - anchor) to the slam anchor, rotated into the map frame.
    # This gives the guard a low-latency, drift-corrected pose every tick.
    _fusion_lock = threading.Lock()
    _fusion = {
        "anchor_slam": None,   # (x, y, theta) at slam-pose arrival
        "anchor_odom": None,   # (x, y, theta) wheel odom at same instant
        "anchor_stamp": 0.0,   # time.monotonic() at arrival
    }

    def _on_slam_pose_arrived(sx: float, sy: float, sth: float,
                              meta: dict | None = None) -> None:
        if meta and meta.get("mode") == "mapping":
            return
        o = _latest_odom["odom"]
        if o is None:
            return
        with _fusion_lock:
            _fusion["anchor_slam"] = (sx, sy, sth)
            _fusion["anchor_odom"] = (o.x, o.y, o.theta)
            _fusion["anchor_stamp"] = time.monotonic()

    mqtt.set_slam_pose_callback(_on_slam_pose_arrived)

    def _fused_pose(odom_state) -> tuple | None:
        """Return (x, y, theta, anchor_age_s) by extrapolating slam anchor
        with wheel-odom delta.  None if no slam anchor yet.
        """
        with _fusion_lock:
            anchor_slam = _fusion["anchor_slam"]
            anchor_odom = _fusion["anchor_odom"]
            anchor_stamp = _fusion["anchor_stamp"]
        if anchor_slam is None or anchor_odom is None:
            return None
        sx, sy, sth = anchor_slam
        ox0, oy0, oth0 = anchor_odom
        # Odom-frame delta from anchor to now
        dx_o = odom_state.x - ox0
        dy_o = odom_state.y - oy0
        dtheta = odom_state.theta - oth0
        # Rotate delta from odom frame into map frame
        # (rotation between frames at the anchor instant: sth - oth0)
        rot = sth - oth0
        c = math.cos(rot)
        s = math.sin(rot)
        dx_m = c * dx_o - s * dy_o
        dy_m = s * dx_o + c * dy_o
        return (sx + dx_m, sy + dy_m, sth + dtheta,
                time.monotonic() - anchor_stamp)

    def _pose_provider() -> tuple | None:
        o = _latest_odom["odom"]
        if o is None:
            return None
        # Include fused/slam source so bump events capture localization quality.
        _, _, _, gsource = _get_guard_pose(o)
        return (o.x, o.y, o.theta, o.linear_vel, o.angular_vel, gsource)

    def _ui_state_provider() -> str:
        return _latest_ui["state"]

    def _get_guard_pose(odom_state) -> tuple:
        """Return (x, y, theta, source) preferring SLAM-corrected pose.

        Priority order:
          1. Fused pose: slam anchor + live odom delta (low-latency, drift-corrected)
             2. Raw external slam_toolbox pose via mqtt.get_slam_pose() if fresh
                 and not explicitly from mapping mode
          3. Raw odometry fallback
        """
        # 1. Fused slam+odom ? use whenever a slam anchor exists and is reasonably
        # fresh.  Stale anchors (>2 s) are dropped because odom drift could be large.
        fp = _fused_pose(odom_state)
        if fp is not None:
            fx, fy, fth, anchor_age = fp
            if anchor_age < 2.0:
                return (fx, fy, fth, "slam_fused")

        # 2. External slam_toolbox (running in container) ? anchor missing/stale
        try:
            sp = mqtt.get_slam_pose()
        except Exception:
            sp = None
            logger.debug("mqtt.get_slam_pose failed", exc_info=True)
        if sp is not None:
            sx, sy, sth, age, meta = sp
            if age < 2.0 and meta.get("mode") != "mapping":
                return (sx, sy, sth, "slam_toolbox")

        # 3. Raw odometry fallback
        return (odom_state.x, odom_state.y, odom_state.theta, "odom")

    bumper_sensors.set_pose_provider(_pose_provider)

    def _motors_provider() -> dict | None:
        return _latest_motors["motors"]

    bump_recorder = BumpEventRecorder(
        mqtt_bridge=mqtt,
        pose_provider=_pose_provider,
        ui_state_provider=_ui_state_provider,
        motors_provider=_motors_provider,
    )
    bumper_sensors.set_bump_event_callback(bump_recorder.on_bump)
    logger.info("[bump_event] recorder ready (log=%s window=%.1fs sample=%.2fs)",
                BUMPER_EVENT_LOG_PATH, BUMPER_EVENT_WINDOW_SECS,
                BUMPER_EVENT_SAMPLE_SECS)

    # Publish initial no-go and bumper state (no serial needed)
    mqtt.publish_nogo_lines(no_go_guard.export_lines())
    if no_go_guard.is_enabled():
        mqtt.publish_nogo_status("ok", f"{no_go_guard.line_count()} no-go line(s) active")
    else:
        mqtt.publish_nogo_status("disabled", "disabled by ROSIE_NOGO_ENABLED")
    mqtt.publish_bumpers(False, False, False, False)  # initial state: all clear

    # ------------------------------------------------------------------
    # Connect serial (retry loop ? MQTT is already up)
    # ------------------------------------------------------------------
    retry_delay = 5
    while not _shutdown:
        try:
            serial.connect()
            break
        except Exception as exc:
            logger.warning("Cannot open serial port: %s ? retrying in %ds", exc, retry_delay)
            time.sleep(retry_delay)
    if _shutdown:
        mqtt.disconnect()
        sys.exit(0)

    # ------------------------------------------------------------------
    # Compute SKey from robot serial number
    # ------------------------------------------------------------------
    try:
        time.sleep(1.0)  # give robot time to be ready
        version = get_version(serial)
        if version.serial_number:
            skey = NeatoSerial.compute_skey(version.serial_number)
            logger.info("SKey computed from S/N: %s (model: %s, fw: %s)",
                        version.serial_number.split(",")[0],
                        version.model, version.software)
        else:
            logger.warning("Could not read serial number ? SetEvent commands disabled")

        # Update HA device info with the actual model + firmware from the robot
        if version.model or version.software:
            display_model = format_model(version.model) or "BotVac"
            firmware     = format_firmware(version.software)
            try:
                mqtt.update_device_info(model=display_model, hw_version=firmware)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Could not update MQTT device info: %s", exc)
            if pipeline is not None:
                try:
                    pipeline.update_device_info(model=display_model, hw_version=firmware)
                except Exception as exc:  # noqa: BLE001
                    logger.warning("Could not update pipeline device info: %s", exc)
    except Exception as exc:
        logger.warning("SKey computation failed: %s ? SetEvent commands disabled", exc)

    # Publish initial robot state now that serial is up
    mqtt.publish_state(
        ui_state="UIMGR_STATE_IDLE",
        robot_state="ST_C_Off",
        error="none", alert="none",
    )

    logger.info("ROSie driver running ? ready for commands (SKey %s)",
                "available" if skey else "UNAVAILABLE")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    odom = OdomState()
    last_scan     = 0.0
    last_state    = 0.0
    last_charger  = 0.0
    last_settings = 0.0
    last_bumpers  = 0.0
    last_nogo_pub = 0.0
    last_serial_stats = 0.0
    BUMPER_INTERVAL = 1.0   # poll bumpers via serial every 1s
    NOGO_PUB_INTERVAL = 1.0 / NOGO_DEBUG_PUBLISH_HZ
    was_scan_active = False  # track transitions to reset odom
    serial_fail_count = 0
    SERIAL_FAIL_LIMIT = 20   # consecutive failures before reconnect attempt


    try:
        while not _shutdown:
            now = time.monotonic()

            # === Always-on polling (works without TestMode) ===

            # --- State + errors ---
            if force_poll or now - last_state >= STATE_INTERVAL:
                last_state = now
                force_poll = False
                try:
                    state = get_robot_state(serial)
                    mqtt.publish_state(
                        ui_state=state.ui_state,
                        robot_state=state.robot_state,
                        error=state.error,
                        alert=state.alert,
                    )
                    _latest_ui["state"] = state.ui_state
                    if pipeline is not None:
                        pipeline.on_state(ui_state=state.ui_state)
                except Exception:
                    logger.debug("State poll failed", exc_info=True)

            # --- Battery / charger ---
            if now - last_charger >= CHARGER_INTERVAL:
                last_charger = now
                try:
                    batt = get_battery(serial)
                    mqtt.publish_battery(
                        fuel_percent=batt.fuel_percent,
                        voltage=batt.voltage,
                        charging=batt.charging_active,
                        ext_power=batt.ext_power_present,
                        temperature=batt.battery_temp_c,
                    )
                    # Update pipeline with latest ext_power for dock detection
                    if pipeline is not None:
                        pipeline.on_state(ext_power=batt.ext_power_present)
                except Exception:
                    logger.debug("Charger poll failed", exc_info=True)

            # --- User settings ---
            if now - last_settings >= SETTINGS_INTERVAL:
                last_settings = now
                try:
                    settings = get_user_settings(serial)
                    mqtt.publish_settings(
                        eco_mode=settings.eco_mode,
                        wall_enable=settings.wall_enable,
                        intense_clean=settings.intense_clean,
                        click_sounds=settings.click_sounds,
                        melody_sounds=settings.melody_sounds,
                        warning_sounds=settings.warning_sounds,
                        bin_full_detect=settings.bin_full_detect,
                        led=settings.led,
                    )
                except Exception:
                    logger.debug("Settings poll failed", exc_info=True)

            # Bumper and analog sensor polling removed ? no longer published to HA.

            # --- Bumper sensors (serial) ---
            # Suppressed during cleaning: GPIO bumper_sensors handles real-time
            # detection then, so the serial poll only wastes ~35 ms of bus time
            # that could be a GetMotors update instead.
            if now - last_bumpers >= BUMPER_INTERVAL and not was_scan_active:
                last_bumpers = now
                try:
                    bumps = get_bumpers(serial)
                    mqtt.publish_bumpers(
                        left_front=bumps.left_front,
                        right_front=bumps.right_front,
                        left_side=bumps.left_side,
                        right_side=bumps.right_side,
                    )
                except Exception:
                    logger.debug("Bumper poll failed", exc_info=True)

            # Reconnect serial if too many consecutive failures (Neato rebooted)
            if serial_fail_count >= SERIAL_FAIL_LIMIT:
                logger.warning("Too many serial failures ? Neato may have rebooted. Reconnecting?")
                serial_fail_count = 0
                lds_active = False
                try:
                    serial.disconnect()
                except Exception:
                    pass
                time.sleep(5)
                reconnected = False
                while not _shutdown and not reconnected:
                    try:
                        serial.connect()
                        reconnected = True
                        logger.info("Serial reconnected to Neato")
                        # Re-compute SKey after reconnect
                        time.sleep(1.0)
                        try:
                            version = get_version(serial)
                            if version.serial_number:
                                skey = NeatoSerial.compute_skey(version.serial_number)
                                logger.info("SKey recomputed after reconnect")
                        except Exception as e:
                            logger.warning("SKey recomputation failed after reconnect: %s", e)
                    except Exception as exc:
                        logger.warning("Reconnect failed: %s ? retrying in 5s", exc)
                        time.sleep(5)

            # === Active-only polling (requires TestMode + LDS) ===
            # Also poll during autonomous cleaning ? robot spins its own LDS

            # Detect if robot is cleaning (LDS is spinning on its own)
            robot_cleaning = False
            try:
                ui = state.ui_state.upper() if 'state' in dir() else ""
                robot_cleaning = any(k in ui for k in (
                    "CLEANINGRUNNING", "CLEANINGPAUSED",
                    "STARTHOUSECLEAN", "STARTSPOTCLEAN",
                ))
            except Exception:
                pass

            scan_active = lds_active or robot_cleaning

            if scan_active:
                _tick_nogo_pulses(now)
            elif any(st["touching"] for st in nogo_seq.values()):
                logger.info("[main] no-go pulse reset (scan inactive)")
                for side in ("left", "right"):
                    _reset_nogo_side(side)
                bumper_sensors.release_virtual()

            # Reset odometry on idle?active transition to avoid
            # huge jumps from stale encoder positions
            if scan_active and not was_scan_active:
                logger.info("Scan active transition ? resetting odometry")
                odom = OdomState()
            was_scan_active = scan_active

            # --- Odometry + no-go guard: run every loop, not just when scan_active ---
            # GetMotors is a fast serial call (~20ms). Running it every iteration
            # gives ~10 Hz position updates ? enough to catch the robot mid-move.
            robot_cleaning_or_manual = scan_active
            try:
                motors = get_motors(serial)
                if motors:
                    _latest_motors["motors"] = motors
                    serial_fail_count = 0
                    if robot_cleaning_or_manual:
                        odom = update_odometry(odom, motors)
                        mqtt.publish_odom(
                            x=odom.x, y=odom.y, theta=odom.theta,
                            linear_vel=odom.linear_vel,
                            angular_vel=odom.angular_vel,
                            stamp=odom.timestamp,
                        )
                        if pipeline is not None:
                            pipeline.on_odom(odom)
                            # Forward latest external slam_toolbox pose into
                            # the pipeline so the overlay marker tracks it.
                            try:
                                sp = mqtt.get_slam_pose()
                                if sp is not None:
                                    sx, sy, sth, age, meta = sp
                                    if age < 2.0:
                                        pipeline.on_external_slam_pose(
                                            sx, sy, sth,
                                            mode=meta.get("mode", "unknown"),
                                            map_id=meta.get("map_id", ""),
                                        )
                            except Exception:
                                logger.debug("forward slam pose failed",
                                             exc_info=True)
                    _latest_odom["odom"] = odom
                    if scan_active:
                        gx, gy, gtheta, gsource = _get_guard_pose(odom)
                        no_go_guard.check(
                            gx, gy, gtheta,
                            odom.angular_vel, odom.linear_vel,
                            pose_source=gsource,
                        )
                        if now - last_nogo_pub >= NOGO_PUB_INTERVAL:
                            last_nogo_pub = now
                            try:
                                mqtt.publish_nogo_debug(
                                    no_go_guard.get_debug_snapshot()
                                )
                            except Exception:
                                logger.debug("publish_nogo_debug failed", exc_info=True)
                else:
                    serial_fail_count += 1
            except Exception:
                serial_fail_count += 1
                logger.debug("Odom poll failed", exc_info=True)

            if scan_active:
                # --- LIDAR scan (~5 Hz) ---
                if now - last_scan >= SCAN_INTERVAL:
                    last_scan = now
                    scan = get_lidar_scan(serial)
                    if scan:
                        mqtt.publish_scan(
                            ranges=scan.ranges,
                            intensities=scan.intensities,
                            angle_min=scan.angle_min,
                            angle_max=scan.angle_max,
                            angle_increment=scan.angle_increment,
                            range_min=scan.range_min,
                            range_max=scan.range_max,
                            rpm=scan.rpm,
                            stamp=scan.timestamp,
                        )
                        if pipeline is not None:
                            pipeline.on_scan(scan, odom)
                    # LDS blocked serial for ~300 ms ? re-poll odom+check immediately
                    # so the guard is up-to-date before the next loop sleep.
                    try:
                        motors2 = get_motors(serial)
                        if motors2:
                            _latest_motors["motors"] = motors2
                            odom = update_odometry(odom, motors2)
                            _latest_odom["odom"] = odom
                            gx, gy, gtheta, gsource = _get_guard_pose(odom)
                            no_go_guard.check(
                                gx, gy, gtheta,
                                odom.angular_vel, odom.linear_vel,
                                pose_source=gsource,
                            )
                    except Exception:
                        logger.debug("Post-LDS odom poll failed", exc_info=True)

            if now - last_serial_stats >= SERIAL_STATS_INTERVAL:
                last_serial_stats = now
                try:
                    mqtt.publish("serial_stats", serial.get_command_stats(reset=True), qos=0, retain=False)
                except Exception:
                    logger.debug("publish serial_stats failed", exc_info=True)

            # Brief sleep
            sleep_time = 0.005 if scan_active else 0.5
            time.sleep(sleep_time)

    except Exception:
        logger.exception("Unhandled error in main loop")
    finally:
        logger.info("Shutting down?")
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        bumper_sensors.stop()
        no_go_guard.cleanup()
        if lds_active:
            serial.set_motors(0, 0, 0)
            time.sleep(0.1)
            serial.set_lds_rotation(False)
            time.sleep(0.1)
            serial.set_test_mode(False)
        mqtt.disconnect()
        serial.disconnect()
        logger.info("ROSie driver stopped")


if __name__ == "__main__":
    main()

