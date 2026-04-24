"""
main.py — ROSie Pi-side driver entry point.

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
import logging
import os
import signal
import subprocess
import sys
import time

from .serial_handler import NeatoSerial
from .lidar import get_lidar_scan
from .odometry import OdomState, get_motors, update_odometry
from .sensors import get_battery, get_bumpers, get_robot_state, get_user_settings, get_version, format_model, format_firmware
from .commands import handle_command, handle_cmd_vel
from .mqtt_bridge import MQTTBridge
from . import no_go_guard
from . import bumper_sensors

# MapPipeline is optional — enabled by ROSIE_MAP_PIPELINE_ENABLED (default on).
# When disabled (Pi 4 / legacy deploy), behaviour is completely unchanged.
try:
    from .map_pipeline import MapPipeline as _MapPipeline
except ImportError:
    _MapPipeline = None

logger = logging.getLogger("rosie")

# Timing intervals (seconds)
SCAN_INTERVAL     = 0.20     # ~5 Hz (active mode only)
STATE_INTERVAL    = 10.0    # ~0.1 Hz — poll GetErr + GetState
CHARGER_INTERVAL  = 10.0  # ~0.1 Hz — poll GetCharger
SETTINGS_INTERVAL = 60.0 # ~once per minute — poll GetUserSettings
NOGO_LINES_FILE = os.environ.get("ROSIE_NOGO_LINES_FILE", "/home/rosie/nogo-lines.json")


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

_shutdown = False


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
    p.add_argument("--mqtt-prefix", default="rosie")
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
    # Initialize MQTT early — before serial — so GPIO/bumper commands
    # work even when the Neato is not yet connected
    # ------------------------------------------------------------------
    mqtt = MQTTBridge(
        host=args.mqtt_host,
        port=args.mqtt_port,
        username=args.mqtt_user,
        password=args.mqtt_pass,
        prefix=args.mqtt_prefix,
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
        logger.info("Activating LDS…")
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
        logger.info("Deactivating LDS…")
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
            logger.info("New map requested — no-go lines cleared: %s", msg)
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
                logger.warning("Reboot requested but no pipeline — ignoring")
        elif cmd_lower == "test_bumper_fl":
            bumper_sensors.test_pin("front_left")
        elif cmd_lower == "test_bumper_fr":
            bumper_sensors.test_pin("front_right")
        elif cmd_lower == "test_bumper_sl":
            bumper_sensors.test_pin("side_left")
        elif cmd_lower == "test_bumper_sr":
            bumper_sensors.test_pin("side_right")
        else:
            handle_command(serial, cmd, skey)

    def on_cmd_vel(lx: float, az: float):
        if not lds_active:
            logger.debug("cmd_vel ignored — LDS not active")
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
            logger.warning("MapPipeline init failed (%s) — running without it", exc)
            pipeline = None

    nogo_pulse_hold_secs = NOGO_PULSE_MS / 1000.0
    nogo_pulse_interval_secs = NOGO_PULSE_INTERVAL_MS / 1000.0
    nogo_seq = {
        "left": {
            "touching": False,
            "is_front": False,
            "side_pulses": 0,
            "next_pulse_at": 0.0,
            "phase": "",
        },
        "right": {
            "touching": False,
            "is_front": False,
            "side_pulses": 0,
            "next_pulse_at": 0.0,
            "phase": "",
        },
    }

    def _reset_nogo_side(side: str) -> None:
        st = nogo_seq[side]
        st["touching"] = False
        st["is_front"] = False
        st["side_pulses"] = 0
        st["next_pulse_at"] = 0.0
        st["phase"] = ""

    def _fire_nogo_pulse(side: str, now_ts: float) -> None:
        st = nogo_seq[side]
        if not st["touching"]:
            return

        use_front = bool(st["is_front"]) or st["side_pulses"] >= NOGO_SIDE_PULSES_BEFORE_FRONT
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

    def _tick_nogo_pulses(now_ts: float) -> None:
        for side, st in nogo_seq.items():
            if st["touching"] and now_ts >= st["next_pulse_at"]:
                _fire_nogo_pulse(side, now_ts)

    # Set up no-go guard touch callback — fires the relevant virtual bumper
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
            logger.info("[main] no-go touch start side=%s front=%s", side, is_front)
            _fire_nogo_pulse(side, time.monotonic())
        else:
            if st["touching"]:
                logger.info("[main] no-go touch release side=%s", side)
            _reset_nogo_side(side)
    no_go_guard.set_touch_callback(_on_nogo_touch)

    # Stop callback for bumper_sensors — sends stop-cleaning to Neato
    def _do_stop() -> None:
        handle_command(serial, "stop", skey)
    bumper_sensors.set_stop_callback(_do_stop)

    # Publish initial no-go and bumper state (no serial needed)
    mqtt.publish_nogo_lines(no_go_guard.export_lines())
    if no_go_guard.is_enabled():
        mqtt.publish_nogo_status("ok", f"{no_go_guard.line_count()} no-go line(s) active")
    else:
        mqtt.publish_nogo_status("disabled", "disabled by ROSIE_NOGO_ENABLED")
    mqtt.publish_bumpers(False, False, False, False)  # initial state: all clear

    # ------------------------------------------------------------------
    # Connect serial (retry loop — MQTT is already up)
    # ------------------------------------------------------------------
    retry_delay = 5
    while not _shutdown:
        try:
            serial.connect()
            break
        except Exception as exc:
            logger.warning("Cannot open serial port: %s — retrying in %ds", exc, retry_delay)
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
            logger.warning("Could not read serial number — SetEvent commands disabled")

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
        logger.warning("SKey computation failed: %s — SetEvent commands disabled", exc)

    # Publish initial robot state now that serial is up
    mqtt.publish_state(
        ui_state="UIMGR_STATE_IDLE",
        robot_state="ST_C_Off",
        error="none", alert="none",
    )

    logger.info("ROSie driver running — ready for commands (SKey %s)",
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
    BUMPER_INTERVAL = 1.0   # poll bumpers via serial every 1s
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

            # Bumper and analog sensor polling removed — no longer published to HA.

            # --- Bumper sensors (serial) ---
            if now - last_bumpers >= BUMPER_INTERVAL:
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
                logger.warning("Too many serial failures — Neato may have rebooted. Reconnecting…")
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
                        logger.warning("Reconnect failed: %s — retrying in 5s", exc)
                        time.sleep(5)

            # === Active-only polling (requires TestMode + LDS) ===
            # Also poll during autonomous cleaning — robot spins its own LDS

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

            # Reset odometry on idle→active transition to avoid
            # huge jumps from stale encoder positions
            if scan_active and not was_scan_active:
                logger.info("Scan active transition — resetting odometry")
                odom = OdomState()
            was_scan_active = scan_active

            # --- Odometry + no-go guard: run every loop, not just when scan_active ---
            # GetMotors is a fast serial call (~20ms). Running it every iteration
            # gives ~10 Hz position updates — enough to catch the robot mid-move.
            robot_cleaning_or_manual = scan_active
            try:
                motors = get_motors(serial)
                if motors:
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
                    if scan_active:
                        no_go_guard.check(odom.x, odom.y, odom.theta, odom.angular_vel, odom.linear_vel)
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
                    # LDS blocked serial for ~300 ms — re-poll odom+check immediately
                    # so the guard is up-to-date before the next loop sleep.
                    try:
                        motors2 = get_motors(serial)
                        if motors2:
                            odom = update_odometry(odom, motors2)
                            no_go_guard.check(odom.x, odom.y, odom.theta, odom.angular_vel, odom.linear_vel)
                    except Exception:
                        logger.debug("Post-LDS odom poll failed", exc_info=True)

            # Brief sleep
            sleep_time = 0.005 if scan_active else 0.5
            time.sleep(sleep_time)

    except Exception:
        logger.exception("Unhandled error in main loop")
    finally:
        logger.info("Shutting down…")
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
