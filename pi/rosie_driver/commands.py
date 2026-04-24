"""
commands.py — Translate HA / ROS 2 commands to Neato serial actions.

Handles:
  - HA vacuum commands (start, stop, pause, return_to_base, locate, clean_spot)
  - Brainslug-style SetEvent commands (cleaning, manual driving)
  - Robot settings (eco mode, wall follower, etc.)
  - cmd_vel twist messages → differential-drive wheel commands
"""

import logging
import math
import time

from .serial_handler import NeatoSerial, BASE_WIDTH_MM, MAX_SPEED_MM_S

logger = logging.getLogger(__name__)

# cmd_vel → SetMotor timing
CMD_VEL_PERIOD_S = 0.1  # how long each SetMotor burst lasts (100ms)

# --- SetEvent names (gen3 / D6) ---
EVENTS = {
    # Cleaning
    "house_clean": "UIMGR_EVENT_SMARTAPP_START_HOUSE_CLEANING",
    "spot_clean": "UIMGR_EVENT_SMARTAPP_START_SPOT_CLEANING",
    "stop_cleaning": "UIMGR_EVENT_SMARTAPP_STOP_CLEANING",
    "pause_cleaning": "UIMGR_EVENT_SMARTAPP_PAUSE_CLEANING",
    "resume_cleaning": "UIMGR_EVENT_SMARTAPP_RESUME_CLEANING",
    "send_to_base": "UIMGR_EVENT_SMARTAPP_SEND_TO_BASE",
    # Virtual boundary (triggers magnetic strip response during cleaning)
    "mag_strip": "UIMGR_EVENT_MAG_STRIP_DETECTED",
    # Manual driving (down = start moving, up = stop moving)
    "manual_forward_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_FORWARD_DOWN",
    "manual_forward_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_FORWARD_UP",
    "manual_backwards_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_BACKWARDS_DOWN",
    "manual_backwards_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_BACKWARDS_UP",
    "manual_turn_left_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_TURN_LEFT_DOWN",
    "manual_turn_left_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_TURN_LEFT_UP",
    "manual_turn_right_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_TURN_RIGHT_DOWN",
    "manual_turn_right_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_TURN_RIGHT_UP",
    "manual_arc_left_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_ARC_LEFT_DOWN",
    "manual_arc_left_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_ARC_LEFT_UP",
    "manual_arc_right_down": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_ARC_RIGHT_DOWN",
    "manual_arc_right_up": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_ARC_RIGHT_UP",
    "manual_btn_timeout": "UIMGR_EVENT_SMARTAPP_DRIVE_MANUAL_BTN_TIMEOUT",
    "start_manual_cleaning": "UIMGR_EVENT_SMARTAPP_START_MANUAL_CLEANING",
}

# Map HA vacuum command payloads → our command names
HA_COMMAND_MAP = {
    "start": "house_clean",
    "stop": "stop_cleaning",
    "pause": "pause_cleaning",
    "resume": "resume_cleaning",
    "return_to_base": "send_to_base",
    "locate": "locate",
    "clean_spot": "spot_clean",
}

# Settings name map: our command name → Neato SetUserSettings parameter
SETTINGS_MAP = {
    "eco_mode": ("EcoMode", "eco_mode"),
    "wall_enable": ("WallEnable", "wall_enable"),
    "intense_clean": ("IntenseClean", "intense_clean"),
    "click_sounds": ("ButtonClick", "click_sounds"),
    "melody_sounds": ("Melodies", "melody_sounds"),
    "warning_sounds": ("Warnings", "warning_sounds"),
    "bin_full_detect": ("BinFullDetect", "bin_full_detect"),
    "led": ("StealthLED", "led"),
}


def handle_command(serial: NeatoSerial, command: str, skey: str = "") -> None:
    """
    Execute a command received from HA or MQTT.

    Uses SetEvent for cleaning/driving, direct serial for settings/utility.
    """
    command = command.strip().lower()

    # Map HA vacuum commands to our internal names
    if command in HA_COMMAND_MAP:
        command = HA_COMMAND_MAP[command]

    logger.info("Executing command: %s", command)

    # --- SetEvent commands (cleaning + manual driving) ---
    if command in EVENTS:
        if not skey:
            logger.error("Cannot send SetEvent — no SKey available")
            return
        serial.send_event(EVENTS[command], skey)

    # --- Locate (direct PlaySound, no SKey needed) ---
    elif command == "locate":
        serial.play_sound(20)

    # --- Spot clean with custom dimensions ---
    elif command.startswith("spot_clean_hw:"):
        # Format: "spot_clean_hw:WIDTH,HEIGHT"
        try:
            dims = command.split(":", 1)[1]
            w, h = dims.split(",")
            serial.send_command(f"Clean Spot Width {int(w)} Height {int(h)}")
        except (ValueError, IndexError):
            logger.warning("Invalid spot_clean_hw format: %s", command)

    # --- Settings toggles ---
    elif command.startswith("set_") and command.endswith(("_on", "_off")):
        # Format: "set_eco_mode_on" or "set_eco_mode_off"
        if command.endswith("_on"):
            setting_name = command[4:-3]  # strip "set_" and "_on"
            value = "ON"
        else:
            setting_name = command[4:-4]  # strip "set_" and "_off"
            value = "OFF"

        if setting_name in SETTINGS_MAP:
            neato_name = SETTINGS_MAP[setting_name][0]
            serial.set_user_setting(neato_name, value)
        else:
            logger.warning("Unknown setting: %s", setting_name)

    # --- Navigation mode ---
    elif command.startswith("set_nav_mode:"):
        mode = command.split(":", 1)[1].strip()
        serial.set_navigation_mode(mode)

    # --- Direct vacuum motor control (TestMode) ---
    elif command == "vacuum_on":
        serial.set_test_mode(True)
        serial.set_vacuum(True, 65)
    elif command == "vacuum_off":
        serial.set_vacuum(False)
        serial.set_test_mode(False)
    elif command.startswith("set_vacuum_speed:"):
        try:
            speed = int(command.split(":", 1)[1])
            serial.set_test_mode(True)
            serial.set_vacuum(True, speed)
        except (ValueError, IndexError):
            logger.warning("Invalid vacuum speed: %s", command)

    # --- Utility commands ---
    elif command == "update_status":
        # Triggers a full data refresh (handled in main loop)
        pass  # main.py will see this and force-poll

    elif command == "clear_errors":
        serial.clear_errors()

    elif command == "shutdown":
        serial.set_system_mode("Shutdown")

    elif command == "powercycle":
        serial.set_system_mode("PowerCycle")

    else:
        logger.warning("Unknown command: %s", command)


def handle_cmd_vel(serial: NeatoSerial,
                   linear_x: float, angular_z: float) -> None:
    """
    Convert a twist-style velocity command into a SetMotor call.

    Args:
        linear_x:  forward velocity in m/s (positive = forward)
        angular_z: rotation in rad/s (positive = counter-clockwise)
    """
    base_width_m = BASE_WIDTH_MM / 1000.0
    v_left = linear_x - (angular_z * base_width_m / 2.0)
    v_right = linear_x + (angular_z * base_width_m / 2.0)

    left_dist_mm = int(v_left * CMD_VEL_PERIOD_S * 1000)
    right_dist_mm = int(v_right * CMD_VEL_PERIOD_S * 1000)
    speed_mm = int(min(max(abs(v_left), abs(v_right)) * 1000, MAX_SPEED_MM_S))

    if left_dist_mm == 0 and right_dist_mm == 0:
        return

    serial.set_motors(left_dist_mm, right_dist_mm, speed_mm)
