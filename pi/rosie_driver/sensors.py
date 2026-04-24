"""
sensors.py — Battery, bumper, error, and state polling for Neato D6.

Provides functions to query all non-LIDAR/non-motor sensor data.
"""

import logging
from dataclasses import dataclass, field


from .serial_handler import NeatoSerial

logger = logging.getLogger(__name__)


@dataclass
class BatteryState:
    fuel_percent: float = 0.0
    voltage: float = 0.0
    charging_active: bool = False
    charging_enabled: bool = False
    ext_power_present: bool = False
    battery_over_temp: bool = False
    battery_temp_c: float = 0.0
    on_reserved_fuel: bool = False
    empty_fuel: bool = False
    battery_failure: bool = False
    charger_mah: float = 0.0
    discharge_mah: float = 0.0



@dataclass
class BumperState:
    left_side: bool = False
    right_side: bool = False
    left_front: bool = False
    right_front: bool = False
    wheel_drop_left: bool = False   # SNSR_LEFT_WHEEL_EXTENDED
    wheel_drop_right: bool = False  # SNSR_RIGHT_WHEEL_EXTENDED


@dataclass
class AnalogSensorState:
    mag_left: float = 0.0    # MagSensorLeft  — left HAL/magnetic wall sensor
    mag_right: float = 0.0   # MagSensorRight — right HAL/magnetic wall sensor
    wall_mm: float = 0.0     # WallSensor     — front wall distance (mm)
    drop_left_mm: float = 0.0   # DropSensorLeft  — left cliff/drop sensor (mm)
    drop_right_mm: float = 0.0  # DropSensorRight — right cliff/drop sensor (mm)


@dataclass
class RobotState:
    ui_state: str = ""
    robot_state: str = ""
    error: str = ""
    alert: str = ""


@dataclass
class UserSettings:
    eco_mode: bool = False
    wall_enable: bool = False
    intense_clean: bool = False
    click_sounds: bool = False
    melody_sounds: bool = False
    warning_sounds: bool = False
    bin_full_detect: bool = False
    led: bool = False


def get_battery(serial: NeatoSerial) -> BatteryState:
    """Read charger/battery info via GetCharger."""
    lines = serial.send_and_collect("GetCharger", "Label", timeout=1.0)
    batt = BatteryState()

    for line in lines:
        parts = line.split(",")
        if len(parts) < 2:
            continue
        key = parts[0].strip()
        val = parts[1].strip()

        try:
            if key == "FuelPercent":
                batt.fuel_percent = float(val)
            elif key == "BatteryOverTemp":
                batt.battery_over_temp = val == "1"
            elif key == "ChargingActive":
                batt.charging_active = val == "1"
            elif key == "ChargingEnabled":
                batt.charging_enabled = val == "1"
            elif key == "ConfidentOnFuel":
                pass  # not critical
            elif key == "OnReservedFuel":
                batt.on_reserved_fuel = val == "1"
            elif key == "EmptyFuel":
                batt.empty_fuel = val == "1"
            elif key == "BatteryFailure":
                batt.battery_failure = val == "1"
            elif key == "ExtPwrPresent":
                batt.ext_power_present = val == "1"
            elif key == "BattTempCAvg":
                batt.battery_temp_c = float(val)
            elif key == "VBattV":
                batt.voltage = float(val)
            elif key == "Charger_mAH":
                batt.charger_mah = float(val)
            elif key == "Discharge_mAH":
                batt.discharge_mah = float(val)
        except ValueError:
            logger.debug("Could not parse charger field %s=%s", key, val)

    return batt



def get_bumpers(serial: NeatoSerial) -> BumperState:
    """Read digital bump and wheel drop sensors via GetDigitalSensors."""
    lines = serial.send_and_collect("GetDigitalSensors", "Digital Sensor Name", timeout=1.0)
    bumpers = BumperState()

    for line in lines:
        parts = line.split(",")
        if len(parts) < 2:
            continue
        key = parts[0].strip()
        val = parts[1].strip()

        if key == "LSIDEBIT":
            bumpers.left_side = val == "1"
        elif key == "RSIDEBIT":
            bumpers.right_side = val == "1"
        elif key == "LFRONTBIT":
            bumpers.left_front = val == "1"
        elif key == "RFRONTBIT":
            bumpers.right_front = val == "1"
        elif key == "SNSR_LEFT_WHEEL_EXTENDED":
            bumpers.wheel_drop_left = val == "1"
        elif key == "SNSR_RIGHT_WHEEL_EXTENDED":
            bumpers.wheel_drop_right = val == "1"

    return bumpers


def get_analog_sensors(serial: NeatoSerial) -> AnalogSensorState:
    """Read analog sensors via GetAnalogSensors.

    Format returned by robot: SensorName,Unit,Value,
    """
    lines = serial.send_and_collect("GetAnalogSensors", "SensorName", timeout=1.0)
    state = AnalogSensorState()
    for line in lines:
        parts = line.split(",")
        if len(parts) < 3:
            continue
        key = parts[0].strip()
        val = parts[2].strip()
        try:
            if key == "MagSensorLeft":
                state.mag_left = float(val)
            elif key == "MagSensorRight":
                state.mag_right = float(val)
            elif key == "WallSensor":
                state.wall_mm = float(val)
            elif key == "DropSensorLeft":
                state.drop_left_mm = float(val)
            elif key == "DropSensorRight":
                state.drop_right_mm = float(val)
        except ValueError:
            continue
    return state


def get_robot_state(serial: NeatoSerial) -> RobotState:
    """
    Read robot error and state info.

    Calls GetErr and GetState separately.
    """
    state = RobotState()

    # Get errors/alerts
    err_lines = serial.send_and_collect("GetErr", "Error", timeout=1.0)
    if len(err_lines) >= 3:
        # Format: Error\n<error_val>\nAlert\n<alert_val>\n...
        state.error = err_lines[0].strip() if err_lines else ""
        # Find the Alert value
        for i, line in enumerate(err_lines):
            if line.strip() == "Alert" and i + 1 < len(err_lines):
                state.alert = err_lines[i + 1].strip()
                break
    elif err_lines:
        state.error = err_lines[0].strip()

    # Get UI state
    serial.flush()
    serial.send_command("GetState")
    deadline_lines = []
    while True:
        line, is_last = serial.get_response(timeout=1.0)
        if not line:
            break
        deadline_lines.append(line)
        if is_last:
            break

    for line in deadline_lines:
        if "Current UI State is:" in line:
            state.ui_state = line.split("Current UI State is:")[-1].strip()
        elif "Current Robot State is:" in line:
            raw = line.split("Current Robot State is:")[-1].strip()
            # Remove trailing Ctrl-Z if present
            state.robot_state = raw.replace("\x1a", "").strip()

    return state


def get_user_settings(serial: NeatoSerial) -> UserSettings:
    """Read user-configurable settings via GetUserSettings."""
    lines = serial.send_and_collect("GetUserSettings", "GetUserSettings", timeout=1.0)
    settings = UserSettings()

    for line in lines:
        parts = line.split(",")
        if len(parts) < 2:
            continue
        key = parts[0].strip()
        val = parts[1].strip().upper()

        if key in ("EcoMode", "Eco Mode"):
            settings.eco_mode = val == "ON"
        elif key == "Wall Enable":
            settings.wall_enable = val == "ON"
        elif key == "IntenseClean":
            settings.intense_clean = val == "ON"
        elif key in ("ClickSounds", "Click Sounds"):
            settings.click_sounds = val == "ON"
        elif key == "Melody Sounds":
            settings.melody_sounds = val == "ON"
        elif key == "Warning Sounds":
            settings.warning_sounds = val == "ON"
        elif key == "Bin Full Detect":
            settings.bin_full_detect = val == "ON"
        elif key == "LED":
            settings.led = val == "ON"

    return settings


@dataclass
class VersionInfo:
    serial_number: str = ""
    model: str = ""
    software: str = ""


def get_version(serial: NeatoSerial) -> VersionInfo:
    """Read robot version info via GetVersion."""
    lines = serial.send_and_collect("GetVersion", "GetVersion", timeout=2.0)
    info = VersionInfo()

    for line in lines:
        # Use split with maxsplit=1 because serial number contains a comma
        parts = line.split(",", 1)
        if len(parts) < 2:
            continue
        key = parts[0].strip()
        val = parts[1].strip()

        if key == "Serial Number":
            info.serial_number = val
        elif key == "Model":
            info.model = val
        elif key == "Software":
            info.software = val

    return info
