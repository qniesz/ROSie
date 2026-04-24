"""
lidar.py — LIDAR scan acquisition from the Neato D6.

Parses the GetLDSScan response into a list of 360 range values (in meters)
and the LIDAR rotation speed (RPM).
"""

import logging
import math
from dataclasses import dataclass, field

from .serial_handler import NeatoSerial, LIDAR_POINTS

logger = logging.getLogger(__name__)


@dataclass
class LidarScan:
    """One complete 360-degree LIDAR scan."""
    ranges: list[float] = field(default_factory=lambda: [0.0] * LIDAR_POINTS)
    intensities: list[float] = field(default_factory=lambda: [0.0] * LIDAR_POINTS)
    rpm: float = 0.0
    timestamp: float = 0.0  # time.monotonic() when scan completed

    # Scan geometry (fixed for Neato D6)
    angle_min: float = 0.0
    angle_max: float = (359.0 * math.pi / 180.0)  # last bin at 359 degrees
    angle_increment: float = math.pi / 180.0
    range_min: float = 0.020  # 20mm
    range_max: float = 5.0    # 5m


def get_lidar_scan(serial: NeatoSerial) -> LidarScan | None:
    """
    Request and parse a LIDAR scan from the Neato.

    Returns a LidarScan with 360 range values in meters, or None on failure.

    The GetLDSScan response format:
        AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
        0,1234,56,0
        1,1230,54,0
        ...
        359,1228,55,0
        ROTATION_SPEED,5.01
    """
    import time

    lines = serial.send_and_collect("GetLDSScan", "AngleInDegrees", timeout=1.5)
    if not lines:
        logger.debug("No LIDAR data received")
        return None

    scan = LidarScan(timestamp=time.monotonic())
    ranges = [0.0] * LIDAR_POINTS
    intensities = [0.0] * LIDAR_POINTS

    for line in lines:
        parts = line.split(",")
        if len(parts) < 4:
            # Check for ROTATION_SPEED footer line
            if len(parts) == 2 and parts[0].strip().startswith("ROTATION_SPEED"):
                try:
                    scan.rpm = float(parts[1].strip())
                except ValueError:
                    pass
            continue

        try:
            angle = int(parts[0].strip())
            distance_mm = int(parts[1].strip())
            intensity = int(parts[2].strip())
            error_code = int(parts[3].strip())
        except (ValueError, IndexError):
            continue

        if 0 <= angle < LIDAR_POINTS:
            if error_code == 0 and distance_mm > 0:
                ranges[angle] = distance_mm / 1000.0  # mm -> meters
                intensities[angle] = float(intensity)
            else:
                # Invalid reading — use inf per ROS convention
                ranges[angle] = float('inf')
                intensities[angle] = 0.0

    scan.ranges = ranges
    scan.intensities = intensities
    return scan
