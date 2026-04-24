"""
slam.py — BreezySLAM wrapper for map creation on Pi Zero 2 W.

Replaces SLAM Toolbox.  Produces home.pgm + home.yaml in the same format
that clean_map.py expects (ROS 2 map_server occupancy grid convention):

    PGM:  P5 binary, uint8
          values < 64   → walls / occupied
          values > 190  → free floor
          values 64-190 → unknown / unexplored

    YAML: image, resolution, origin [ox, oy, 0.0]

BreezySLAM uses:
    0            → unvisited (initialised)  ← same code-point as "wall"
    ~0  (low)    → occupied / wall         ← laser hits
    ~255 (high)  → free                    ← laser pass-through

After slam.getmap() a border flood-fill remaps all border-connected 0-cells
to 127 (unknown), leaving interior 0-cells as walls.  clean_map.py then
classifies them correctly via its < 64 / > 190 thresholds.

Thread-safety: update() is called from the main serial loop (~5 Hz).
save_map() is called from the pipeline thread.  A single Lock guards _slam.
"""

import logging
import math
import threading
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
# 12 m × 12 m at 500 × 500 pixels → ~2.4 cm / pixel.  Generous for a home.
MAP_SIZE_PIXELS: int = 500
MAP_SIZE_METERS: float = 12.0
RESOLUTION: float = MAP_SIZE_METERS / MAP_SIZE_PIXELS  # ~0.024 m / pixel

# ---------------------------------------------------------------------------
# Module state
# ---------------------------------------------------------------------------
_slam = None                           # RMHC_SLAM instance
_mapbytes: Optional[bytearray] = None  # flat MAP_SIZE_PIXELS² buffer
_lock = threading.Lock()

# Odometry tracking (deltas fed to BreezySLAM pose_change)
_prev_x: float = 0.0
_prev_y: float = 0.0
_prev_theta: float = 0.0
_prev_time: float = 0.0
_odom_ready: bool = False


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def start() -> None:
    """Initialise a fresh SLAM session.

    The robot's starting position (dock) becomes world (0, 0).
    Raises RuntimeError if breezyslam is not installed.
    """
    global _slam, _mapbytes, _odom_ready
    global _prev_x, _prev_y, _prev_theta, _prev_time

    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
    except ImportError as exc:
        raise RuntimeError(
            "breezyslam not installed — run: pip install breezyslam"
        ) from exc

    # Neato D6 LDS: 360 beams, ~5 Hz, 360° FOV, 5 m max range
    laser = Laser(
        scan_size=360,
        scan_rate_hz=5,
        detection_angle_degrees=360,
        distance_no_detection_mm=5000,
        detection_margin=0,
        offset_mm=0,
    )

    with _lock:
        _slam = RMHC_SLAM(
            laser,
            MAP_SIZE_PIXELS,
            MAP_SIZE_METERS,
            map_quality=50,
            hole_width_mm=600,
            random_seed=42,
            sigma_xy_mm=100,
            sigma_theta_degrees=20,
        )
        _mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        _odom_ready = False
        _prev_x = _prev_y = _prev_theta = _prev_time = 0.0

    logger.info(
        "BreezySLAM started (%d×%d grid, %.3f m/px, %.0f m coverage)",
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, RESOLUTION, MAP_SIZE_METERS,
    )


def update(scan, odom) -> None:
    """Feed one LidarScan + OdomState into SLAM.

    Args:
        scan: LidarScan from lidar.py  (scan.ranges in metres, scan.timestamp)
        odom: OdomState from odometry.py  (x, y, theta in m / rad, timestamp)

    Safe to call at ~5 Hz from the main serial loop.
    Silently returns if SLAM is not started.
    """
    global _prev_x, _prev_y, _prev_theta, _prev_time, _odom_ready

    if _slam is None:
        return

    # Convert scan ranges to mm integers (0 = no detection in BreezySLAM)
    scan_mm = [
        int(r * 1000)
        if (math.isfinite(r) and 20.0 <= r * 1000.0 <= 5000.0)
        else 0
        for r in scan.ranges
    ]

    # Compute pose change from odometry deltas
    pose_change = None
    if odom is not None:
        now = odom.timestamp
        if _odom_ready:
            dx = odom.x - _prev_x
            dy = odom.y - _prev_y
            dtheta = odom.theta - _prev_theta

            # Normalise angle delta to (-π, π)
            while dtheta > math.pi:
                dtheta -= 2.0 * math.pi
            while dtheta < -math.pi:
                dtheta += 2.0 * math.pi

            dt = now - _prev_time
            if dt > 0.001:
                dxy_mm = math.hypot(dx, dy) * 1000.0       # m → mm
                dtheta_deg = math.degrees(dtheta)
                pose_change = (dxy_mm, dtheta_deg, dt)
        else:
            _odom_ready = True

        _prev_x = odom.x
        _prev_y = odom.y
        _prev_theta = odom.theta
        _prev_time = now

    with _lock:
        _slam.update(scan_mm, pose_change)


def get_position() -> tuple[float, float, float]:
    """Return current SLAM estimate as (x_m, y_m, theta_rad) in world frame.

    World frame: dock = (0, 0, 0), X forward, Y left, theta CCW.
    Returns (0, 0, 0) if SLAM is not started.
    """
    if _slam is None:
        return 0.0, 0.0, 0.0

    with _lock:
        x_mm, y_mm, theta_deg = _slam.getpos()

    # BreezySLAM map origin is bottom-left; robot starts at map centre.
    # Convert to world frame where dock = (0, 0).
    half_m = MAP_SIZE_METERS / 2.0
    x_world = (x_mm / 1000.0) - half_m
    y_world = (y_mm / 1000.0) - half_m
    theta_rad = math.radians(theta_deg)
    return x_world, y_world, theta_rad


def save_map(map_dir: Path) -> tuple[Path, Path]:
    """Save current SLAM state to home.pgm + home.yaml.

    Applies a border flood-fill to remap unexplored zero-cells to 127
    so clean_map.py's thresholds work correctly.

    Returns (yaml_path, pgm_path).
    Raises RuntimeError if SLAM is not started.
    """
    if _slam is None:
        raise RuntimeError("SLAM not started — call slam.start() first")

    try:
        import numpy as np
        from scipy.ndimage import binary_propagation
    except ImportError as exc:
        raise RuntimeError("numpy / scipy required for save_map()") from exc

    map_dir = Path(map_dir)
    map_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = map_dir / "home.pgm"
    yaml_path = map_dir / "home.yaml"

    # Grab the current map snapshot
    with _lock:
        _slam.getmap(_mapbytes)

    # ── Post-process: distinguish unexplored (0) from walls (also 0) ──────
    # Flood-fill from all four borders through the zero cells.  Any zero cell
    # reachable from the border without crossing explored (non-zero) space is
    # "unexplored" → remap to 127 (unknown).  Interior zero cells (actual walls)
    # stay at 0.
    m = np.frombuffer(bytes(_mapbytes), dtype=np.uint8).copy()
    m = m.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)

    zero_mask = (m == 0)

    # Seed from all border pixels that are zero
    seed = np.zeros_like(zero_mask)
    seed[0, :] = zero_mask[0, :]
    seed[-1, :] = zero_mask[-1, :]
    seed[:, 0] = zero_mask[:, 0]
    seed[:, -1] = zero_mask[:, -1]

    # Propagate seeds through zero_mask
    unexplored = binary_propagation(seed, mask=zero_mask)
    m[unexplored] = 127  # remap border-connected zeros to "unknown"

    # ── Write PGM (P5 binary) ──────────────────────────────────────────────
    # BreezySLAM row 0 = y_min (bottom); PGM row 0 = y_max (top) → flip rows.
    header = f"P5\n{MAP_SIZE_PIXELS} {MAP_SIZE_PIXELS}\n255\n".encode("ascii")
    with open(pgm_path, "wb") as fh:
        fh.write(header)
        for row in range(MAP_SIZE_PIXELS - 1, -1, -1):
            fh.write(m[row].tobytes())

    # ── Write YAML ─────────────────────────────────────────────────────────
    # Map origin = world coords of the bottom-left corner (pixel 0, 0).
    # Robot started at map centre = world (0, 0) → bottom-left = (-half, -half).
    half = MAP_SIZE_METERS / 2.0
    yaml_text = (
        f"image: home.pgm\n"
        f"resolution: {RESOLUTION:.6f}\n"
        f"origin: [{-half:.4f}, {-half:.4f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    yaml_path.write_text(yaml_text)

    logger.info(
        "Map saved: %s + %s (%d×%d, %.4f m/px)",
        pgm_path.name, yaml_path.name,
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, RESOLUTION,
    )
    return yaml_path, pgm_path


def stop() -> None:
    """Release BreezySLAM resources."""
    global _slam, _mapbytes
    with _lock:
        _slam = None
        _mapbytes = None
    logger.info("BreezySLAM stopped")


def is_running() -> bool:
    """Return True if a SLAM session is currently active."""
    return _slam is not None
