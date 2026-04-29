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
# 12 m × 12 m at 800 × 800 pixels → ~1.5 cm / pixel.  Higher resolution
# than the original 500-px grid; map buffer is still only ~640 KB so it
# fits comfortably on the Pi Zero 2 W.  clean_map._safe_scale will auto-
# drop SCALE so peak render RAM stays in budget.
MAP_SIZE_PIXELS: int = 800
MAP_SIZE_METERS: float = 12.0
RESOLUTION: float = MAP_SIZE_METERS / MAP_SIZE_PIXELS  # ~0.015 m / pixel

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
            map_quality=80,           # walls are painted strongly per scan
                                       # so they survive small pose hops
                                       # without being carved away by the
                                       # next ray's wall-erase pass.
            hole_width_mm=60,         # very thin wall-erase corridor (was
                                       # 120) — reduces the "walls fade as
                                       # the vac moves" effect by limiting
                                       # how aggressively each ray erases
                                       # cells near its endpoint.
            random_seed=42,
            sigma_xy_mm=30,           # trust odometry more (was 50) now that
                                       # the pose_change feed uses signed
                                       # forward distance (see update()).
            sigma_theta_degrees=5,    # tighter rotation lock (was 10).
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

    # ── Scan input filtering ──────────────────────────────────────────────
    # 1) Intensity gate — drop returns the LDS reports as low-confidence
    #    (glass, dark surfaces, very-distant) which are the main source of
    #    phantom blobs floating away from real walls.
    # 2) Median-of-3 smoother — kill single-bin range spikes.
    INTENSITY_MIN = 100
    raw_ranges = scan.ranges
    intens = getattr(scan, "intensities", None)
    n = len(raw_ranges)
    cleaned = [0.0] * n
    for i in range(n):
        r = raw_ranges[i]
        if not math.isfinite(r):
            cleaned[i] = 0.0
            continue
        if intens is not None and i < len(intens) and intens[i] < INTENSITY_MIN:
            cleaned[i] = 0.0
            continue
        cleaned[i] = r

    scan_mm = [0] * n
    for i in range(n):
        a = cleaned[i - 1] if i > 0 else cleaned[-1]
        b = cleaned[i]
        c = cleaned[i + 1] if i < n - 1 else cleaned[0]
        # Median of 3 — but only count valid (>0) neighbours; if b is the
        # only valid one keep it as-is (don't over-erase isolated walls).
        vals = [v for v in (a, b, c) if v > 0.0]
        if not vals:
            continue
        vals.sort()
        r = vals[len(vals) // 2]
        if 0.020 <= r <= 5.0:
            scan_mm[i] = int(r * 1000)

    # ── Pose change from odometry ─────────────────────────────────────────
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
            # Skip stale deltas — a serial stall can hand us a huge dt with
            # an equally huge dx/dy that BreezySLAM will happily believe.
            if 0.001 < dt < 0.5:
                # SIGNED forward distance along the previous heading.
                # math.hypot(dx, dy) was unsigned, so any backward motion
                # (undock, recovery, bump-back) was reported as forward to
                # BreezySLAM and corrupted the pose track — this is the
                # root cause of the offset-blob constellations.
                forward_m = (dx * math.cos(_prev_theta)
                             + dy * math.sin(_prev_theta))
                dxy_mm = forward_m * 1000.0
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


def get_snapshot_array():
    """Return current SLAM grid as a numpy uint8 array oriented PGM-style
    (row 0 = top = +y_max), or None if SLAM is not running.

    Cheap: just copies the current mapbytes and flips rows. No flood-fill,
    no morphology — meant for live HA preview at a few-Hz cadence.

    Pixel codes (BreezySLAM raw):
        0           → wall OR unexplored (indistinguishable here)
        ~255 (high) → free / scanned floor
    """
    if _slam is None:
        return None
    try:
        import numpy as np
    except ImportError:
        return None

    with _lock:
        _slam.getmap(_mapbytes)
    m = np.frombuffer(bytes(_mapbytes), dtype=np.uint8).copy()
    m = m.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    return m[::-1, :]  # flip rows so row 0 = top (PGM convention)


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
