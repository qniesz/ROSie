"""
map_pipeline.py — Map-creation pipeline and map-overlay supervisor.

Runs on Pi Zero 2 W alongside the serial driver, replacing rosie_server.py
on the Pi 4.  All ROS 2 / Docker dependencies are removed; mapping uses
BreezySLAM (see slam.py) and image processing uses the existing clean_map.py.

Responsibilities
----------------
* HA MQTT Discovery for map entities (camera, pipeline sensor, create-map
  button, map-meta sensor, diagnostics, reboot button).
* create_map pipeline state machine:
    clearing → mapping → waiting_for_dock → saving → processing →
    publishing → idle
* Live map snapshots every 10 s while the robot is cleaning.
* Persistent dock position (dock.json).
* Periodic map overlay refresh (dock + robot + no-go lines drawn on the
  base clean_map.py PNG and published as base64 JPEG to HA camera entity).
* System diagnostics (CPU %, RAM MB) published every 30 s.

Integration with main.py
------------------------
MapPipeline.on_scan(scan, odom)   — called from serial loop at ~5 Hz
MapPipeline.on_state(ui, ext_pw)  — called after GetState / GetCharger polls
MapPipeline.on_odom(odom)         — called after every GetMotors cycle
MapPipeline.on_nogo_lines(lines)  — called after no-go lines are updated
MapPipeline.on_command(cmd)       — receives 'create_map', 'reboot'
MapPipeline.start()               — call once, after MQTT is connected
MapPipeline.stop()                — call during shutdown
"""

import base64
import io
import json
import logging
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

from PIL import Image, ImageDraw, ImageFont

from . import slam as _slam_mod

logger = logging.getLogger("rosie.pipeline")

# ---------------------------------------------------------------------------
# Pipeline state names
# ---------------------------------------------------------------------------
IDLE              = "idle"
CLEARING          = "clearing"
MAPPING           = "mapping"
WAITING_FOR_DOCK  = "waiting_for_dock"
SAVING            = "saving"
PROCESSING        = "processing"
PUBLISHING        = "publishing"
ERROR             = "error"

# ---------------------------------------------------------------------------
# Visual constants (match rosie_server.py / clean_map.py)
# ---------------------------------------------------------------------------
DOCK_COLOUR  = (34, 170, 85)
ROBOT_COLOUR = (41, 121, 255)
NOGO_COLOUR  = (220, 40, 40)
BG_COLOUR    = (240, 245, 250)

# Default map directory (no Docker volume; override via MAP_DIR env var)
DEFAULT_MAP_DIR = Path(os.environ.get("ROSIE_MAP_DIR", "/home/rosie/maps"))

PIPELINE_TIMEOUT    = 7200   # 2-hour hard limit on a clean cycle
SNAPSHOT_INTERVAL   = 0      # 0 = disabled. Live snapshots cost ~15 s of
                             # full-CPU clean_map work each time on a Pi
                             # Zero 2 W (single core), starving the SLAM
                             # scan-matcher and producing fragmented maps.
                             # Set to e.g. 60 to re-enable on faster hardware.
MAP_REFRESH_INTERVAL = 2     # overlay refresh cadence (seconds)
DIAG_INTERVAL        = 30    # diagnostics publish cadence (seconds)


# ---------------------------------------------------------------------------
# Helper: HA discovery device block (must match mqtt_bridge.py)
# ---------------------------------------------------------------------------
_DEFAULT_DEVICE = {
    "identifiers": ["rosie_neato_d6"],
    "name": "ROSie",
    "manufacturer": "Neato Robotics",
    "model": "BotVac (detecting...)",
}


class MapPipeline:
    """Map-creation pipeline and map-overlay supervisor."""

    def __init__(self, mqtt, map_dir: Path = DEFAULT_MAP_DIR):
        """
        Args:
            mqtt:    MQTTBridge instance (already constructed but not yet
                     connected — start() is called after connect()).
            map_dir: Directory for home.pgm / home.yaml / home_clean.png etc.
        """
        self._mqtt = mqtt
        self._map_dir = Path(map_dir)
        self._pfx = mqtt._prefix

        # Per-instance device dict (mirrors mqtt_bridge); updated when the
        # driver calls update_device_info() after reading GetVersion.
        self._device = dict(_DEFAULT_DEVICE)

        # ── Pipeline state ────────────────────────────────────────────────
        self._status = IDLE
        self._pipeline_lock = threading.Lock()
        self._slam_active = False          # True while BreezySLAM is running

        # ── Robot state (updated via on_state / on_odom) ──────────────────
        self._ui_state: str = ""
        self._ext_power: bool = False
        self._robot_pose: Optional[tuple] = None   # (x, y, theta) world frame
        self._odom_pose: Optional[tuple] = None    # raw odom fallback
        self._odom_stamp: float = 0.0
        self._pose_stamp: float = 0.0

        # ── No-go lines ───────────────────────────────────────────────────
        self._nogo_lines: list = []        # list of {"p1":[x,y],"p2":[x,y]}

        # ── Dock position ─────────────────────────────────────────────────
        self._dock_pose: Optional[tuple] = None    # (x, y) world frame
        self._last_dock_save: float = 0.0

        # ── Map overlay ───────────────────────────────────────────────────
        self._base_img: Optional[Image.Image] = None   # clean_map.py output
        self._map_meta: Optional[dict] = None
        self._last_overlay_key = None
        self._last_map_refresh: float = 0.0

        # ── Diagnostics ───────────────────────────────────────────────────
        self._last_diag: float = 0.0
        # ── Update tracking ──────────────────────────────────────────────────
        self._update_log = Path.home() / "last-update.txt"
        self._update_log_mtime: float = 0.0
        # ── Background thread ─────────────────────────────────────────────
        self._stop_event = threading.Event()
        self._bg_thread: Optional[threading.Thread] = None

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def start(self) -> None:
        """Call once after MQTT is connected.  Loads existing map, publishes
        HA discovery, and starts the background tick thread."""
        self._map_dir.mkdir(parents=True, exist_ok=True)
        self._load_dock_pose()
        self._publish_ha_discovery()
        self._publish_status(IDLE)

        # Recover from an interrupted mapping run (OOM kill, power loss).
        # If a previous run was killed after SLAM had saved home.pgm but
        # before clean_map produced home_clean.png, finish that step now
        # so the user gets the map they just made instead of a placeholder.
        self._maybe_recover_interrupted_pipeline()

        self._load_map_overlay_data()
        self._publish_map_meta()
        self._publish_map_with_markers()
        self._publish_version()
        self._publish_last_update()

        self._bg_thread = threading.Thread(
            target=self._bg_loop, name="map-pipeline-bg", daemon=True,
        )
        self._bg_thread.start()
        logger.info("MapPipeline started (map_dir=%s)", self._map_dir)

    def _maybe_recover_interrupted_pipeline(self) -> None:
        """If a previous pipeline run was killed mid-flight, try to finish."""
        state = self._load_pipeline_state()
        if not state:
            return
        prev = state.get("status")
        age = time.time() - float(state.get("timestamp", 0))
        logger.warning(
            "Previous pipeline state was '%s' (%.0f s ago) — attempting recovery",
            prev, age,
        )

        pgm = self._map_dir / "home.pgm"
        yaml_path = self._map_dir / "home.yaml"
        clean_png = self._map_dir / "home_clean.png"
        meta_json = self._map_dir / "home_meta.json"

        # Clear stale state file up-front so we don't loop on persistent failure
        try:
            self._pipeline_state_path().unlink()
        except OSError:
            pass

        if pgm.exists() and yaml_path.exists() and not (clean_png.exists() and meta_json.exists()):
            logger.info("Found %s but no clean output — running clean_map to finalize", pgm)
            try:
                self._run_clean_map(yaml_path)
                logger.info("Recovery: clean_map succeeded")
            except Exception as exc:
                logger.error("Recovery: clean_map failed: %s", exc)
                self._publish_status(
                    ERROR,
                    f"Recovery from interrupted '{prev}' failed: {exc}",
                )
                return

        self._publish_status(IDLE, f"Recovered from interrupted '{prev}'")

    def stop(self) -> None:
        """Signal the background thread to exit and wait for it."""
        self._stop_event.set()
        if self._bg_thread and self._bg_thread.is_alive():
            self._bg_thread.join(timeout=5)
        if _slam_mod.is_running():
            _slam_mod.stop()

    # =========================================================================
    # Callbacks (called from main.py serial loop)
    # =========================================================================

    def on_scan(self, scan, odom) -> None:
        """Feed a LidarScan + OdomState into BreezySLAM.
        Only active while self._slam_active is True.

        After each scan we pull the SLAM-corrected pose (lidar scan-matching
        + odometry fusion) and use it as the authoritative robot pose for
        the map overlay.  This is dramatically more accurate than raw wheel
        odometry, which drifts heavily on carpet and after turns.
        """
        if not self._slam_active:
            return
        _slam_mod.update(scan, odom)

        # Stationary scan averaging — when the robot is essentially stopped,
        # re-imprint the same scan twice more with no pose change.  This is
        # cheap pseudo loop-closure: walls solidify and noisy single hits
        # get drowned out, with zero drift cost since the pose isn't moving.
        # The robot pauses several times during docking/undocking so this
        # runs naturally during a normal cleaning cycle.
        try:
            lin = abs(getattr(odom, "linear_vel", 0.0)) if odom else 0.0
            ang = abs(getattr(odom, "angular_vel", 0.0)) if odom else 0.0
            if lin < 0.02 and ang < 0.05:
                _slam_mod.update(scan, None)
                _slam_mod.update(scan, None)
        except Exception:
            logger.debug("stationary averaging failed", exc_info=True)

        try:
            x, y, theta = _slam_mod.get_position()
            self._robot_pose = (x, y, theta)
            self._pose_stamp = time.monotonic()
        except Exception:
            logger.debug("get_position failed", exc_info=True)

    def on_state(self, ui_state: Optional[str] = None,
                 ext_power: Optional[bool] = None) -> None:
        """Update robot UI state and/or ext_power flag.  Either may be None
        to leave the existing value unchanged."""
        if ui_state is not None:
            self._ui_state = ui_state
        if ext_power is not None:
            self._ext_power = ext_power

    def on_odom(self, odom) -> None:
        """Update robot pose from odometry — used only as a fallback when
        SLAM is not running (idle / pre-mapping) or its pose is stale."""
        self._odom_pose = (odom.x, odom.y, odom.theta)
        self._odom_stamp = time.monotonic()
        # SLAM pose takes precedence whenever it's recent (<15 s old).
        if time.monotonic() - self._pose_stamp > 15:
            self._robot_pose = self._odom_pose

    def on_nogo_lines(self, lines: list) -> None:
        """Update stored no-go lines and refresh map overlay."""
        self._nogo_lines = list(lines)
        self._last_overlay_key = None   # force refresh
        self._publish_map_meta()

    def on_command(self, cmd: str) -> None:
        """Handle pipeline-level commands from MQTT."""
        if cmd == "create_map":
            threading.Thread(
                target=self._run_pipeline_guarded,
                name="map-pipeline",
                daemon=True,
            ).start()
        elif cmd == "reboot":
            logger.warning("Reboot requested — rebooting in 3 s")
            subprocess.Popen(["sudo", "reboot"])

    # =========================================================================
    # Background tick loop
    # =========================================================================

    def _bg_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._maybe_refresh_map()
                self._publish_diagnostics()
                self._check_update_log()
            except Exception:
                logger.debug("bg_loop exception", exc_info=True)
            self._stop_event.wait(timeout=1.0)

    # =========================================================================
    # Map overlay refresh
    # =========================================================================

    def _maybe_refresh_map(self) -> None:
        """Republish map with updated markers if anything changed."""
        if self._status not in (IDLE, ERROR):
            return   # pipeline manages its own publishes

        now = time.monotonic()
        if now - self._last_map_refresh < MAP_REFRESH_INTERVAL:
            return
        self._last_map_refresh = now

        if self._base_img is None:
            return

        docked = self._is_docked()
        key = (
            self._robot_pose,
            self._dock_pose,
            docked,
            tuple(json.dumps(l, sort_keys=True) for l in self._nogo_lines),
        )
        if key == self._last_overlay_key:
            return
        self._last_overlay_key = key
        self._publish_map_with_markers()

    # =========================================================================
    # Diagnostics
    # =========================================================================

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        if now - self._last_diag < DIAG_INTERVAL:
            return
        self._last_diag = now
        try:
            import psutil
            cpu = psutil.cpu_percent(interval=None)
            ram = psutil.virtual_memory()
            payload = {
                "cpu_percent": round(cpu, 1),
                "ram_used_mb": round(ram.used / 1024 / 1024),
                "ram_percent": round(ram.percent, 1),
                "ram_total_mb": round(ram.total / 1024 / 1024),
            }
            self._mqtt.publish("diagnostics", payload, retain=True)
        except ImportError:
            pass   # psutil optional

    def _publish_version(self) -> None:
        """Read git version from the local repo and publish to MQTT."""
        try:
            repo = Path.home() / "rosie"
            short_sha = subprocess.check_output(
                ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL, text=True,
            ).strip()
            commit_date = subprocess.check_output(
                ["git", "-C", str(repo), "log", "-1", "--format=%ci"],
                stderr=subprocess.DEVNULL, text=True,
            ).strip()[:10]  # YYYY-MM-DD only
            self._mqtt.publish(
                "version",
                {"short_sha": short_sha, "commit_date": commit_date},
                retain=True,
            )
        except Exception:
            logger.debug("Could not read git version", exc_info=True)

    def _publish_last_update(self) -> None:
        """Read ~/last-update.txt and publish its content to MQTT."""
        try:
            if self._update_log.exists():
                content = self._update_log.read_text().strip()
                self._update_log_mtime = self._update_log.stat().st_mtime
            else:
                content = "never"
                self._update_log_mtime = 0.0
            self._mqtt.publish("last_update", content, retain=True)
        except Exception:
            logger.debug("Could not read last-update.txt", exc_info=True)

    def _check_update_log(self) -> None:
        """Re-publish version + last-update info if last-update.txt changed."""
        try:
            mtime = self._update_log.stat().st_mtime if self._update_log.exists() else 0.0
        except OSError:
            mtime = 0.0
        if mtime != self._update_log_mtime:
            self._publish_last_update()
            self._publish_version()

    # =========================================================================
    # Pipeline state machine
    # =========================================================================

    def _run_pipeline_guarded(self) -> None:
        with self._pipeline_lock:
            if self._status not in (IDLE, ERROR):
                logger.warning("Pipeline already running — ignoring create_map")
                return
            try:
                self._pipeline_impl()
            except Exception as exc:
                logger.exception("Pipeline failed")
                self._publish_status(ERROR, str(exc))
                if _slam_mod.is_running():
                    _slam_mod.stop()
                self._slam_active = False

    def _pipeline_impl(self) -> None:
        # ── 1. Clear old map ──────────────────────────────────────────────
        self._publish_status(CLEARING)
        self._clear_old_map()

        # ── 2. Start BreezySLAM ───────────────────────────────────────────
        self._publish_status(MAPPING)
        _slam_mod.start()
        self._slam_active = True
        # SLAM resets the world frame so robot (= on dock) is at (0, 0).
        # Persist this immediately so overlay rendering during this run
        # has a valid dock pose.  A clean re-dock at the end keeps it.
        self._save_dock_pose(0.0, 0.0)

        # ── 3. Send 'start' to robot ──────────────────────────────────────
        self._ui_state = ""              # clear stale retained value
        self._mqtt.publish("command", "start", qos=1)

        # Wait for robot to undock (up to 2 min)
        logger.info("Waiting for robot to undock…")
        undock_deadline = time.time() + 120
        while time.time() < undock_deadline:
            if self._stop_event.is_set():
                raise RuntimeError("Shutdown during pipeline")
            if not self._is_docked():
                logger.info("Robot has left the dock")
                break
            time.sleep(2)
        else:
            logger.warning("Robot did not undock within 120 s — continuing anyway")

        # ── 4. Wait for cleaning to finish and robot to return ────────────
        self._publish_status(WAITING_FOR_DOCK)
        start_time = time.time()
        was_cleaning = False
        last_snapshot: float = 0.0
        min_dock_time = time.time() + 60   # ignore early dock readings

        while time.time() - start_time < PIPELINE_TIMEOUT:
            if self._stop_event.is_set():
                raise RuntimeError("Shutdown during pipeline")

            if "CLEANING" in self._ui_state.upper():
                was_cleaning = True

            if was_cleaning and self._is_docked() and time.time() > min_dock_time:
                logger.info("Robot returned to dock (elapsed %.0f s)",
                            time.time() - start_time)
                time.sleep(10)   # let final scans arrive
                break

            # Live snapshots every SNAPSHOT_INTERVAL s while cleaning
            # (disabled when SNAPSHOT_INTERVAL == 0 to free CPU for SLAM)
            if was_cleaning and SNAPSHOT_INTERVAL > 0:
                now = time.time()
                if now - last_snapshot >= SNAPSHOT_INTERVAL:
                    last_snapshot = now
                    self._publish_live_snapshot()

            time.sleep(2)
        else:
            raise RuntimeError("Cleaning timeout (2 h)")

        # ── 5. Save map ───────────────────────────────────────────────────
        self._publish_status(SAVING)
        yaml_path, _pgm_path = _slam_mod.save_map(self._map_dir)
        self._slam_active = False
        _slam_mod.stop()

        # ── 6. Process map (clean_map.py) ─────────────────────────────────
        self._publish_status(PROCESSING)
        self._run_clean_map(yaml_path)

        # ── 7. Publish to HA ──────────────────────────────────────────────
        self._publish_status(PUBLISHING)
        self._load_map_overlay_data()
        self._publish_map_meta()
        self._publish_map_with_markers()

        self._publish_status(IDLE, "Map updated successfully")
        logger.info("Map pipeline complete")

    # =========================================================================
    # Pipeline helpers
    # =========================================================================

    def _clear_old_map(self) -> None:
        """Delete stale files and publish a blank 'Mapping…' placeholder."""
        self._nogo_lines = []
        if self._map_meta is not None:
            self._publish_map_meta()

        for fname in ("home_clean.png", "home_meta.json"):
            p = self._map_dir / fname
            if p.exists():
                p.unlink()
                logger.info("Deleted %s", p)

        self._base_img = None
        self._map_meta = None
        self._last_overlay_key = None

        blank = Image.new("RGB", (200, 200), BG_COLOUR)
        draw = ImageDraw.Draw(blank)
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
        except (OSError, IOError):
            font = ImageFont.load_default()
        draw.text((30, 85), "Mapping...", fill=(120, 130, 140), font=font)
        self._publish_pil_image(blank)
        logger.info("Old map cleared — placeholder published")

    def _is_docked(self) -> bool:
        if self._ext_power:
            return True
        ui = self._ui_state.upper()
        return any(kw in ui for kw in ("DOCKED", "CHARGING", "BASE"))

    def _publish_live_snapshot(self) -> None:
        """Save + clean + publish the in-progress SLAM map."""
        try:
            yaml_path, _ = _slam_mod.save_map(self._map_dir)
            self._run_clean_map(yaml_path)
            self._load_map_overlay_data()
            self._publish_map_with_markers()
            logger.info("Live map snapshot published")
        except Exception:
            logger.debug("Live snapshot failed", exc_info=True)

    def _run_clean_map(self, yaml_path: Path) -> None:
        """Run clean_map.py in a subprocess.

        Running it inline previously caused OOM kills on the Pi Zero 2 W
        because peak RSS (numpy + PIL filters + scipy.ndimage on the
        upscaled image) is briefly large.  A subprocess releases all of
        that memory the moment it exits, even if the OS doesn't return
        freed pages to the parent.
        """
        out_png  = self._map_dir / "home_clean.png"
        meta_out = self._map_dir / "home_meta.json"
        cmd = [
            sys.executable, "-m", "rosie_driver.clean_map",
            "--yaml", str(yaml_path),
            "--out",  str(out_png),
            "--meta", str(meta_out),
        ]
        try:
            res = subprocess.run(
                cmd,
                capture_output=True, text=True,
                timeout=180,
                check=False,
            )
        except subprocess.TimeoutExpired:
            logger.error("clean_map subprocess timed out after 180 s")
            raise
        if res.returncode != 0:
            logger.error(
                "clean_map subprocess failed (rc=%s)\nstdout: %s\nstderr: %s",
                res.returncode, res.stdout.strip(), res.stderr.strip(),
            )
            raise RuntimeError(
                f"clean_map exited {res.returncode}: {res.stderr.strip()[:200]}"
            )
        if res.stdout.strip():
            logger.info("clean_map: %s", res.stdout.strip().splitlines()[-1])

    # =========================================================================
    # Dock pose
    # =========================================================================

    def _dock_json_path(self) -> Path:
        return self._map_dir / "dock.json"

    def _load_dock_pose(self) -> None:
        p = self._dock_json_path()
        if p.exists():
            try:
                data = json.loads(p.read_text())
                self._dock_pose = (float(data["x"]), float(data["y"]))
                logger.info("Loaded dock pose: (%.3f, %.3f)", *self._dock_pose)
            except Exception as exc:
                logger.warning("Failed to load dock pose: %s", exc)

    def _save_dock_pose(self, x: float, y: float) -> None:
        self._dock_pose = (x, y)
        self._last_dock_save = time.monotonic()
        p = self._dock_json_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({"x": x, "y": y}))
        logger.info("Saved dock pose: (%.3f, %.3f)", x, y)

    # =========================================================================
    # Map overlay
    # =========================================================================

    def _load_map_overlay_data(self) -> None:
        out_png  = self._map_dir / "home_clean.png"
        meta_out = self._map_dir / "home_meta.json"
        if out_png.exists() and meta_out.exists():
            try:
                self._base_img = Image.open(out_png).convert("RGB")
                self._map_meta = json.loads(meta_out.read_text())
                logger.info("Loaded base map (%d×%d) + meta",
                            self._base_img.width, self._base_img.height)
            except Exception as exc:
                logger.warning("Failed to load map overlay data: %s", exc)
                self._base_img = None
                self._map_meta = None

    def _publish_map_meta(self) -> None:
        if self._map_meta is None:
            return
        payload = dict(self._map_meta)
        payload["nogo_lines"] = self._nogo_lines
        self._mqtt.publish("map_meta", payload, qos=1, retain=True)

    def _map_to_pixel(self, mx: float, my: float, meta: dict):
        """Convert world (m) → pixel coords in the clean_map.py output image."""
        ox, oy = meta["origin"]
        resolution = meta["resolution"]
        map_h = meta["map_h"]
        angle = meta["angle"]
        scale = meta["scale"]
        pre_rot_w  = meta["pre_rot_w"]
        pre_rot_h  = meta["pre_rot_h"]
        post_rot_w = meta["post_rot_w"]
        post_rot_h = meta["post_rot_h"]
        crop_r = meta["crop_r"]
        crop_c = meta["crop_c"]
        border = meta["border"]

        px = (mx - ox) / resolution
        py = (map_h - 1) - (my - oy) / resolution

        if abs(angle) > 0.1:
            rad = math.radians(angle)
            cos_a, sin_a = math.cos(rad), math.sin(rad)
            cx, cy = pre_rot_w / 2, pre_rot_h / 2
            dx, dy = px - cx, py - cy
            px = cos_a * dx - sin_a * dy + post_rot_w / 2
            py = sin_a * dx + cos_a * dy + post_rot_h / 2

        px = int(px * scale) - crop_c + border
        py = int(py * scale) - crop_r + border
        return px, py

    def _publish_map_with_markers(self) -> None:
        """Draw dock + robot markers on base map and publish."""
        if self._base_img is None or self._map_meta is None:
            # No processed map yet — fall back to raw file if it exists
            raw = self._map_dir / "home_clean.png"
            if raw.exists():
                data = raw.read_bytes()
                b64 = base64.b64encode(data).decode("ascii")
                self._mqtt.publish("map_image", b64, qos=1, retain=True)
            return

        try:
            meta  = self._map_meta
            img   = self._base_img.copy()
            draw  = ImageDraw.Draw(img)
            scale = meta["scale"]

            def _font(size, bold=False):
                try:
                    name = (
                        "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
                    )
                    return ImageFont.truetype(
                        f"/usr/share/fonts/truetype/dejavu/{name}",
                        max(10, size),
                    )
                except (OSError, IOError):
                    return ImageFont.load_default()

            icon_font  = _font(int(scale * 3 * 1.2))
            label_font = _font(max(10, scale * 2), bold=True)

            # ── Dock marker ───────────────────────────────────────────────
            if self._dock_pose:
                dx, dy = self._map_to_pixel(
                    self._dock_pose[0], self._dock_pose[1], meta)
                r = scale * 3
                if 0 <= dx < img.width and 0 <= dy < img.height:
                    draw.ellipse([dx - r, dy - r, dx + r, dy + r],
                                 fill=DOCK_COLOUR, outline=(20, 120, 60),
                                 width=max(2, scale // 2))
                    icon = "\u2302"
                    bb = draw.textbbox((0, 0), icon, font=icon_font)
                    tw, th = bb[2] - bb[0], bb[3] - bb[1]
                    draw.text((dx - tw // 2, dy - th // 2 - bb[1]),
                              icon, fill=(255, 255, 255), font=icon_font)
                    lbl = "Base"
                    lb = draw.textbbox((0, 0), lbl, font=label_font)
                    lw = lb[2] - lb[0]
                    draw.text((dx - lw // 2, dy + r + 2),
                              lbl, fill=DOCK_COLOUR, font=label_font)

            # ── Robot marker ──────────────────────────────────────────────
            draw_pose = self._robot_pose
            if self._is_docked() and self._dock_pose:
                draw_pose = (self._dock_pose[0], self._dock_pose[1], 0.0)
            if draw_pose:
                rx, ry = self._map_to_pixel(draw_pose[0], draw_pose[1], meta)
                r = scale * 2
                if 0 <= rx < img.width and 0 <= ry < img.height:
                    draw.ellipse([rx - r, ry - r, rx + r, ry + r],
                                 fill=ROBOT_COLOUR, outline=(20, 70, 200),
                                 width=max(2, scale // 2))
                    adj_theta = draw_pose[2] + math.radians(meta["angle"])
                    tri = r * 1.8
                    tip_x = rx + tri * math.cos(adj_theta)
                    tip_y = ry - tri * math.sin(adj_theta)
                    lx = rx + r * 0.6 * math.cos(adj_theta + 2.4)
                    ly = ry - r * 0.6 * math.sin(adj_theta + 2.4)
                    rx2 = rx + r * 0.6 * math.cos(adj_theta - 2.4)
                    ry2 = ry - r * 0.6 * math.sin(adj_theta - 2.4)
                    draw.polygon(
                        [(tip_x, tip_y), (lx, ly), (rx2, ry2)],
                        fill=(255, 255, 255),
                    )

            self._publish_pil_image(img)

        except Exception:
            logger.exception("Failed to publish map with markers")

    def _publish_pil_image(self, img: Image.Image) -> None:
        buf = io.BytesIO()
        img.save(buf, "PNG", optimize=True)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        self._mqtt.publish("map_image", b64, qos=1, retain=True)

    # =========================================================================
    # Status publishing
    # =========================================================================

    # Pipeline states that are worth persisting (i.e. mid-run, recoverable
    # if the process is killed or the system reboots).
    _PERSISTED_STATES = {MAPPING, WAITING_FOR_DOCK, SAVING, PROCESSING}

    def _pipeline_state_path(self) -> Path:
        return self._map_dir / "pipeline_state.json"

    def _save_pipeline_state(self, status: str) -> None:
        """Persist (or clear) the in-progress pipeline state to disk."""
        p = self._pipeline_state_path()
        try:
            if status in self._PERSISTED_STATES:
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text(json.dumps({
                    "status": status,
                    "timestamp": time.time(),
                }))
            elif p.exists():
                p.unlink()
        except OSError as exc:
            logger.debug("Could not write pipeline_state.json: %s", exc)

    def _load_pipeline_state(self) -> Optional[dict]:
        p = self._pipeline_state_path()
        if not p.exists():
            return None
        try:
            return json.loads(p.read_text())
        except (OSError, ValueError) as exc:
            logger.warning("Bad pipeline_state.json (%s) — ignoring", exc)
            try:
                p.unlink()
            except OSError:
                pass
            return None

    def _publish_status(self, status: str, detail: str = "") -> None:
        self._status = status
        self._save_pipeline_state(status)
        self._mqtt.publish(
            "map_pipeline/status",
            {"status": status, "detail": detail},
            qos=1, retain=True,
        )
        logger.info("Pipeline: %s %s", status, detail)

    # =========================================================================
    # HA MQTT Discovery
    # =========================================================================

    def _pub_discovery(self, component: str, object_id: str, config: dict) -> None:
        pfx = self._pfx
        config.setdefault("device", self._device)
        config.setdefault("availability", [{
            "topic": f"{pfx}/availability",
            "payload_available": "online",
            "payload_not_available": "offline",
        }])
        self._mqtt._client.publish(
            f"homeassistant/{component}/rosie_{object_id}/config",
            json.dumps(config), qos=1, retain=True,
        )

    def update_device_info(self, model: Optional[str] = None,
                           hw_version: Optional[str] = None) -> None:
        """Update the device block and re-publish pipeline HA discovery."""
        changed = False
        if model and model != self._device.get("model"):
            self._device["model"] = model
            changed = True
        if hw_version and hw_version != self._device.get("hw_version"):
            self._device["hw_version"] = hw_version
            changed = True
        if changed:
            try:
                self._publish_ha_discovery()
            except Exception as exc:  # noqa: BLE001
                logger.warning("Re-publishing pipeline HA discovery failed: %s", exc)

    def _publish_ha_discovery(self) -> None:
        pfx = self._pfx

        # Button: Create New Map
        self._pub_discovery("button", "create_map", {
            "name": "Create New Map",
            "unique_id": "rosie_create_map",
            "command_topic": f"{pfx}/command",
            "payload_press": "create_map",
            "icon": "mdi:map-plus",
        })

        # Button: Reboot Pi
        self._pub_discovery("button", "reboot", {
            "name": "Reboot Pi",
            "unique_id": "rosie_reboot",
            "command_topic": f"{pfx}/command",
            "payload_press": "reboot",
            "icon": "mdi:restart",
            "entity_category": "config",
        })

        # Sensor: Pipeline Status
        self._pub_discovery("sensor", "map_pipeline_status", {
            "name": "Map Pipeline",
            "unique_id": "rosie_map_pipeline_status",
            "state_topic": f"{pfx}/map_pipeline/status",
            "value_template": "{{ value_json.status }}",
            "json_attributes_topic": f"{pfx}/map_pipeline/status",
            "icon": "mdi:map-clock",
        })

        # Camera: Map Image
        self._pub_discovery("camera", "map", {
            "name": "ROSie Map",
            "unique_id": "rosie_map",
            "topic": f"{pfx}/map_image",
            "image_encoding": "b64",
            "icon": "mdi:floor-plan",
        })

        # Sensor: Map Metadata (for no-go editor coordinate transforms)
        self._pub_discovery("sensor", "map_meta", {
            "name": "Map Metadata",
            "unique_id": "rosie_map_meta",
            "state_topic": f"{pfx}/map_meta",
            "value_template": "{{ 'available' if value_json is defined else 'unavailable' }}",
            "json_attributes_topic": f"{pfx}/map_meta",
            "icon": "mdi:map-legend",
        })

        # Sensor: Pi CPU Load
        self._pub_discovery("sensor", "cpu_load", {
            "name": "Pi CPU Load",
            "unique_id": "rosie_cpu_load",
            "state_topic": f"{pfx}/diagnostics",
            "value_template": "{{ value_json.cpu_percent }}",
            "unit_of_measurement": "%",
            "state_class": "measurement",
            "icon": "mdi:cpu-64-bit",
        })

        # Sensor: Pi RAM
        self._pub_discovery("sensor", "ram", {
            "name": "Pi RAM",
            "unique_id": "rosie_ram",
            "state_topic": f"{pfx}/diagnostics",
            "value_template": "{{ value_json.ram_used_mb }} MB used ({{ value_json.ram_percent }}%)",
            "icon": "mdi:memory",
        })

        # Button: Update Software
        self._pub_discovery("button", "update_software", {
            "name": "Update Software",
            "unique_id": "rosie_update_software",
            "command_topic": f"{pfx}/command",
            "payload_press": "update",
            "icon": "mdi:update",
            "entity_category": "config",
        })

        # Sensor: Last Update
        self._pub_discovery("sensor", "last_update", {
            "name": "Last Update",
            "unique_id": "rosie_last_update",
            "state_topic": f"{pfx}/last_update",
            "icon": "mdi:package-down",
            "entity_category": "diagnostic",
        })

        # Sensor: Software Version
        self._pub_discovery("sensor", "version", {
            "name": "Software Version",
            "unique_id": "rosie_version",
            "state_topic": f"{pfx}/version",
            "value_template": "{{ value_json.short_sha }}",
            "json_attributes_topic": f"{pfx}/version",
            "icon": "mdi:source-branch",
            "entity_category": "diagnostic",
        })

        # Binary Sensor: Update Available (state published by check_updates.sh)
        self._pub_discovery("binary_sensor", "update_available", {
            "name": "Update Available",
            "unique_id": "rosie_update_available",
            "state_topic": f"{pfx}/update_available",
            "payload_on": "ON",
            "payload_off": "OFF",
            "device_class": "update",
            "icon": "mdi:update",
            "entity_category": "diagnostic",
        })

        logger.info("MapPipeline HA discovery published (11 entities)")
