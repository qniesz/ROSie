"""
Clean up a SLAM-generated occupancy grid map for display in Home Assistant.

Reads the raw PGM map and produces a polished PNG with:
  - Rotation so walls are axis-aligned
  - Unknown (gray) areas filled with a clean background colour
  - Wall edges smoothed
  - Optional room colour tinting
  - Robot dock marker

Usage:
    python scripts/clean_map.py                         # uses defaults
    python scripts/clean_map.py --rotate -43             # manual rotation
    python scripts/clean_map.py --no-autorotate          # skip auto-rotation

Output: server/maps/home_clean.png  (also copies to ha/www/ for HA)
"""

import argparse
import ast
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

# ── Defaults ──────────────────────────────────────────────────────────
REPO = Path(__file__).resolve().parents[1]
DEFAULT_MAP_YAML = REPO / "server" / "maps" / "home.yaml"
DEFAULT_OUT = REPO / "server" / "maps" / "home_clean.png"
HA_WWW_OUT = REPO / "ha" / "www" / "rosie_map.png"

# Colours
BG_COLOUR = (240, 245, 250)        # light blue-gray background
FLOOR_COLOUR = (255, 255, 255)     # white floor
WALL_COLOUR = (50, 55, 65)         # dark charcoal walls
UNKNOWN_COLOUR = BG_COLOUR         # same as background
DOCK_COLOUR = (34, 170, 85)        # green dock marker

SCALE = 8                          # up-scale factor (raw pixels are 5cm)


def load_map_yaml(yaml_path: Path):
    """Parse a nav2 map YAML and return (pgm_path, resolution, origin)."""
    raw = {}
    for line in yaml_path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = line.split(":", 1)
            raw[k.strip()] = v.strip()
    pgm = (yaml_path.parent / raw["image"].strip("'\"")).resolve()
    resolution = float(raw.get("resolution", "0.05"))
    origin = ast.literal_eval(raw.get("origin", "[0,0,0]"))
    return pgm, resolution, (origin[0], origin[1])


def auto_rotate_angle(walls: np.ndarray, step: float = 0.5) -> float:
    """Find the rotation angle that best axis-aligns wall pixels."""
    ys, xs = np.nonzero(walls)
    if len(xs) < 10:
        return 0.0
    pts = np.column_stack((xs.astype(float), ys.astype(float)))
    best_angle = 0.0
    best_score = -1
    for deg in np.arange(-90, 90, step):
        rad = math.radians(deg)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        rotated = pts @ np.array([[cos_a, sin_a], [-sin_a, cos_a]])
        # score = sharpness of histogram in both axes (more aligned = sharper)
        hx = np.histogram(rotated[:, 0], bins=max(10, walls.shape[1] // 2))[0]
        hy = np.histogram(rotated[:, 1], bins=max(10, walls.shape[0] // 2))[0]
        score = float(np.sum(hx ** 2) + np.sum(hy ** 2))
        if score > best_score:
            best_score = score
            best_angle = deg
    return best_angle


def clean_map(
    yaml_path: Path = DEFAULT_MAP_YAML,
    out_path: Path = DEFAULT_OUT,
    rotate_deg: float | None = None,
    auto_rotate: bool = True,
    scale: int = SCALE,
):
    pgm_path, resolution, (ox, oy) = load_map_yaml(yaml_path)
    raw = np.array(Image.open(pgm_path).convert("L"))

    # Classify pixels: 0 = wall (occupied), 254/255 = free, middle = unknown
    walls = raw < 64
    free = raw > 190
    unknown = ~walls & ~free

    # Determine rotation angle
    if rotate_deg is not None:
        angle = rotate_deg
    elif auto_rotate:
        angle = auto_rotate_angle(walls)
    else:
        angle = 0.0
    print(f"Rotation: {angle:.1f}°")

    # Build colour image at native resolution
    h, w = raw.shape
    colour = np.zeros((h, w, 3), dtype=np.uint8)
    colour[free] = FLOOR_COLOUR
    colour[walls] = WALL_COLOUR
    colour[unknown] = UNKNOWN_COLOUR

    img = Image.fromarray(colour, "RGB")

    # Rotate
    if abs(angle) > 0.1:
        img = img.rotate(angle, resample=Image.BICUBIC, expand=True,
                         fillcolor=BG_COLOUR)

    # Up-scale
    new_size = (img.width * scale, img.height * scale)
    img = img.resize(new_size, Image.NEAREST)

    # Smooth wall edges (slight blur then re-threshold)
    img_arr = np.array(img).astype(float)
    blurred = np.array(img.filter(ImageFilter.GaussianBlur(radius=scale * 0.4)))

    # Re-threshold: dark pixels stay walls, light stay floor
    grey = np.mean(blurred, axis=2)
    wall_mask = grey < 100
    floor_mask = grey > 200

    result = np.full_like(img_arr, BG_COLOUR, dtype=np.uint8)
    result[floor_mask] = FLOOR_COLOUR
    result[wall_mask] = WALL_COLOUR

    img = Image.fromarray(result, "RGB")

    # Draw dock marker at origin (0,0) in map coordinates
    draw = ImageDraw.Draw(img)
    if abs(angle) > 0.1:
        # After rotation the image expanded; compute new pixel position
        rad = math.radians(angle)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        # Original pixel coords of origin
        px_orig = (0 - ox) / resolution
        py_orig = (h - 1) - (0 - oy) / resolution  # flip Y (image top-left origin)
        # Centre of original image
        cx, cy = w / 2, h / 2
        # Rotate around centre
        dx, dy = px_orig - cx, py_orig - cy
        rpx = cos_a * dx - sin_a * dy + img.width / (2 * scale)
        rpy = sin_a * dx + cos_a * dy + img.height / (2 * scale)
        dock_x = int(rpx * scale)
        dock_y = int(rpy * scale)
    else:
        dock_x = int((0 - ox) / resolution * scale)
        dock_y = int(((h - 1) - (0 - oy) / resolution) * scale)

    r = scale * 2
    draw.ellipse([dock_x - r, dock_y - r, dock_x + r, dock_y + r],
                 fill=DOCK_COLOUR, outline=(20, 120, 60), width=max(1, scale // 3))

    # Add thin border
    bordered = Image.new("RGB", (img.width + 4, img.height + 4), BG_COLOUR)
    bordered.paste(img, (2, 2))
    img = bordered

    # Save
    out_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_path, "PNG")
    print(f"Saved: {out_path}  ({img.width}x{img.height})")

    # Also copy to HA www folder
    HA_WWW_OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(HA_WWW_OUT, "PNG")
    print(f"Saved: {HA_WWW_OUT}")

    return img


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean a SLAM map for HA display")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--rotate", type=float, default=None,
                        help="Manual rotation in degrees")
    parser.add_argument("--no-autorotate", action="store_true")
    parser.add_argument("--scale", type=int, default=SCALE)
    args = parser.parse_args()
    clean_map(args.yaml, args.out, args.rotate, not args.no_autorotate, args.scale)
