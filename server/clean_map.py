"""
Clean up a SLAM-generated occupancy grid map for display.

Container-friendly version — no repo-relative paths.
Called by rosie_server.py during the map pipeline.

Can also run standalone:
    python3 clean_map.py
    python3 clean_map.py --yaml /ros2_ws/maps/home.yaml --rotate 0
"""

import argparse
import ast
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont
from scipy import ndimage

# ── Defaults (container paths) ────────────────────────────────────
DEFAULT_MAP_YAML = Path("/ros2_ws/maps/home.yaml")
DEFAULT_OUT = Path("/ros2_ws/maps/home_clean.png")
DEFAULT_META_OUT = Path("/ros2_ws/maps/home_meta.json")

# Colours
BG_COLOUR = (240, 245, 250)        # light blue-gray background
FLOOR_COLOUR = (255, 255, 255)     # white floor
WALL_COLOUR = (50, 55, 65)         # dark charcoal walls
DOCK_COLOUR = (34, 170, 85)        # green dock marker
ROBOT_COLOUR = (41, 121, 255)      # blue robot marker

SCALE = 8                          # up-scale factor (raw 5cm pixels)
PADDING = 6                        # pixels of border around content
BORDER = 4                         # border around final image


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


def auto_rotate_angle(walls: np.ndarray) -> float:
    """Find the rotation angle that best axis-aligns wall edges.

    Uses gradient (Sobel) direction at wall boundaries to find the
    dominant orientation.  Much more robust than pixel-histogram methods
    because it responds to *edge direction* rather than pixel density.
    """
    from scipy.ndimage import sobel

    wall_f = walls.astype(np.float64)
    gx = sobel(wall_f, axis=1)  # horizontal gradient
    gy = sobel(wall_f, axis=0)  # vertical gradient

    # Only consider pixels with significant gradient (wall edges)
    mag = np.sqrt(gx ** 2 + gy ** 2)
    threshold = mag.max() * 0.3
    mask = mag > threshold
    if mask.sum() < 10:
        return 0.0

    gx_edge = gx[mask]
    gy_edge = gy[mask]

    # Gradient direction gives wall-normal angle.  We want to find the
    # dominant orientation modulo 90° (walls can be H or V).
    # Double-angle trick: map angle to 4*theta so 90° wraps to 360°.
    theta = np.arctan2(gy_edge, gx_edge)        # -pi..pi
    c = np.cos(4 * theta)
    s = np.sin(4 * theta)
    # Weighted by gradient magnitude for robustness
    w = mag[mask]
    mean_angle = math.atan2(np.sum(w * s), np.sum(w * c)) / 4.0
    deg = math.degrees(mean_angle)

    # Keep result in -45..+45 range
    while deg > 45:
        deg -= 90
    while deg < -45:
        deg += 90

    return round(deg, 1)


def _fill_interior_unknowns(walls, free, unknown):
    """Fill unknown pixels that are surrounded by floor/walls (interior gaps)."""
    labeled, _ = ndimage.label(unknown)
    edge_labels = set()
    edge_labels.update(labeled[0, :].tolist())
    edge_labels.update(labeled[-1, :].tolist())
    edge_labels.update(labeled[:, 0].tolist())
    edge_labels.update(labeled[:, -1].tolist())
    edge_labels.discard(0)
    interior = unknown.copy()
    for lbl in edge_labels:
        interior[labeled == lbl] = False
    return interior


def _crop_to_content(img_arr, bg_colour, padding=PADDING):
    """Crop image array to bounding box of non-background content + padding.
    Returns (cropped_array, row_min, col_min)."""
    bg = np.array(bg_colour, dtype=np.uint8)
    mask = ~np.all(img_arr == bg, axis=2)
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    if not rows.any():
        return img_arr, 0, 0
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    rmin = max(0, rmin - padding)
    rmax = min(img_arr.shape[0] - 1, rmax + padding)
    cmin = max(0, cmin - padding)
    cmax = min(img_arr.shape[1] - 1, cmax + padding)
    return img_arr[rmin:rmax + 1, cmin:cmax + 1], rmin, cmin


def map_to_pixel(mx, my, origin, resolution, map_h, angle, scale,
                 pre_rot_w, pre_rot_h, post_rot_w, post_rot_h,
                 crop_r, crop_c, border):
    """Convert map-frame coordinates (m) to pixel coordinates in the final image."""
    ox, oy = origin
    # Raw pixel in the original PGM (before rotation)
    px = (mx - ox) / resolution
    py = (map_h - 1) - (my - oy) / resolution

    # Apply rotation around the centre of the pre-rotation image
    if abs(angle) > 0.1:
        rad = math.radians(angle)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        cx, cy = pre_rot_w / 2, pre_rot_h / 2
        dx, dy = px - cx, py - cy
        px = cos_a * dx - sin_a * dy + post_rot_w / 2
        py = sin_a * dx + cos_a * dy + post_rot_h / 2

    # Scale
    px = int(px * scale)
    py = int(py * scale)

    # Adjust for crop offset + border
    px = px - crop_c + border
    py = py - crop_r + border
    return px, py


def clean_map(
    yaml_path: Path = DEFAULT_MAP_YAML,
    out_path: Path = DEFAULT_OUT,
    meta_path: Path = DEFAULT_META_OUT,
    rotate_deg: float | None = None,
    auto_rotate: bool = True,
    scale: int = SCALE,
):
    """Load a SLAM map, clean it up, and save a polished PNG + metadata JSON."""
    pgm_path, resolution, (ox, oy) = load_map_yaml(yaml_path)
    raw = np.array(Image.open(pgm_path).convert("L"))

    # Classify pixels
    walls = raw < 64
    free = raw > 190
    unknown = ~walls & ~free

    # Fill interior unknown patches as floor
    interior = _fill_interior_unknowns(walls, free, unknown)
    free = free | interior
    unknown = unknown & ~interior

    # Close small wall gaps (morphological closing)
    walls_closed = ndimage.binary_closing(walls, structure=np.ones((3, 3)),
                                          iterations=1)
    walls = walls_closed & ~free

    # Open to remove small bumps/protrusions
    walls = ndimage.binary_opening(walls, structure=np.ones((2, 2)),
                                   iterations=1)

    # Remove tiny wall fragments (noise)
    labeled_walls, n_wall = ndimage.label(walls)
    for i in range(1, n_wall + 1):
        if np.sum(labeled_walls == i) < 4:
            walls[labeled_walls == i] = False

    # Determine rotation
    h, w = raw.shape
    if rotate_deg is not None:
        angle = rotate_deg
    elif auto_rotate:
        angle = auto_rotate_angle(walls)
    else:
        angle = 0.0
    print(f"Rotation: {angle:.1f}°")

    # ── Build floor image and wall mask separately ──────────────
    # Floor/bg image (no walls)
    colour = np.full((h, w, 3), BG_COLOUR, dtype=np.uint8)
    colour[free] = FLOOR_COLOUR

    floor_img = Image.fromarray(colour, "RGB")
    pre_rot_w, pre_rot_h = floor_img.width, floor_img.height

    # Wall mask as grayscale (0=no wall, 255=wall)
    wall_img = Image.fromarray((walls.astype(np.uint8) * 255), "L")

    # Rotate both the same way
    if abs(angle) > 0.1:
        floor_img = floor_img.rotate(angle, resample=Image.BICUBIC,
                                     expand=True, fillcolor=BG_COLOUR)
        wall_img = wall_img.rotate(angle, resample=Image.BICUBIC,
                                   expand=True, fillcolor=0)
    post_rot_w, post_rot_h = floor_img.width, floor_img.height

    # Up-scale floor with NEAREST (sharp pixel edges)
    new_size = (floor_img.width * scale, floor_img.height * scale)
    floor_img = floor_img.resize(new_size, Image.NEAREST)

    # Up-scale wall mask with BILINEAR (smooth edges instead of blocky)
    wall_img = wall_img.resize(new_size, Image.BILINEAR)

    # Blur the wall mask to smooth further, then re-threshold
    blur_radius = scale * 0.7
    wall_arr = np.array(
        wall_img.filter(ImageFilter.GaussianBlur(radius=blur_radius)),
        dtype=np.float32,
    )
    wall_mask = wall_arr > 60

    # Morphological smoothing at high resolution — straightens edges
    # Use rectangular elements to favor axis-aligned walls
    horiz = np.ones((1, scale), dtype=bool)
    vert = np.ones((scale, 1), dtype=bool)
    wall_mask = ndimage.binary_closing(wall_mask, structure=horiz, iterations=1)
    wall_mask = ndimage.binary_closing(wall_mask, structure=vert, iterations=1)
    wall_mask = ndimage.binary_opening(wall_mask, structure=np.ones((3, 3)),
                                       iterations=1)

    # Composite: floor + walls
    result = np.array(floor_img)
    result[wall_mask] = WALL_COLOUR

    # Smooth floor/bg boundary with a light blur + rethreshold
    img = Image.fromarray(result, "RGB")
    light_blur = np.array(
        img.filter(ImageFilter.GaussianBlur(radius=scale * 0.3)))
    grey = np.mean(light_blur, axis=2)
    final = np.full_like(result, BG_COLOUR)
    final[grey > 210] = FLOOR_COLOUR
    final[wall_mask] = WALL_COLOUR  # keep walls crisp

    # Crop to content
    cropped, crop_r, crop_c = _crop_to_content(final, BG_COLOUR,
                                                padding=PADDING * scale)
    img = Image.fromarray(cropped, "RGB")

    # Add border
    bordered = Image.new("RGB",
                         (img.width + BORDER * 2, img.height + BORDER * 2),
                         BG_COLOUR)
    bordered.paste(img, (BORDER, BORDER))
    img = bordered

    # Save the base image (no markers)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_path, "PNG", optimize=True)
    print(f"Saved: {out_path}  ({img.width}x{img.height})")

    # Save metadata for marker overlay
    meta = {
        "origin": [ox, oy],
        "resolution": resolution,
        "map_h": h,
        "map_w": w,
        "angle": angle,
        "scale": scale,
        "pre_rot_w": pre_rot_w,
        "pre_rot_h": pre_rot_h,
        "post_rot_w": post_rot_w,
        "post_rot_h": post_rot_h,
        "crop_r": crop_r,
        "crop_c": crop_c,
        "border": BORDER,
        "image_w": img.width,
        "image_h": img.height,
    }
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"Meta: {meta_path}")

    return img


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean a SLAM map for display")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--meta", type=Path, default=DEFAULT_META_OUT)
    parser.add_argument("--rotate", type=float, default=None,
                        help="Manual rotation in degrees")
    parser.add_argument("--no-autorotate", action="store_true")
    parser.add_argument("--scale", type=int, default=SCALE)
    args = parser.parse_args()
    clean_map(args.yaml, args.out, args.meta, args.rotate,
              not args.no_autorotate, args.scale)
