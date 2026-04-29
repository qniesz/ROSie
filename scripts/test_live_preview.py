#!/usr/bin/env python3
"""Quick test: can _publish_slam_preview() even produce a frame?

Imports the same modules map_pipeline uses and exercises the snapshot →
classify → render → JPEG path. Prints precise pass/fail reasons so we
know what's blocking the live preview during real cleaning cycles.
"""
import sys, traceback, time
sys.path.insert(0, "/home/rosie/rosie/pi")

print("=== imports ===")
try:
    import numpy as np
    print(f"  numpy {np.__version__}")
except Exception as e:
    print(f"  numpy FAIL: {e}")
    sys.exit(1)

try:
    from PIL import Image, ImageDraw, ImageFont
    print(f"  PIL OK")
except Exception as e:
    print(f"  PIL FAIL: {e}")
    sys.exit(1)

try:
    from scipy.ndimage import binary_dilation, binary_propagation
    print(f"  scipy OK")
except Exception as e:
    print(f"  scipy FAIL: {e}")

try:
    from rosie_driver import slam as _slam_mod
    print(f"  slam module OK   MAP_SIZE_PIXELS={_slam_mod.MAP_SIZE_PIXELS}")
except Exception as e:
    print(f"  slam import FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

print("=== start slam + grab snapshot ===")
try:
    _slam_mod.start()
    print("  slam.start OK")
except Exception as e:
    print(f"  slam.start FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

# Feed a couple of fake updates so the grid isn't pure unknown
try:
    grid = _slam_mod.get_snapshot_array()
    print(f"  get_snapshot_array OK   shape={None if grid is None else grid.shape} dtype={None if grid is None else grid.dtype}")
    if grid is not None:
        u, c = np.unique(grid, return_counts=True)
        print(f"  unique counts: {dict(zip(u.tolist(), c.tolist()))}")
except Exception as e:
    print(f"  get_snapshot_array FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

print("=== render ===")
try:
    rgb = np.empty((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
    rgb[:] = (210, 215, 222)
    rgb[grid > 190] = (255, 255, 255)
    rgb[grid < 64]  = (50, 55, 65)
    img = Image.fromarray(rgb, "RGB")
    img = img.resize((grid.shape[1]*2, grid.shape[0]*2), Image.NEAREST)
    print(f"  PIL image OK   size={img.size}")
except Exception as e:
    print(f"  render FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

print("=== font ===")
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
    print(f"  truetype OK")
except Exception as e:
    print(f"  truetype FAIL → fallback default: {e}")
    font = ImageFont.load_default()

print("=== save jpeg ===")
try:
    import io, base64
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=70, optimize=True)
    print(f"  jpeg bytes={len(buf.getvalue())}")
except Exception as e:
    print(f"  jpeg FAIL: {e}")
    traceback.print_exc()
    sys.exit(1)

print("=== ALL OK ===")
