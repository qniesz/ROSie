"""Quick test: render dock marker on the clean map to verify coordinates."""
import json, sys
from pathlib import Path
sys.path.insert(0, "/ros2_ws")
from PIL import Image, ImageDraw, ImageFont
from clean_map import map_to_pixel, DOCK_COLOUR

meta = json.loads(Path("/ros2_ws/maps/home_meta.json").read_text())
dock = json.loads(Path("/ros2_ws/maps/dock.json").read_text())
img = Image.open("/ros2_ws/maps/home_clean.png").convert("RGB")

conv_args = (
    meta["origin"], meta["resolution"], meta["map_h"],
    meta["angle"], meta["scale"],
    meta["pre_rot_w"], meta["pre_rot_h"],
    meta["post_rot_w"], meta["post_rot_h"],
    meta["crop_r"], meta["crop_c"], meta["border"],
)
dx, dy = map_to_pixel(dock["x"], dock["y"], *conv_args)
print(f"Dock pixel: ({dx}, {dy}) in {img.width}x{img.height} image")

draw = ImageDraw.Draw(img)
r = 24
draw.ellipse([dx-r, dy-r, dx+r, dy+r], fill=DOCK_COLOUR, outline=(20,120,60), width=4)
img.save("/ros2_ws/maps/test_overlay.png")
print("Saved test_overlay.png")
