"""
Live robot position plot with SLAM map overlay and interactive line drawing.
Subscribes to MQTT odom topic and plots in real time.

Controls:
  - Right-click two points to draw a no-go line
  - Press 'c' to clear drawn lines
  - Press 'p' to print all line coordinates (copy into no_go_guard.py)

Requirements: pip install paho-mqtt matplotlib pillow
Optional: set ROSIE_MAP_YAML to point at a different nav2 map YAML.
"""
import ast
import json
import math
import os
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.image import imread
import paho.mqtt.client as mqtt

# --- Config ---
MQTT_HOST = os.environ["MQTT_HOST"]
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USER = os.environ["MQTT_USER"]
MQTT_PASS = os.environ["MQTT_PASS"]
ODOM_TOPIC = "rosie/odom"
POSE_TOPIC = "rosie/pose"

# Map config is loaded from nav2 map YAML.
DEFAULT_MAP_YAML = Path(__file__).resolve().parents[1] / "server" / "maps" / "home.yaml"
MAP_YAML = Path(os.environ.get("ROSIE_MAP_YAML", str(DEFAULT_MAP_YAML)))

# Existing no-go line (from no_go_guard.py)
EXISTING_LINES = [
    ((0.25, -1.06), (0.65, 0.45)),
]

# --- State ---
trail_x = []
trail_y = []
slam_trail_x = []
slam_trail_y = []
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
pose_source = "odom"  # "odom" or "slam"

# Interactive drawing state
draw_points = []    # temp clicks for current line
drawn_lines = []    # completed [(p1, p2), ...]


def load_map_from_yaml(map_yaml: Path):
    """Load map metadata (image, resolution, origin) from a nav2 map YAML."""
    if not map_yaml.exists():
        raise FileNotFoundError(f"map yaml not found: {map_yaml}")

    raw = {}
    for raw_line in map_yaml.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        raw[key.strip()] = value.strip()

    image_value = raw.get("image")
    if not image_value:
        raise ValueError(f"missing image in {map_yaml}")

    image_rel = image_value.strip("'\"")
    image_path = (map_yaml.parent / image_rel).resolve()
    if not image_path.exists():
        raise FileNotFoundError(f"map image not found: {image_path}")

    resolution = float(raw.get("resolution", "0.05"))
    origin = ast.literal_eval(raw.get("origin", "[0.0, 0.0, 0.0]"))
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError(f"invalid origin in {map_yaml}: {origin}")

    return image_path, resolution, (float(origin[0]), float(origin[1]), float(origin[2]))


def on_message(client, userdata, msg):
    global robot_x, robot_y, robot_theta, pose_source
    try:
        data = json.loads(msg.payload)
        x = data.get("x", 0.0)
        y = data.get("y", 0.0)
        theta = data.get("theta", 0.0)

        if msg.topic == POSE_TOPIC:
            # SLAM-corrected pose — always preferred
            robot_x = x
            robot_y = y
            robot_theta = theta
            slam_trail_x.append(x)
            slam_trail_y.append(y)
            pose_source = "slam"
        elif msg.topic == ODOM_TOPIC:
            trail_x.append(x)
            trail_y.append(y)
            if pose_source != "slam":
                robot_x = x
                robot_y = y
                robot_theta = theta
    except Exception:
        pass


# MQTT setup
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.username_pw_set(MQTT_USER, MQTT_PASS)
client.on_message = on_message
client.connect(MQTT_HOST, MQTT_PORT)
client.subscribe(ODOM_TOPIC)
client.subscribe(POSE_TOPIC)
client.loop_start()

# Load map from YAML so remaps stay aligned without code edits
MAP_IMAGE, MAP_RESOLUTION, MAP_ORIGIN = load_map_from_yaml(MAP_YAML)
print(f"Loaded map yaml: {MAP_YAML}")
print(f"Map image: {MAP_IMAGE}")
print(f"Resolution: {MAP_RESOLUTION}  Origin: {MAP_ORIGIN}")
map_img = imread(str(MAP_IMAGE))

# Compute map extent in metres
map_h, map_w = map_img.shape[:2]
map_x_min = MAP_ORIGIN[0]
map_x_max = MAP_ORIGIN[0] + map_w * MAP_RESOLUTION
map_y_min = MAP_ORIGIN[1]
map_y_max = MAP_ORIGIN[1] + map_h * MAP_RESOLUTION
map_extent = [map_x_min, map_x_max, map_y_min, map_y_max]

# Plot setup
fig, ax = plt.subplots(1, 1, figsize=(12, 10))
ax.set_title("ROSie Live Position — right-click to draw no-go lines, 'c' to clear, 'p' to print")
ax.set_xlabel("X (metres)")
ax.set_ylabel("Y (metres)")
ax.set_aspect("equal")
ax.grid(True, alpha=0.2)

# Draw map (flip vertically — PGM origin is top-left, map origin is bottom-left)
ax.imshow(map_img, extent=map_extent, origin='lower', cmap='gray', alpha=0.7)

# Draw existing no-go lines (extend just slightly beyond map)
for p1, p2 in EXISTING_LINES:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    extend = 2  # moderate extension
    lx = [p1[0] - extend * dx, p2[0] + extend * dx]
    ly = [p1[1] - extend * dy, p2[1] + extend * dy]
    ax.plot(lx, ly, 'r-', linewidth=3, alpha=0.7, label='Existing No-Go')
    ax.plot(*p1, 'rs', markersize=10)
    ax.plot(*p2, 'rs', markersize=10)

# Dock marker
ax.plot(0, 0, 'g^', markersize=14, label='Dock (0,0)', zorder=5)

# Trail and robot
trail_line, = ax.plot([], [], 'b-', alpha=0.2, linewidth=1, label='Odom Trail')
slam_trail_line, = ax.plot([], [], 'g-', alpha=0.5, linewidth=1.5, label='SLAM Trail')
robot_dot, = ax.plot([], [], 'bo', markersize=10, zorder=5)
robot_arrow = None


def on_click(event):
    """Right-click to place points. Two points = one line."""
    if event.button != 3 or event.inaxes != ax:  # right-click only
        return
    x, y = event.xdata, event.ydata
    draw_points.append((x, y))
    ax.plot(x, y, 'mo', markersize=8, zorder=6)

    if len(draw_points) == 2:
        p1, p2 = draw_points
        drawn_lines.append((p1, p2))
        # Draw as extended line within map
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        extend = 2
        lx = [p1[0] - extend * dx, p2[0] + extend * dx]
        ly = [p1[1] - extend * dy, p2[1] + extend * dy]
        ax.plot(lx, ly, 'm-', linewidth=3, label=f'New line {len(drawn_lines)}')
        print(f"  Line {len(drawn_lines)}: P1=({p1[0]:.3f}, {p1[1]:.3f})  P2=({p2[0]:.3f}, {p2[1]:.3f})")
        draw_points.clear()

    fig.canvas.draw_idle()


def on_key(event):
    """'c' = clear drawn lines, 'p' = print coordinates."""
    if event.key == 'c':
        drawn_lines.clear()
        draw_points.clear()
        print("Cleared all drawn lines. (Refresh plot to remove visuals.)")
    elif event.key == 'p':
        print("\n=== No-Go Line Coordinates (paste into no_go_guard.py) ===")
        for i, (p1, p2) in enumerate(drawn_lines, 1):
            print(f"  LINE_P1 = ({p1[0]:.3f}, {p1[1]:.3f})")
            print(f"  LINE_P2 = ({p2[0]:.3f}, {p2[1]:.3f})")
        if not drawn_lines:
            print("  (no lines drawn yet)")
        print()


fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('key_press_event', on_key)


def update(frame):
    global robot_arrow
    trail_line.set_data(trail_x, trail_y)
    slam_trail_line.set_data(slam_trail_x, slam_trail_y)
    robot_dot.set_data([robot_x], [robot_y])
    color = "green" if pose_source == "slam" else "blue"
    robot_dot.set_color(color)

    if robot_arrow:
        robot_arrow.remove()
    arrow_len = 0.1
    robot_arrow = ax.annotate("",
        xy=(robot_x + arrow_len * math.cos(robot_theta),
            robot_y + arrow_len * math.sin(robot_theta)),
        xytext=(robot_x, robot_y),
        arrowprops=dict(arrowstyle="->", color=color, lw=2))

    ax.set_title(f"ROSie Live Position [{pose_source.upper()}] — right-click lines, 'c' clear, 'p' print")
    return trail_line, slam_trail_line, robot_dot


# Zoom to map bounds with a small margin
margin = 0.3
ax.set_xlim(map_x_min - margin, map_x_max + margin)
ax.set_ylim(map_y_min - margin, map_y_max + margin)
ax.legend(loc='upper left')

ani = FuncAnimation(fig, update, interval=500, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()

client.loop_stop()
client.disconnect()
