# ROSie

**Local-first smart vacuum control for the Neato Botvac D6**, built with ROS 2 Jazzy and Home Assistant.

ROSie replaces Neato's cloud dependency with a fully local stack running entirely on a Raspberry Pi 4:

```
Neato D6 (USB serial)
    |
    v
+----------------------------------------------+
|  Raspberry Pi 4                              |
|                                              |
|  +-----------------+  +--------------------+ |
|  |  rosie-driver   |  |    rosie-nav       | |
|  |  (Docker)       |  |    (Docker)        | |
|  |                 |  |                    | |
|  |  Python driver  |  |  ROS 2 Jazzy       | |
|  |  +- Serial      |  |  +- Nav2           | |
|  |  +- LIDAR       |  |  +- SLAM Toolbox   | |
|  |  +- Odometry    |  |  +- Foxglove       | |
|  |  +- Sensors     |  |  +- rosie_server   | |
|  |  +- MQTT bridge |  |     (supervisor)   | |
|  +--------+--------+  +--------+-----------+ |
+-----------|--------------------|--------------+
            |       MQTT         |
            v                    v
   +---------------------------------+
   |  Home Assistant (Mosquitto)     |
   |  +- MQTT vacuum entity          |
   |  +- ~49 auto-discovered entities|
   |  +- Live map camera feed        |
   |  +- Lovelace dashboard          |
   +---------------------------------+
```

## Project Structure

```
ROSie/
+-- pi/                          # Pi 4 Docker stack
|   +-- docker-compose.yaml      # Two services: rosie-driver + rosie-nav
|   +-- driver/Dockerfile        # Serial driver container
|   +-- nav/Dockerfile           # ROS 2 Nav/SLAM container
|   +-- rosie_driver/            # Python driver package
|   |   +-- main.py              # Main loop orchestrator
|   |   +-- serial_handler.py    # Neato serial protocol
|   |   +-- lidar.py             # LIDAR acquisition (~5 Hz)
|   |   +-- odometry.py          # Differential-drive odometry (~20 Hz)
|   |   +-- sensors.py           # Battery, state, settings
|   |   +-- commands.py          # MQTT command -> serial
|   |   +-- mqtt_bridge.py       # Paho MQTT publisher/subscriber
|   |   +-- no_go_guard.py       # GPIO bump injection for no-go zones
|   +-- requirements.txt
+-- server/                      # ROS 2 supervisor + map tools
|   +-- rosie_server.py          # Supervisor: process lifecycle + map pipeline + HA discovery
|   +-- clean_map.py             # Map post-processing (Pillow/numpy)
|   +-- ros2_ws/src/
|   |   +-- rosie_bringup/       # Launch files, Nav2/SLAM configs, URDF
|   |   +-- rosie_msgs/          # Custom messages: VacuumState, RobotSettings, SpotConfig
|   +-- maps/                    # Map files (pgm/yaml + processed PNGs)
+-- ha/                          # Home Assistant configuration
|   +-- dashboard/               # Lovelace dashboard card YAML
|   +-- templates/               # MQTT vacuum entity template
|   +-- www/
|       +-- rosie-nogo-editor-card.js  # Custom no-go zone editor card
+-- maps/                        # Runtime map data (dock position, overlays)
+-- scripts/                     # Diagnostic and utility scripts
```

## Quick Start

### Prerequisites

- Raspberry Pi 4 with Docker and Docker Compose installed
- Home Assistant with Mosquitto MQTT broker add-on
- Neato Botvac D6 connected via USB (`/dev/ttyACM0`)

### 1. Deploy to Pi

```bash
# Clone the repo on the Pi
git clone https://github.com/qniesz/ROSie.git ~/rosie
cd ~/rosie/pi

# Configure environment
cp ../.env.example .env
nano .env   # set MQTT_HOST, MQTT_USER, MQTT_PASS

# Build and start both containers
docker compose up -d --build
```

Both services start automatically on boot (`restart: unless-stopped`).

**Fast rebuild tips** (no full rebuild needed for most changes):

```bash
docker compose up -d --build rosie-driver   # ~2 min -- driver code changes
docker compose up -d --build rosie-nav      # ~4 min -- ROS 2 package changes
docker compose restart rosie-nav            # seconds -- rosie_server.py / clean_map.py only
```

### 2. Home Assistant

All entities are created automatically via MQTT Discovery on first connection -- no manual YAML configuration needed. You should see ~49 entities appear under the **ROSie** device.

Add the dashboard cards from `ha/dashboard/`:
1. In HA -> Dashboard -> Edit -> Add Card -> Manual YAML
2. Paste the contents of `rosie-card.yaml` (main control card)
3. Optionally add `rosie-map-card.yaml` for the live map view

**No-Go Zone Editor** (`ha/www/rosie-nogo-editor-card.js`):
1. Copy the JS file to your HA `www/` folder
2. Register it as a Lovelace resource at `/local/rosie-nogo-editor-card.js`
3. After each update, bump the `?v=N` query param on the resource -- HA caches JS aggressively

### 3. Create a Map

Trigger the automated map pipeline from HA:
1. Press the **Create New Map** button in the dashboard (or publish `create_map` to `rosie/command`)
2. The robot runs a full clean cycle while SLAM Toolbox builds the map
3. On return to dock, the map is saved, cleaned, and published as a camera feed in HA
4. Nav2 automatically restarts with the new map

### 4. Foxglove Visualization

Connect **Foxglove Studio** to `ws://<pi-ip>:8765` to visualize LIDAR scans, odometry, costmaps, and Nav2 paths in real time.

## MQTT Topics

| Topic | Direction | Description |
|---|---|---|
| `rosie/scan` | Pi -> Nav | LIDAR scan (JSON: ranges, intensities, geometry) |
| `rosie/odom` | Pi -> Nav | Odometry (JSON: x, y, theta, velocities) |
| `rosie/battery` | Pi -> HA | Battery state (JSON: fuel_percent, voltage, charging) |
| `rosie/bumpers` | Pi -> Nav | Bumper states (JSON: left/right side/front) |
| `rosie/state` | Pi -> HA | Robot UI state, errors, alerts |
| `rosie/settings` | Pi -> HA | User settings (EcoMode, WallEnable, etc.) |
| `rosie/spot_config` | Pi -> HA | Spot clean dimensions |
| `rosie/pose` | Nav -> HA | Robot pose from AMCL (JSON: x, y, theta) |
| `rosie/map` | Nav -> HA | Cleaned map image (base64 JPEG, MQTT camera) |
| `rosie/map_meta` | Nav -> HA | Map coordinate metadata for client transforms |
| `rosie/command` | HA -> Pi | Commands: start, stop, pause, return_to_base, locate, clean_spot, create_map |
| `rosie/cmd_vel` | Nav -> Pi | Velocity twist (JSON: linear_x, angular_z) |
| `rosie/settings/+/set` | HA -> Pi | Toggle individual settings (EcoMode, WallEnable, etc.) |
| `rosie/nogo_lines` | Pi -> HA | Active no-go lines: `{"lines":[{"p1":[x,y],"p2":[x,y]}]}` |
| `rosie/nogo_lines/set` | HA -> Pi | Replace no-go lines with JSON payload |
| `rosie/nogo_status` | Pi -> HA | Last no-go update result |
| `rosie/availability` | Pi -> HA | `online` / `offline` (LWT) |

### Update No-Go Lines

```bash
mosquitto_pub -h $MQTT_HOST -u $MQTT_USER -P $MQTT_PASS \
  -t rosie/nogo_lines/set \
  -m '{"lines":[{"p1":[0.25,-1.06],"p2":[0.65,0.45]},{"p1":[1.2,-0.2],"p2":[1.2,1.1]}]}'
```

No-go lines persist across restarts, are drawn as red overlays on the map in HA, and are enforced via GPIO bump injection during clean cycles.

### No-Go Pulse Behavior

No-go line responses now use short virtual bumper pulses instead of holding the bumper state. For angled approaches, side pulses are fired first, then the matching front bumper is used if contact persists.

- Pulse width default: `ROSIE_NOGO_PULSE_MS=80`
- Pulse interval default: `ROSIE_NOGO_PULSE_INTERVAL_MS=160`
- Escalation threshold default: `ROSIE_NOGO_SIDE_PULSES_BEFORE_FRONT=3`
- No-go virtual stop callback default: `ROSIE_NOGO_VIRTUAL_STOP_ENABLED=0`

With defaults, side contact gets 3 quick side pulses, then escalates to the matching front bumper only if still touching the line.

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | LIDAR from bridge |
| `/odom` | `nav_msgs/Odometry` | Odometry from bridge |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery from bridge |
| `/vacuum_state` | `rosie_msgs/VacuumState` | Robot state from bridge |
| `/robot_settings` | `rosie_msgs/RobotSettings` | Settings from bridge |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 -> bridge -> robot |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox output |

## Hardware

- **Robot**: Neato Botvac D6 Connected (gen3 serial protocol, firmware 4.5.3)
- **Compute**: Raspberry Pi 4 (USB serial `/dev/ttyACM0` at 115200 baud)
- **MQTT Broker**: Mosquitto on Home Assistant
- **Visualization**: Foxglove Studio (`ws://<pi-ip>:8765`)

## License

MIT