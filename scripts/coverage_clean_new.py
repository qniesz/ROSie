"""
coverage_clean.py — Native cleaning with no-go line enforcement.

Starts the Neato's built-in house cleaning, then monitors the robot's
position via TF. If the robot crosses a no-go line, it commands the
motors to reverse and turn away — like hitting a physical wall.
No pause/stop is sent; the motors are briefly overridden directly.

Usage (inside container):
  python3 /tmp/coverage_clean.py
"""
import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import paho.mqtt.client as mqtt
import time
import math
import json

# No-go line segment in map coordinates (from the Paint drawing)
# Direction chosen so robot's home side (0,0) has NEGATIVE signed distance
NOGO_LINE_P1 = (0.25, -1.06)
NOGO_LINE_P2 = (0.65, 0.45)

# Trigger distance — how far past the line before we bounce
NOGO_MARGIN = 0.10  # 10cm past the line

MQTT_HOST = os.environ["MQTT_HOST"]
MQTT_USER = os.environ["MQTT_USER"]
MQTT_PASS = os.environ["MQTT_PASS"]


def signed_distance_to_line(px, py, x1, y1, x2, y2):
    """Signed distance from point to line. Negative = allowed side."""
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx * dx + dy * dy)
    if length < 1e-6:
        return 0.0
    return (dy * px - dx * py + x2 * y1 - y2 * x1) / length


class CleaningMonitor(Node):
    def __init__(self):
        super().__init__('cleaning_monitor')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._mqtt = mqtt.Client()
        self._mqtt.username_pw_set(MQTT_USER, MQTT_PASS)
        self._mqtt.on_message = self._on_mqtt_msg
        self._mqtt.connect(MQTT_HOST)
        self._mqtt.subscribe("rosie/state")
        self._mqtt.loop_start()
        self._bounce_count = 0
        self._cleaning_active = False
        self._bouncing = False

    def _on_mqtt_msg(self, client, userdata, msg):
        if msg.topic == "rosie/state":
            try:
                state = json.loads(msg.payload.decode())
                activity = state.get("vacuum_activity", "")
                if activity in ("cleaning", "paused"):
                    self._cleaning_active = True
                elif activity in ("docked", "idle", "returning", "error"):
                    if self._cleaning_active:
                        self.get_logger().info(f'Cleaning ended — state: {activity}')
                    self._cleaning_active = False
            except Exception:
                pass

    def send_cmd_vel(self, linear_x, angular_z):
        """Send motor command directly via MQTT — overrides cleaning motors."""
        payload = json.dumps({"linear_x": linear_x, "angular_z": angular_z})
        self._mqtt.publish("rosie/cmd_vel", payload)

    def bounce_away(self):
        """Reverse and turn away from the no-go line — like hitting a wall."""
        self._bounce_count += 1
        self._bouncing = True
        self.get_logger().warn(f'WALL BOUNCE #{self._bounce_count}')

        # Reverse at 150mm/s and turn for 1 second
        end_time = time.time() + 1.0
        while time.time() < end_time:
            self.send_cmd_vel(-0.15, 1.5)  # reverse + turn left
            time.sleep(0.05)

        # Stop motors — cleaning firmware takes back over
        self.send_cmd_vel(0.0, 0.0)
        self._bouncing = False

    def get_robot_position(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link',
                                                  rclpy.time.Time(),
                                                  timeout=rclpy.duration.Duration(seconds=1.0))
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def is_violation(self, x, y):
        dist = signed_distance_to_line(
            x, y,
            NOGO_LINE_P1[0], NOGO_LINE_P1[1],
            NOGO_LINE_P2[0], NOGO_LINE_P2[1])
        return dist > NOGO_MARGIN, dist

    def run(self):
        self.get_logger().info('Starting native house cleaning...')
        self._mqtt.publish("rosie/command", "start")
        self._cleaning_active = True
        time.sleep(5)

        self.get_logger().info('Monitoring for no-go line violations...')
        idle_count = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self._bouncing:
                continue

            pos = self.get_robot_position()
            if pos is None:
                continue

            x, y = pos
            violation, dist = self.is_violation(x, y)

            if violation:
                self.get_logger().warn(f'NO-GO at ({x:.2f}, {y:.2f}) dist={dist:.2f}m')
                self.bounce_away()
            elif int(time.time()) % 15 == 0:
                self.get_logger().info(
                    f'OK: ({x:.2f}, {y:.2f}) dist={dist:.2f}m '
                    f'bounces={self._bounce_count}')

            if not self._cleaning_active:
                idle_count += 1
                if idle_count > 10:
                    self.get_logger().info(
                        f'Cleaning complete. Total bounces: {self._bounce_count}')
                    break
            else:
                idle_count = 0

            time.sleep(0.5)

        self._mqtt.loop_stop()
        self._mqtt.disconnect()


rclpy.init()
monitor = CleaningMonitor()
monitor.run()
monitor.destroy_node()
rclpy.shutdown()
