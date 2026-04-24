"""
coverage_clean.py — Native cleaning with no-go line enforcement.

Starts the Neato's built-in house cleaning, then monitors the robot's
position via TF. If the robot crosses a no-go line, it briefly pauses
and resumes — acting like a virtual wall. The Neato recalculates its
path and continues cleaning on the allowed side.

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
NOGO_MARGIN = 0.15  # 15cm past the line

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
        self._last_bounce_time = 0

    def _on_mqtt_msg(self, client, userdata, msg):
        """Monitor cleaning state from the driver."""
        if msg.topic == "rosie/state":
            try:
                state = json.loads(msg.payload.decode())
                activity = state.get("vacuum_activity", "")
                if activity in ("cleaning", "paused"):
                    self._cleaning_active = True
                elif activity in ("docked", "idle", "returning", "error"):
                    if self._cleaning_active:
                        self.get_logger().info(
                            f'Cleaning ended — state: {activity}')
                    self._cleaning_active = False
            except Exception:
                pass

    def send_mqtt_command(self, cmd):
        self._mqtt.publish("rosie/command", cmd)

    def start_cleaning(self):
        self.get_logger().info('Starting native house cleaning...')
        self.send_mqtt_command('start')
        self._cleaning_active = True

    def bounce(self):
        """Pause briefly then resume — virtual wall bounce."""
        now = time.time()
        if now - self._last_bounce_time < 3.0:
            return  # Don't spam bounces
        self._bounce_count += 1
        self._last_bounce_time = now
        self.get_logger().warn(
            f'BOUNCE #{self._bounce_count} — pausing and resuming')
        self.send_mqtt_command('pause')
        time.sleep(2)
        self.send_mqtt_command('resume')

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
        self.start_cleaning()
        time.sleep(5)  # Wait for cleaning to start and robot to leave dock

        self.get_logger().info('Monitoring for no-go line violations...')
        idle_count = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            pos = self.get_robot_position()
            if pos is None:
                continue

            x, y = pos
            violation, dist = self.is_violation(x, y)

            if violation:
                self.get_logger().warn(
                    f'NO-GO at ({x:.2f}, {y:.2f}) dist={dist:.2f}m')
                self.bounce()
            elif int(time.time()) % 15 == 0:
                self.get_logger().info(
                    f'OK: ({x:.2f}, {y:.2f}) dist={dist:.2f}m '
                    f'bounces={self._bounce_count}')

            # Check if cleaning finished naturally
            if not self._cleaning_active:
                idle_count += 1
                if idle_count > 10:  # ~5 seconds of no cleaning
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

