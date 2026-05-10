#!/usr/bin/env python3
from __future__ import annotations

import os
import struct
import zlib

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def _paeth(left: int, up: int, upper_left: int) -> int:
    estimate = left + up - upper_left
    left_dist = abs(estimate - left)
    up_dist = abs(estimate - up)
    upper_left_dist = abs(estimate - upper_left)
    if left_dist <= up_dist and left_dist <= upper_left_dist:
        return left
    if up_dist <= upper_left_dist:
        return up
    return upper_left


def _read_png_rgba(path: str) -> tuple[int, int, bytes]:
    raw = open(path, "rb").read()
    if not raw.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"{path} is not a PNG")

    offset = 8
    width = height = bit_depth = color_type = None
    idat = bytearray()
    while offset < len(raw):
        chunk_len = struct.unpack(">I", raw[offset:offset + 4])[0]
        chunk_type = raw[offset + 4:offset + 8]
        chunk_data = raw[offset + 8:offset + 8 + chunk_len]
        offset += 12 + chunk_len
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, compression, png_filter, interlace = struct.unpack(
                ">IIBBBBB", chunk_data
            )
            if bit_depth != 8 or compression != 0 or png_filter != 0 or interlace != 0:
                raise ValueError("only 8-bit non-interlaced PNG images are supported")
            if color_type not in (0, 2, 6):
                raise ValueError(f"unsupported PNG color type {color_type}")
        elif chunk_type == b"IDAT":
            idat.extend(chunk_data)
        elif chunk_type == b"IEND":
            break

    if width is None or height is None or color_type is None:
        raise ValueError("PNG is missing IHDR")

    channels = {0: 1, 2: 3, 6: 4}[color_type]
    stride = width * channels
    inflated = zlib.decompress(bytes(idat))
    rows: list[bytearray] = []
    cursor = 0
    prior = bytearray(stride)
    for _ in range(height):
        filter_type = inflated[cursor]
        cursor += 1
        row = bytearray(inflated[cursor:cursor + stride])
        cursor += stride
        for index, value in enumerate(row):
            left = row[index - channels] if index >= channels else 0
            up = prior[index]
            upper_left = prior[index - channels] if index >= channels else 0
            if filter_type == 1:
                row[index] = (value + left) & 0xFF
            elif filter_type == 2:
                row[index] = (value + up) & 0xFF
            elif filter_type == 3:
                row[index] = (value + ((left + up) // 2)) & 0xFF
            elif filter_type == 4:
                row[index] = (value + _paeth(left, up, upper_left)) & 0xFF
            elif filter_type != 0:
                raise ValueError(f"unsupported PNG filter {filter_type}")
        rows.append(row)
        prior = row

    rgba = bytearray(width * height * 4)
    out = 0
    for row in rows:
        for index in range(0, len(row), channels):
            if color_type == 0:
                grey = row[index]
                rgba[out:out + 4] = bytes((grey, grey, grey, 255))
            elif color_type == 2:
                rgba[out:out + 4] = bytes((row[index], row[index + 1], row[index + 2], 255))
            else:
                rgba[out:out + 4] = bytes((row[index], row[index + 1], row[index + 2], row[index + 3]))
            out += 4
    return width, height, bytes(rgba)


class VacMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("rosie_vac_marker")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(Marker, "rosie_vac_marker", qos)
        self.texture_path = os.environ.get(
            "ROSIE_VAC_MARKER_TEXTURE",
            "/eval/robot/meshes/rosie_vac_top.png",
        )
        self.width = float(os.environ.get("ROSIE_VAC_MARKER_WIDTH", "0.340"))
        self.length = float(os.environ.get("ROSIE_VAC_MARKER_LENGTH", "0.321"))
        self.z = float(os.environ.get("ROSIE_VAC_MARKER_Z", "0.090"))
        self.pixel_count = int(os.environ.get("ROSIE_VAC_MARKER_PIXELS", "56"))
        self.flat_side_forward = os.environ.get("ROSIE_VAC_MARKER_FLAT_FRONT", "1") != "0"
        self.points, self.colors, self.tile_size = self._load_sprite()
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.publish_marker()
        self.get_logger().info(
            f"Publishing vac marker on /rosie_vac_marker from {self.texture_path} "
            f"({len(self.points)} colored tiles, tile={self.tile_size:.4f}m)"
        )

    def _load_sprite(self) -> tuple[list[Point], list[ColorRGBA], float]:
        try:
            image_width, image_height, pixels = _read_png_rgba(self.texture_path)
        except FileNotFoundError:
            self.get_logger().warning(
                f"Texture not found at {self.texture_path}; using fallback grey disc"
            )
            return self._fallback_disc()
        columns = max(8, self.pixel_count)
        rows = max(8, round(columns * image_height / image_width))
        tile_width = self.width / columns
        tile_height = self.length / rows
        points: list[Point] = []
        colors: list[ColorRGBA] = []

        for row in range(rows):
            source_y = min(image_height - 1, int((row + 0.5) * image_height / rows))
            y = self.length / 2.0 - (row + 0.5) * tile_height
            for column in range(columns):
                source_x = min(image_width - 1, int((column + 0.5) * image_width / columns))
                pixel_index = (source_y * image_width + source_x) * 4
                red, green, blue, alpha = pixels[pixel_index:pixel_index + 4]
                if alpha < 48:
                    continue
                x = -self.width / 2.0 + (column + 0.5) * tile_width
                if self.flat_side_forward:
                    points.append(Point(x=-y, y=x, z=0.0))
                else:
                    points.append(Point(x=x, y=y, z=0.0))
                colors.append(ColorRGBA(
                    r=red / 255.0,
                    g=green / 255.0,
                    b=blue / 255.0,
                    a=alpha / 255.0,
                ))

        return points, colors, min(tile_width, tile_height)

    def _fallback_disc(self) -> tuple[list[Point], list[ColorRGBA], float]:
        """Generate a grey D-shape (flat front, curved back) when no texture PNG is available."""
        columns = max(8, self.pixel_count)
        rows = max(8, round(columns * self.length / self.width))
        tile_width = self.width / columns
        tile_height = self.length / rows
        rx = self.width / 2.0
        ry = self.length / 2.0
        # Flat front: cut at the centre of the ellipse (y=0).
        # This gives a true semicircle D with the flat edge spanning the FULL
        # robot width (2*rx).  Any cut before y=0 clips the ellipse to less
        # than full width, making the "flat" edge look curved/pinched.
        flat_y = 0.0
        points: list[Point] = []
        colors: list[ColorRGBA] = []
        for row in range(rows):
            y = ry - (row + 0.5) * tile_height
            if y < flat_y:
                continue  # flat front — skip past the cut line
            for col in range(columns):
                x = -rx + (col + 0.5) * tile_width
                if (x / rx) ** 2 + (y / ry) ** 2 > 1.0:
                    continue
                if self.flat_side_forward:
                    points.append(Point(x=-y, y=x, z=0.0))
                else:
                    points.append(Point(x=x, y=y, z=0.0))
                colors.append(ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.9))
        return points, colors, min(tile_width, tile_height)

    def publish_marker(self) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "rosie_vac"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.position.z = self.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.tile_size * 1.08
        marker.scale.y = self.tile_size * 1.08
        marker.scale.z = 0.006
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 2
        marker.frame_locked = True
        marker.points = self.points
        marker.colors = self.colors

        self.publisher.publish(marker)


def main() -> None:
    rclpy.init()
    node = VacMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()