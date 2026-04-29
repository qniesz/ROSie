#!/usr/bin/env python3
"""Publish a synthetic 'live preview' frame to rosie/map_image.

Confirms the end-to-end MQTT → HA camera path works independent of
BreezySLAM and the bg_loop. If you see this image in the HA `ROSie
Map` camera entity within a few seconds, the topic & encoding are
fine and the only question is whether _publish_slam_preview() is
firing during real cycles.
"""
import os, io, base64, sys, time
import paho.mqtt.client as mqtt
from PIL import Image, ImageDraw, ImageFont

# Read driver env
env = {}
for line in open("/home/rosie/rosie-driver.env"):
    line = line.strip()
    if "=" in line and not line.startswith("#"):
        k, v = line.split("=", 1)
        env[k] = v

img = Image.new("RGB", (400, 400), (240, 240, 240))
draw = ImageDraw.Draw(img)
try:
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
except Exception:
    font = ImageFont.load_default()
draw.rectangle([20, 20, 380, 380], outline=(60, 60, 200), width=4)
draw.text((40, 60), "LIVE PREVIEW", fill=(220, 60, 60), font=font)
draw.text((40, 120), "TEST PATTERN", fill=(50, 50, 50), font=font)
draw.text((40, 180), time.strftime("%H:%M:%S"), fill=(50, 100, 50), font=font)
draw.text((40, 260), "If you see this in", fill=(80, 80, 80), font=font)
draw.text((40, 300), "HA, MQTT works.", fill=(80, 80, 80), font=font)

buf = io.BytesIO()
img.save(buf, "JPEG", quality=80)
b64 = base64.b64encode(buf.getvalue()).decode("ascii")

cli = mqtt.Client()
cli.username_pw_set(env["MQTT_USER"], env["MQTT_PASS"])
cli.connect(env["MQTT_HOST"], int(env.get("MQTT_PORT", "1883")), 30)
cli.loop_start()
info = cli.publish("rosie/map_image", b64, qos=1, retain=True)
info.wait_for_publish(timeout=10)
print(f"published rid={info.rc} bytes_b64={len(b64)} jpeg_bytes={len(buf.getvalue())}")
time.sleep(1)
cli.loop_stop()
cli.disconnect()
