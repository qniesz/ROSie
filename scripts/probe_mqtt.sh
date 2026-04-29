#!/bin/bash
set -a
source /home/rosie/rosie-driver.env
set +a
echo "uptime: $(uptime)"
echo "svc: $(systemctl is-active rosie)"
for t in rosie/map_image rosie/map_pipeline/status rosie/map_meta rosie/availability rosie/pose rosie/state; do
  s=$(mosquitto_sub -h "$MQTT_HOST" -u "$MQTT_USER" -P "$MQTT_PASS" -t "$t" -C 1 -W 3 2>/dev/null | wc -c)
  echo "$t  size=$s"
done
