#!/bin/bash
set -a
. /home/rosie/rosie-driver.env
set +a
mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -u "$MQTT_USER" -P "$MQTT_PASS" \
  -t 'rosie/scan' -C 1 2>&1 | python3 -c '
import sys, json
d = json.loads(sys.stdin.read())
r = d["ranges"]
n0 = sum(1 for x in r if x == 0 or x == float("inf") or x is None)
valid = [x for x in r if x and x != float("inf")]
print(f"total={len(r)} zero/inf={n0} valid={len(valid)}")
print("rpm=", d.get("rpm"))
if valid:
    print(f"min={min(valid):.2f}m max={max(valid):.2f}m mean={sum(valid)/len(valid):.2f}m")
print("first 16:", [round(x,2) if x and x!=float("inf") else 0 for x in r[:16]])
'
