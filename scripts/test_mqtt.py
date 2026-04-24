import os
import paho.mqtt.client as mqtt
import threading
import sys

result_event = threading.Event()
connect_rc = [None]

def on_connect(client, userdata, flags, rc):
    RC_CODES = {0:"OK", 1:"Bad protocol", 2:"Client ID rejected",
                3:"Server unavailable", 4:"Bad credentials", 5:"Not authorized"}
    connect_rc[0] = rc
    print(f"CONNACK rc={rc} ({RC_CODES.get(rc, 'unknown')})")
    result_event.set()

def on_disconnect(client, userdata, rc):
    if not result_event.is_set():
        connect_rc[0] = rc
        print(f"Disconnected before CONNACK, rc={rc}")
        result_event.set()

c = mqtt.Client(client_id="rosie-pi-driver", protocol=mqtt.MQTTv311)
c.username_pw_set(os.environ["MQTT_USER"], os.environ["MQTT_PASS"])
c.on_connect = on_connect
c.on_disconnect = on_disconnect
c.connect(os.environ["MQTT_HOST"], int(os.environ.get("MQTT_PORT", "1883")), 10)
c.loop_start()
result_event.wait(timeout=5)
c.loop_stop()

if connect_rc[0] == 0:
    info = c.publish("rosie/test", "hello from pi", qos=1)
    info.wait_for_publish()
    c.disconnect()
    print("MQTT fully working!")
elif connect_rc[0] is None:
    print("TIMEOUT - no CONNACK received")
    sys.exit(1)
else:
    print("Connection FAILED")
    sys.exit(1)
