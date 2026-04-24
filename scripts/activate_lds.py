import os
import paho.mqtt.client as m
c = m.Client()
c.username_pw_set(os.environ["MQTT_USER"], os.environ["MQTT_PASS"])
c.connect(os.environ["MQTT_HOST"])
c.publish("rosie/command", "activate", qos=1)
c.loop(1)
c.disconnect()
print("activate sent")
