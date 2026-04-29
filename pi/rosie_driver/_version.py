"""Single source of truth for the ROSie driver version.

Bump this string when releasing a new version, then commit + push + run
``pi/update.sh`` on the Pi.  The value is published to MQTT and exposed
in Home Assistant as ``sensor.rosie_software_version``.

Convention: simple "MAJOR.MINOR" string (e.g. ``"1.01"``, ``"1.02"``).
"""

VERSION = "1.01"
