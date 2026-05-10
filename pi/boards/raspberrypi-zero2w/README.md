# Raspberry Pi Zero 2 W Board Profile

This profile keeps the existing ROSie bumper/no-go wiring:

| Bumper | Physical Pin | GPIO ID |
|---|---:|---|
| Front Left | 29 | BCM GPIO5 |
| Front Right | 31 | BCM GPIO6 |
| Side Left | 33 | BCM GPIO13 |
| Side Right | 37 | BCM GPIO26 |

The deployment script installs `python3-rpi.gpio` and selects the `rpi_gpio` backend.
