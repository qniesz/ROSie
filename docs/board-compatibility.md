# Zero-Class Board Compatibility

ROSie targets a shared pure-Python runtime for both Raspberry Pi Zero 2 W and Orange Pi Zero 2 W. The driver code stays in `pi/rosie_driver`; board-specific facts live under `pi/boards`.

## Supported Profiles

| Profile | Board | OS Target | GPIO Backend | Status |
|---|---|---|---|---|
| `raspberrypi-zero2w` | Raspberry Pi Zero 2 W | Raspberry Pi OS / Debian arm64 | `rpi_gpio` | verified existing wiring |
| `orangepi-zero2w` | Orange Pi Zero 2 W | Armbian 64-bit Debian/Ubuntu | `gpiod` | requires chip/line verification |

The deployment script auto-detects the board from `/proc/device-tree/model`. You can override it in `scripts/ROSie.conf`:

```text
Pi_board_type:auto
```

Accepted values are `auto`, `raspberrypi`, and `orangepi`.

## Folder Layout

```text
pi/
  rosie_driver/
    gpio_backend.py
    gpio_pins.py
    bumper_sensors.py
  boards/
    raspberrypi-zero2w/
      gpio.json
      packages.txt
      env.example
    orangepi-zero2w/
      gpio.json
      packages.txt
      env.example
```

## Stable Wiring Contract

The physical header pins stay the same across boards:

| Bumper | Physical Pin |
|---|---:|
| Front Left | 29 |
| Front Right | 31 |
| Side Left | 33 |
| Side Right | 37 |

Raspberry Pi uses BCM numbers in its profile. Orange Pi uses libgpiod chip/line IDs, which must be verified on the actual board.

## Orange Pi Bring-Up

Run this on the Orange Pi after deploying Armbian:

```bash
gpioinfo
```

Update `pi/boards/orangepi-zero2w/gpio.json` or set env overrides in `/home/rosie/rosie-driver.env`:

```bash
ROSIE_GPIO_FRONT_LEFT=/dev/gpiochip0:<line>
ROSIE_GPIO_FRONT_RIGHT=/dev/gpiochip0:<line>
ROSIE_GPIO_SIDE_LEFT=/dev/gpiochip0:<line>
ROSIE_GPIO_SIDE_RIGHT=/dev/gpiochip0:<line>
```

Then run:

If the GPIO profile is incomplete or the required Python GPIO package is missing, ROSie logs a clear warning and the rest of the driver can still run. No-go bumper injection will be degraded until GPIO is configured.
