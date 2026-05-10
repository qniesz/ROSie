# Orange Pi Zero 2 W Board Profile

Target OS: Armbian 64-bit Debian/Ubuntu.

ROSie keeps the same physical bumper/no-go header pins as the Raspberry Pi profile:

| Bumper | Physical Pin | libgpiod Mapping |
|---|---:|---|
| Front Left | 29 | verify with `gpioinfo` |
| Front Right | 31 | verify with `gpioinfo` |
| Side Left | 33 | verify with `gpioinfo` |
| Side Right | 37 | verify with `gpioinfo` |

The deployment script installs `gpiod`, `libgpiod-dev`, and `python3-libgpiod`, then selects the `gpiod` backend.

Before enabling no-go bumper injection on Orange Pi hardware, run:

```bash
gpioinfo
```

Then either update `gpio.json` with the verified chip/line offsets or set these env overrides in `/home/rosie/rosie-driver.env`:

```bash
ROSIE_GPIO_FRONT_LEFT=/dev/gpiochip0:<line>
ROSIE_GPIO_FRONT_RIGHT=/dev/gpiochip0:<line>
ROSIE_GPIO_SIDE_LEFT=/dev/gpiochip0:<line>
ROSIE_GPIO_SIDE_RIGHT=/dev/gpiochip0:<line>
```
