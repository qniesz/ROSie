#!/bin/bash
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="plugdev"' > /etc/udev/rules.d/60-gpiochip.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=gpio
echo "=== gpiochip perms ==="
ls -la /dev/gpiochip*
