#!/bin/bash
# tighten_pi_for_slam.sh — free RAM on Pi Zero 2 W for online slam_toolbox.
#
# RUN ONCE WITH SUDO. Reboot afterward to apply zswap.
#
#   sudo bash tighten_pi_for_slam.sh
#   sudo reboot
#
# All changes are idempotent and reversible (see UNDO instructions at end).

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Must run as root (sudo)." >&2
  exit 1
fi

echo "=== Tier 1: disable unused services ==="
SERVICES_TO_DISABLE=(
  bluetooth.service
  avahi-daemon.service
  avahi-daemon.socket
  unattended-upgrades.service
  serial-getty@ttyS0.service
  cloud-init.service
  cloud-init-local.service
  cloud-init-network.service
  cloud-init-main.service
  cloud-config.service
  cloud-final.service
)
for svc in "${SERVICES_TO_DISABLE[@]}"; do
  if systemctl list-unit-files "$svc" >/dev/null 2>&1; then
    systemctl disable --now "$svc" 2>/dev/null || true
    echo "  disabled $svc"
  fi
done

echo
echo "=== Tier 2a: enable zswap (zstd, 25% pool) ==="
CMDLINE=/boot/firmware/cmdline.txt
if [[ ! -f $CMDLINE ]]; then
  CMDLINE=/boot/cmdline.txt
fi
if [[ ! -f $CMDLINE ]]; then
  echo "  WARN: no cmdline.txt found, skipping zswap"
else
  if ! grep -q 'zswap.enabled=1' "$CMDLINE"; then
    cp "$CMDLINE" "${CMDLINE}.bak.$(date +%s)"
    sed -i 's|$| zswap.enabled=1 zswap.compressor=zstd zswap.max_pool_percent=25 zswap.zpool=z3fold|' "$CMDLINE"
    echo "  patched $CMDLINE (backup .bak.<ts> created)"
  else
    echo "  zswap already enabled"
  fi
  cat "$CMDLINE"
fi

echo
echo "=== Tier 2b: CycloneDDS shared memory off (system-wide) ==="
CYCLONE_XML=/etc/cyclonedds_no_shm.xml
cat > "$CYCLONE_XML" <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <SharedMemory>
      <Enable>false</Enable>
    </SharedMemory>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="default"/>
    </Internal>
  </Domain>
</CycloneDDS>
XML
echo "  wrote $CYCLONE_XML"

echo
echo "=== Tier 2c: drop page cache now ==="
sync
echo 3 > /proc/sys/vm/drop_caches
free -m

echo
echo "=== DONE ==="
echo "Reboot to activate zswap:   sudo reboot"
echo
echo "Verify after reboot:"
echo "  cat /sys/module/zswap/parameters/enabled         # should print Y"
echo "  cat /sys/module/zswap/parameters/compressor      # should print zstd"
echo "  systemctl is-active bluetooth.service avahi-daemon.service  # both inactive"
echo
echo "UNDO any service:   sudo systemctl enable --now <name>.service"
echo "UNDO zswap:         restore /boot/firmware/cmdline.txt.bak.<ts>, reboot"
