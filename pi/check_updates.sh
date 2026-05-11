#!/usr/bin/env bash
# check_updates.sh — ROSie daily update availability check
#
# Compares VERSION in local _version.py against origin/main:_version.py.
# Only reports an update when the stable version number changes — unreleased
# commits that don't bump VERSION are invisible to this check.
# Publishes ON/OFF to {MQTT_PREFIX}/update_available (retained) so HA can
# display binary_sensor.rosie_update_available without pulling any code.
#
# Called by rosie-check-updates.timer daily.

set -euo pipefail

REPO_DIR="$HOME/rosie"
ENV_FILE="$HOME/rosie-driver.env"

# shellcheck source=/dev/null
source "$ENV_FILE" 2>/dev/null || true

# Skip the check entirely in manual-updates mode
if [[ "${ROSIE_MANUAL_UPDATES:-false}" == "true" ]]; then
    exit 0
fi

# Fetch without pulling — silent on network failure (no update info is OK)
git -C "$REPO_DIR" fetch origin main 2>/dev/null || exit 0

LOCAL_VER=$(grep '^VERSION' "$REPO_DIR/pi/rosie_driver/_version.py" 2>/dev/null \
    | head -1 | cut -d'"' -f2) || exit 0
REMOTE_VER=$(git -C "$REPO_DIR" show origin/main:pi/rosie_driver/_version.py 2>/dev/null \
    | grep '^VERSION' | head -1 | cut -d'"' -f2) || exit 0

[[ "$LOCAL_VER" == "$REMOTE_VER" ]] && PAYLOAD="OFF" || PAYLOAD="ON"

# Publish if mosquitto_pub is available
if command -v mosquitto_pub >/dev/null 2>&1 && [[ -n "${MQTT_HOST:-}" ]]; then
    _MPUB_ARGS="-h ${MQTT_HOST} -p ${MQTT_PORT:-1883} -r"
    [[ -n "${MQTT_USER:-}" ]] && _MPUB_ARGS="$_MPUB_ARGS -u ${MQTT_USER}"
    [[ -n "${MQTT_PASS:-}" ]] && _MPUB_ARGS="$_MPUB_ARGS -P ${MQTT_PASS}"
    # shellcheck disable=SC2086
    mosquitto_pub $_MPUB_ARGS \
        -t "${MQTT_PREFIX:-rosie}/update_available" \
        -m "$PAYLOAD" 2>/dev/null || true
fi
