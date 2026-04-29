#!/usr/bin/env bash
# check_updates.sh — ROSie daily update availability check
#
# Compares local git HEAD against origin/main.
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

LOCAL=$(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null)            || exit 0
REMOTE=$(git -C "$REPO_DIR" rev-parse origin/main 2>/dev/null)    || exit 0

[[ "$LOCAL" == "$REMOTE" ]] && PAYLOAD="OFF" || PAYLOAD="ON"

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
