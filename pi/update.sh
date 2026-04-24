#!/usr/bin/env bash
# update.sh — ROSie software update
#
# Usage: update.sh [--force]
#   --force  Skip the cleaning-in-progress guard (used by on-demand HA button).
#
# Writes result to ~/last-update.txt (single line) and appends to
# ~/update-history.log (ring-buffered at 10 lines).
#
# Requires:
#   - ~/rosie-driver.env with MQTT_HOST / MQTT_PORT / MQTT_USER / MQTT_PASS
#   - sudoers entry: rosie ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart rosie

set -euo pipefail

FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

REPO_DIR="$HOME/rosie"
VENV="$HOME/rosie-venv"
LOG_FILE="$HOME/last-update.txt"
HISTORY_FILE="$HOME/update-history.log"
ENV_FILE="$HOME/rosie-driver.env"
LOCK_FILE="/tmp/rosie-update.lock"

# ── Prevent concurrent runs ────────────────────────────────────────────────────
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "$(date -Iseconds) SKIPPED: another update already running" > "$LOG_FILE"
    exit 0
fi

# ── Source MQTT credentials ────────────────────────────────────────────────────
# shellcheck source=/dev/null
source "$ENV_FILE" 2>/dev/null || true

# ── Helpers ────────────────────────────────────────────────────────────────────
_log_result() {
    local msg="$1"
    echo "$msg" > "$LOG_FILE"
    { tail -9 "$HISTORY_FILE" 2>/dev/null; echo "$msg"; } > "${HISTORY_FILE}.tmp" \
        && mv "${HISTORY_FILE}.tmp" "$HISTORY_FILE"
}

_fail() {
    local reason="$1"
    _log_result "$(date -Iseconds) FAILED: $reason"
    echo "update.sh: FAILED: $reason" >&2
    exit 1
}

_rollback() {
    local reason="$1"
    local prev_sha="$2"
    echo "update.sh: rolling back to $prev_sha — $reason" >&2
    git -C "$REPO_DIR" reset --hard "$prev_sha" 2>/dev/null || true
    "$VENV/bin/pip" install -q -r "$REPO_DIR/pi/zero_requirements.txt" 2>/dev/null || true
    sudo systemctl restart rosie 2>/dev/null || true
    _fail "rollback: $reason"
}

# ── Stash current SHA ──────────────────────────────────────────────────────────
PREV_SHA=$(git -C "$REPO_DIR" rev-parse HEAD 2>/dev/null) \
    || _fail "git rev-parse failed — is the repository initialised?"

# ── Cleaning guard ─────────────────────────────────────────────────────────────
if [[ "$FORCE" -eq 0 ]]; then
    if command -v mosquitto_sub >/dev/null 2>&1 && [[ -n "${MQTT_HOST:-}" ]]; then
        _MSUB_ARGS="-h ${MQTT_HOST} -p ${MQTT_PORT:-1883}"
        [[ -n "${MQTT_USER:-}" ]] && _MSUB_ARGS="$_MSUB_ARGS -u ${MQTT_USER}"
        [[ -n "${MQTT_PASS:-}" ]] && _MSUB_ARGS="$_MSUB_ARGS -P ${MQTT_PASS}"
        # shellcheck disable=SC2086
        STATE=$(timeout 5 mosquitto_sub $_MSUB_ARGS -t "${MQTT_PREFIX:-rosie}/state" -C 1 2>/dev/null || true)
        if echo "$STATE" | grep -qi "cleaning"; then
            _log_result "$(date -Iseconds) SKIPPED: cleaning in progress"
            exit 0
        fi
    fi
fi

# ── Network check ──────────────────────────────────────────────────────────────
if ! ping -c 1 -W 5 github.com >/dev/null 2>&1; then
    _fail "no network — cannot reach github.com"
fi

# ── Fetch remote ──────────────────────────────────────────────────────────────
git -C "$REPO_DIR" fetch origin main 2>/dev/null \
    || _fail "git fetch failed — check network or repository access"

NEW_SHA=$(git -C "$REPO_DIR" rev-parse origin/main)

# ── Already up to date ────────────────────────────────────────────────────────
if [[ "$PREV_SHA" == "$NEW_SHA" ]]; then
    _log_result "$(date -Iseconds) OK: already up to date (${PREV_SHA:0:7})"
    exit 0
fi

# ── Hash requirements before pull ─────────────────────────────────────────────
OLD_REQ_HASH=$(sha256sum "$REPO_DIR/pi/zero_requirements.txt" 2>/dev/null | cut -d' ' -f1 || echo "none")

# ── Pull ──────────────────────────────────────────────────────────────────────
git -C "$REPO_DIR" pull --ff-only origin main \
    || _rollback "git pull failed" "$PREV_SHA"

NEW_REQ_HASH=$(sha256sum "$REPO_DIR/pi/zero_requirements.txt" 2>/dev/null | cut -d' ' -f1 || echo "changed")

# ── Reinstall packages only if requirements changed ───────────────────────────
if [[ "$OLD_REQ_HASH" != "$NEW_REQ_HASH" ]]; then
    "$VENV/bin/pip" install -q -r "$REPO_DIR/pi/zero_requirements.txt" \
        || _rollback "pip install failed" "$PREV_SHA"
fi

# ── Restart service ───────────────────────────────────────────────────────────
sudo systemctl restart rosie \
    || _rollback "systemctl restart failed" "$PREV_SHA"

# ── Health check — wait up to 30 s ────────────────────────────────────────────
for _i in $(seq 1 30); do
    sleep 1
    if sudo systemctl is-active rosie >/dev/null 2>&1; then
        _log_result "$(date -Iseconds) OK: ${PREV_SHA:0:7} -> ${NEW_SHA:0:7}"
        exit 0
    fi
done

_rollback "service failed to become active within 30 s" "$PREV_SHA"
