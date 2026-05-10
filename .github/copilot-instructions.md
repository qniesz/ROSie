# ROSie Copilot Instructions

ROSie's active development target is the Raspberry Pi runtime, not local Docker.

## Active Runtime

- Treat the Pi systemd service as the primary runtime: `rosie.service` running under `~/rosie`.
- The Pi runtime environment is `~/rosie-driver.env`; do not overwrite it with the local `.env` unless the user explicitly requests it.
- Validate behavior on the Pi with SSH/SCP, `systemctl`, `journalctl`, MQTT, Home Assistant, Foxglove, and the real robot.

## Normal Development

- Edit files locally in VS Code.
- Sync only changed files to the Pi with `scripts/dev_pi.ps1 sync ...`.
- Run checks on the Pi with `scripts/dev_pi.ps1 check ...`.
- Restart the real Pi service with `scripts/dev_pi.ps1 restart`.
- Collect logs from the Pi with `scripts/dev_pi.ps1 logs` or `scripts/dev_pi.ps1 collect-logs`.
- Commit and push only after Pi validation succeeds and the user explicitly chooses to release.

## Avoid For Pi Zero Development

- Do not run local `docker compose`, local Docker builds, local ROS containers, local service installs, or package installs as the default validation path.
- Only use Docker commands when the user explicitly asks to work on the legacy Pi 4 Docker/Nav2 stack or on a Docker image itself.
- Do not run `~/rosie/pi/update.sh --force`, trigger the HA software update command, commit, push, or change the Pi update state without explicit user confirmation.

## Dev Pi Safety

- Use `scripts/dev_pi.ps1 dev-mode on` before syncing uncommitted files to the Pi.
- Dev mode creates `~/rosie/.rosie-dev-mode`, disables update timers, and makes `pi/update.sh` refuse public Git pulls.
- Use `scripts/dev_pi.ps1 dev-mode off` only when the user is ready for this Pi to accept public Git updates again.