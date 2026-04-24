# Security & Secrets

## Reporting a vulnerability

If you find a security issue, please open a private security advisory on GitHub
(`Security` tab → `Report a vulnerability`) rather than a public issue.

## What this repo expects you to provide locally

The following values are **never committed** and must be supplied via your own
`.env` file (copy `.env.example`) or interactively at deploy time:

| Variable             | Used by                                  |
| -------------------- | ---------------------------------------- |
| `MQTT_HOST`          | docker-compose, deploy_pi.ps1, scripts/  |
| `MQTT_PORT`          | (defaults to 1883)                       |
| `MQTT_USER`          | docker-compose, deploy_pi.ps1, scripts/  |
| `MQTT_PASS`          | docker-compose, deploy_pi.ps1, scripts/  |
| `HA_TOKEN`           | Home Assistant long-lived access token   |
| `ROSIE_PI_HOST`      | scripts that SSH into the Pi             |

`pi/docker-compose.yaml` uses `${VAR:?msg}` substitution, so docker compose
will refuse to start if a required secret is missing instead of falling back
to a hardcoded default.

## Files that are intentionally **not** tracked

- `.env`, `*.env` (except `.env.example`)
- Any file under `secrets/`
- Personal SSH keys
- Home Assistant long-lived access tokens

These are excluded by `.gitignore`.

## If you fork this repo

1. Copy `.env.example` to `.env` and fill in your own values.
2. Set a strong, unique password on your MQTT broker.
3. Do not commit `.env`.
4. Rotate any credential you suspect has been exposed.

## What is safe to be public

- All Python / shell / PowerShell source
- Systemd unit files (no embedded secrets)
- The `.env.example` template (fields are blank)
- Hardware documentation and dashboard YAML
