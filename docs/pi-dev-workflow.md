# Pi Development Workflow

ROSie development has two separate lanes:

- **Dev lane:** edit locally, sync selected files to the Pi, validate on the Pi, restart `rosie.service`, and test on the real robot.
- **Release lane:** after Pi validation succeeds, review diffs, commit, push to Git, then let production Pis update from Git.

The Pi is the runtime. The Windows machine is the editor and Git workstation.

## Put The Pi In Dev Mode

Before syncing uncommitted work, pin the Pi so it cannot pull public code over your test files:

```powershell
.\scripts\dev_pi.ps1 dev-mode on
```

This command:

- sets `ROSIE_MANUAL_UPDATES=true` in `~/rosie-driver.env`
- disables `rosie-update.timer` and `rosie-check-updates.timer`
- creates `~/rosie/.rosie-dev-mode`

While that marker exists, `~/rosie/pi/update.sh` refuses to pull `origin/main`, even when called with `--force` by the Home Assistant update command.

## Sync One Or Two Files

For everyday development, sync only the files you changed:

```powershell
.\scripts\dev_pi.ps1 sync pi\rosie_driver\map_pipeline.py
.\scripts\dev_pi.ps1 sync pi\rosie_driver\main.py pi\rosie_driver\mqtt_bridge.py
```

The files are copied to matching paths under `~/rosie` on the Pi. This does not rebuild Docker images, reinstall packages, or copy the whole repo.

## Check On The Pi

Run syntax checks in the Pi runtime environment:

```powershell
.\scripts\dev_pi.ps1 check pi\rosie_driver\map_pipeline.py
.\scripts\dev_pi.ps1 check tools\slam_toolbox_eval\run_online.sh
```

With no paths, `check` validates the common driver and online SLAM files.

## Restart And Watch Logs

Restart the actual Pi service:

```powershell
.\scripts\dev_pi.ps1 restart
```

If you changed online SLAM sidecar files, restart with the sidecar reset:

```powershell
.\scripts\dev_pi.ps1 restart -Sidecar
```

View recent logs:

```powershell
.\scripts\dev_pi.ps1 logs
```

Follow live logs while testing:

```powershell
.\scripts\dev_pi.ps1 logs -Follow
```

Collect a local diagnostic snapshot:

```powershell
.\scripts\dev_pi.ps1 collect-logs
```

Snapshots are written under `logs/pi/<timestamp>/`, which is ignored by Git.

## Broader Runtime Refresh

Use this only when you intentionally want to refresh the active runtime folders:

```powershell
.\scripts\dev_pi.ps1 sync-runtime
```

It syncs `pi/rosie_driver`, `tools/slam_toolbox_eval`, and selected active diagnostic scripts. Prefer `sync <file>` for normal work.

## Release To Git

When the Pi-tested behavior is ready for everyone:

```powershell
.\scripts\dev_pi.ps1 release-check
git status
git diff
```

Review the files carefully. Do not commit `.env`, logs, tokens, passwords, maps, or local runtime artifacts.

After you intentionally commit and push, allow the Pi to take public updates again:

```powershell
.\scripts\dev_pi.ps1 dev-mode off
ssh rosie@192.168.x.x "~/rosie/pi/update.sh --force"
```

## Important Rules

- Do not use local Docker as the default Pi Zero validation path.
- Do not trigger the HA software update command while dev mode is on.
- Do not run `~/rosie/pi/update.sh --allow-dev-mode` unless you intentionally want to overwrite dev-synced files with Git.
- Keep `~/rosie-driver.env` as the Pi runtime config and `.env` as local connection/development settings.