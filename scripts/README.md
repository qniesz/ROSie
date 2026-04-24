# Scripts Directory

This folder now keeps only active operational scripts near the top level.

## Active scripts

- Mapping and no-go tools: `live_plot.py`, `coverage_clean.py`, `coverage_clean_new.py`
- GPIO and hardware tests: `bump_test.py`, `gpio_mag_test.py`, `gpio_toggle.py`
- Driver/service helpers: `activate_lds.py`, `activate_nav2.py`, `setup_ssh.py`, `setup_wifi.ps1`
- Robot diagnostics and control helpers: `query_sensors.py`, `query_setsensor.py`, `setpose.py`, `nav_goal.py`, `test_actuators.py`, `test_mqtt.py`
- Legacy but still local tests: `clean_test.ps1`, `clean_test.py`, `mag_raw.py`, `ping_robot.ps1`

`live_plot.py` now reads map metadata directly from `server/maps/home.yaml` by default, so map origin/resolution stay in sync after remapping. To use a different map, set `ROSIE_MAP_YAML` before running the script.
