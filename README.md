# AGV LiDAR Path Planner — v9.2 release candidate (MIT)

Desktop LiDAR occupancy-grid route planning and mecanum simulation, with **optional,
experimental** ROS 2/Nav2 connectivity. Tkinter GUI, A*/Dijkstra/Theta*, multiple
stops, path smoothing, virtual obstacles, wheel telemetry and CSV/YAML exports.

**Status:** offline GUI has been exercised on Ubuntu with real maps in v9.1;
this release candidate still requires manual regression testing. **Real-robot Nav2
execution has not been verified.** Do not rely on this application as a safety system.

## Install offline (Ubuntu)

```bash
sudo apt install python3-tk python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
agv-path-planner
```

Choose **Load Map** and select `examples/demo_map.yaml`. Set a start and goal,
then plan a route and simulate. Real maps use ROS YAML + PGM pairs.

## Safety model

The orange ring is a **proximity visualization**, not a collision boundary.
The offline simulator checks an oriented rectangular footprint at interpolated
poses and pauses if occupied or unknown cells intersect it. Paths are validated
against the inflated grid after smoothing. For Nav2, a conservative circular
footprint preflight is applied. Map accuracy, robot dimensions, localization,
controller behavior and moving obstacles are not guaranteed.

Example footprint 0.34 x 0.30 m has circumscribed radius ~0.227 m; 0.20 m
clearance is insufficient for this conservative Nav2 preflight. Confirm actual
robot dimensions before using a real robot.

## Configuration

Defaults: `src/agv_planner/config/robot_config.yaml`.
Override with `AGV_CONFIG_FILE=/absolute/path/robot_config.yaml`.

## Tests and development

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python -m compileall -q src
python -m pip check
```

CI tests offline algorithms; it does not test GUI interaction or hardware.
See `docs/RELEASE_CHECKLIST.md`, `docs/ROS_SETUP.md`, and `docs/DEPENDENCIES.md`.

## Limitations

- ROS 2/Nav2 requires a compatible distro and Python environment, TF and Nav2 actions.
- Pathfinding still runs on the Tkinter main thread; large maps can stall the GUI.
- No live LiDAR collision avoidance is provided by this desktop application.
- Distributed under the MIT License. See [LICENSE](LICENSE).

## License

MIT License — copyright (c) 2026 AGV LiDAR Path Planner contributors. See [LICENSE](LICENSE).
