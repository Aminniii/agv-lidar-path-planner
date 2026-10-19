# Release readiness checklist

## Verified automatically
- [ ] `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q`
- [ ] `python -m pip check` in a clean offline environment
- [ ] Python compilation and GitHub Actions CI on supported versions

## Verify manually on Ubuntu desktop
- [ ] Open example map and real SLAM map
- [ ] Set start, goal, stops; plan with A*, Dijkstra, Theta*
- [ ] Zoom, pan, Fit and resize; coordinates remain correct
- [ ] Add/remove virtual obstacles; confirm route invalidation
- [ ] Start, pause and stop simulation; check wheel telemetry
- [ ] Confirm obstacle proximity warning and predicted collision pause
- [ ] Export CSV and preview YAML; inspect coordinates and stop order
- [ ] Test on large map; record responsiveness and peak RAM

## Real ROS 2/Nav2 (not verified by CI)
- [ ] Validate ROS distro and Python ABI match
- [ ] Validate TF map->odom->base_link and localization age
- [ ] Check /follow_path action server, controller, and collision monitor
- [ ] Check physical robot footprint dimensions and clearance
- [ ] Test cancellation, watchdog and physical emergency stop
- [ ] Supervised low-speed test in a clear area with spotter

## Before public GitHub publication
- [ ] Choose a license and replace LICENSE placeholder
- [ ] Remove private maps, names, secrets, IP addresses and credentials
- [ ] Add screenshots and credit third-party assets
- [ ] Review README limitations and installation instructions
- [ ] Tag a release only after manual tests
