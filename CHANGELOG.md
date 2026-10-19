# Changelog

Notable changes to the AGV LiDAR Path Planner are documented here.

## [Unreleased]

### Documentation
- Professional README and system architecture documentation
- Compatibility matrix and contribution guidelines
- GitHub issue templates
- CI test coverage reporting
- Project and licensing attribution

## [0.9.2] - 2026-10-19

### Initial public release
- Tkinter desktop interface for occupancy-grid planning
- A*, Dijkstra and Theta* route planning
- Obstacle inflation and collision-aware smoothing
- Multiple stops and route export
- Offline mecanum robot simulation
- Simulated proximity and footprint collision checks
- Optional experimental ROS 2/Nav2 integration
- Automated offline tests and GitHub Actions CI

### Known limitations
- Large maps may block the GUI during planning
- ROS 2/Nav2 real-robot operation remains experimental
- Offline tests do not validate physical robot safety
