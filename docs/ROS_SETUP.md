# ROS 2 environment isolation

The offline application is installable without ROS 2. ROS 2 binaries built for one Python
minor version may not import into a virtual environment using another minor version.
Use a ROS-matched interpreter and source the appropriate distro before enabling Nav2.

To test offline without ROS pytest plugins:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

If the shell has sourced ROS, `PYTHONPATH` can expose ROS packages to an offline venv.
Use a fresh shell without sourcing ROS for standalone work. In a ROS-enabled shell,
`python -m pip check` may report incomplete ROS dependencies; this is separate from
standalone planner requirements. Avoid mixing Python 3.14 venvs with incompatible ROS
binary extensions. Do not install random PyPI replacements for ROS binary packages.

The app's Nav2 execution is experimental and must not be treated as collision avoidance.
## RViz2 visualization (ROS 2 Jazzy)

This workflow displays the demo occupancy map and planner preview path in RViz2 without a physical robot. It targets Ubuntu 24.04 with ROS 2 Jazzy; other distributions are not verified by these steps.

### Install ROS packages

```bash
sudo apt update
sudo apt install ros-jazzy-rviz2 ros-jazzy-nav2-map-server
```

### Start the map server

From the repository root, in a ROS-enabled terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$(pwd)/examples/demo_map.yaml"
```

Leave that terminal running. In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic list
```

Confirm that `/map` appears.

### Configure RViz2

In a ROS-enabled terminal, run `rviz2`. Set Global Options > Fixed Frame to `map`. Add a Map display on `/map` and a Path display on `/planner_preview_path`. In the Path display, set Durability Policy to **Transient Local** to receive the last retained path.

Start the planner using its documented ROS-enabled workflow and publish a preview path. Verify the topic with:

```bash
ros2 topic type /planner_preview_path
```

Expected type: `nav_msgs/msg/Path`. The path frame should be `map`.

### Troubleshooting

- Missing map: check that the map server is activated and `examples/demo_map.yaml` references `demo_map.pgm`.
- Invisible path: check that a path was published and RViz Path durability is Transient Local.
- Python/ROS import errors: use a ROS-matched Python interpreter and avoid mixing incompatible environments.
- Offline pytest plugin errors: run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q`. This is for offline tests, not ROS-specific tests.

RViz visualization does not validate Nav2 execution, obstacle avoidance, or real-robot safety.
