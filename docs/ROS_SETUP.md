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
