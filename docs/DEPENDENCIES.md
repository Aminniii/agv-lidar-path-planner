# Python / ROS environment

ROS 2 binary Python packages can leak into virtual environments via `PYTHONPATH` when ROS has been sourced. For **offline** testing, use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q`.

To satisfy ROS testing plugin requirements in an ROS-sourced environment, install `setuptools osrf-pycommon jinja2 typeguard empy` into the appropriate environment. Do not assume that doing so validates ROS 2/Nav2 compatibility. Prefer separate virtual environments for offline planning and ROS integration.

Do not install ROS 2 packages from PyPI as a substitute for your distribution's ROS binaries.
