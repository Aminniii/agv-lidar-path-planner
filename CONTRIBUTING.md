# Contributing

Thanks for your interest in the AGV LiDAR Path Planner.

## Development setup

```bash
git clone https://github.com/Aminniii/agv-lidar-path-planner.git
cd agv-lidar-path-planner

python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
```

## Before submitting changes

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python -m compileall -q src
python -m pip check
```

To inspect test coverage:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest \
  --cov=agv_planner --cov-report=term-missing
```

## Pull requests

- Create a branch for your change.
- Explain the problem and the proposed solution.
- Include or update tests for behavioral changes.
- Update documentation when user-facing behavior changes.
- Do not include virtual environments, generated coverage files,
  personal map data, tokens or credentials.
- Do not describe offline tests as real-robot safety validation.

## Bug reports

Use the repository bug-report template. Include reproducible steps,
your operating system, Python version and relevant error output.

## Safety-related changes

Any change to footprint checks, obstacle clearance or ROS 2/Nav2
execution requires careful review and separate hardware validation.
