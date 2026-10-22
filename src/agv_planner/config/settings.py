"""Validated defaults; optional AGV_CONFIG_FILE points to user YAML."""
import os
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path

import yaml


@dataclass(frozen=True)
class RobotSettings:
    wheel_radius_m: float = 0.04
    wheelbase_m: float = 0.18
    track_width_m: float = 0.29
    footprint_length_m: float = 0.34
    footprint_width_m: float = 0.30
    default_clearance_m: float = 0.20
    pose_stale_seconds: float = 1.5

def load_settings(path=None):
    source = Path(path or os.environ.get('AGV_CONFIG_FILE') or files('agv_planner.config').joinpath('robot_config.yaml'))
    data = yaml.safe_load(source.read_text(encoding='utf8')) or {}
    robot, planning, ros = (data.get(k) or {} for k in ('robot','planning','ros'))
    defaults = RobotSettings()
    values = {name: section.get(name, getattr(defaults,name)) for section in (robot,planning,ros) for name in section if name in RobotSettings.__dataclass_fields__}
    settings = RobotSettings(**values)
    if any(getattr(settings, k) <= 0 for k in RobotSettings.__dataclass_fields__):
        raise ValueError('Robot dimensions, clearance and timeouts must be positive')
    return settings
