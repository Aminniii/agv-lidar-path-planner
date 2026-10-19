# Software Architecture

## Overview

The application provides offline path planning from a saved ROS
occupancy-grid map, with an optional ROS 2/Nav2 interface.

```mermaid
flowchart LR
    MAP["Map YAML / PGM"] --> LOAD["Map loader"]
    LOAD --> GRID["Occupancy grid"]
    GRID --> CLEAR["Obstacle clearance"]
    CLEAR --> PLAN["A* / Dijkstra / Theta*"]
    PLAN --> SMOOTH["Smoothing + validation"]
    SMOOTH --> GUI["Tkinter interface"]
    SMOOTH --> SIM["Offline simulator"]
    SIM --> WHEELS["Mecanum kinematics"]
    SMOOTH --> EXPORT["Route export"]
    EXPORT --> NAV["Optional Nav2 workflow"]
```

## Source layout

| Module | Responsibility |
|---|---|
| `src/agv_planner/planner.py` | Main application entry point and GUI logic |
| `src/agv_planner/core.py` | Shared planning functionality |
| `src/agv_planner/maps/` | Occupancy-grid processing |
| `src/agv_planner/planning/` | Algorithms, proximity and safety |
| `src/agv_planner/simulation/` | Robot and wheel simulation |
| `src/agv_planner/ros/` | ROS-related support |
| `src/agv_planner/config/` | Robot configuration |
| `tests/` | Automated offline tests |

## Robot system context

The wider robotics project uses a Raspberry Pi, Arduino Mega,
four mecanum wheels, encoders and RPLIDAR.

The desktop planner is separate from the motor controller.

```mermaid
flowchart TD
    L["RPLIDAR"] --> PI["Raspberry Pi / ROS 2"]
    ENC["Wheel encoders"] --> MEGA["Arduino Mega"]
    MEGA <--> PI
    PI --> ODOM["Odometry + TF"]
    PI --> SCAN["Laser scans"]
    SCAN --> SLAM["SLAM / saved map"]
    ODOM --> SLAM
    SLAM --> APP["Desktop path planner"]
    APP -. "Experimental interface" .-> NAV["Nav2"]
    NAV -. "Motion commands" .-> PI
    MEGA --> DRIVERS["Motor drivers"]
    DRIVERS --> WHEELS["Four mecanum wheels"]
```

This diagram represents the intended system-level integration.
It is not proof of end-to-end Nav2 operation.

## Safety boundary

Map-based planning and offline collision prediction do not replace
hardware emergency stops, live sensing, localization validation,
or supervised robot testing.
