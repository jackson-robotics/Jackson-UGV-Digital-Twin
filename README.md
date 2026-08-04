# Jackson UGV Digital Twin

Open research and development repository for the Jackson low-cost multi-sensor unmanned ground vehicle (UGV) and its digital twin implemented with ROS 2 and NVIDIA Isaac Sim.

The project integrates the physical robot, virtual sensing, state estimation, SLAM, navigation, experimental trajectory controllers, real-to-sim validation, and physical-to-virtual mirror operation.

## Project Scope

This repository contains the consolidated source code and configurations developed for:

- Physical Jackson UGV operation on an NVIDIA Jetson Orin Nano.
- Virtual Jackson simulation in NVIDIA Isaac Sim.
- ROS 2 sensor acquisition and preprocessing.
- Wheel odometry and IMU fusion using `robot_localization`.
- Mapping and localization using SLAM Toolbox, Map Server, and AMCL.
- Autonomous navigation using Nav2 and DWB.
- Square, figure-eight, and dynamic-cylinder experiments.
- Physical-to-virtual pose synchronization and mirror-mode logging.
- Reproducible processing of experimental results.

## System Architecture

| Component | Physical platform | Virtual platform |
|---|---|---|
| Compute | NVIDIA Jetson Orin Nano 8 GB | Lenovo Legion with NVIDIA RTX 4060 |
| Operating system | Ubuntu / JetPack | Ubuntu 22.04 |
| Middleware | ROS 2 Humble | ROS 2 Humble |
| Simulation | — | NVIDIA Isaac Sim 4.5 |
| ROS domain | `0` | `10` |
| Motion model | Differential drive | Differential-drive articulation |
| State estimation | Wheel odometry + IMU + EKF | Joint states + virtual IMU + EKF |
| Localization | SLAM Toolbox / AMCL | Map Server / AMCL |
| Navigation | Nav2 + DWB | Nav2 + DWB |

## Repository Structure

```text
.
├── config/                    # Navigation and state-estimation configurations
├── data/processed/            # Small processed datasets and CSV results
├── digital_twin_sync/         # Physical-to-virtual synchronization tools
├── docs/                      # Architecture, installation, and protocols
├── hardware/                  # Hardware documentation
├── isaac_sim/scenes/          # Canonical Isaac Sim USD scenes
├── maps/physical/             # ROS occupancy-grid maps
├── publications/              # Publication-specific supporting material
├── results/                   # Tables and figures
├── ros2_ws/src/
│   ├── physical_jetson/       # Physical robot ROS 2 packages
│   └── virtual_legion/        # Virtual robot ROS 2 packages
└── scripts/
    ├── analysis/              # Data-analysis scripts
    ├── data_extraction/       # Bag and topic extraction utilities
    └── experiments/           # Physical and virtual experiments
