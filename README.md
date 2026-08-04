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
```

## Software Requirements

- Ubuntu 22.04.
- ROS 2 Humble.
- Python 3.10 or later.
- NVIDIA Isaac Sim 4.5 for the virtual platform.
- NVIDIA JetPack and ROS 2 Humble for the physical Jetson platform.

## Build

From the repository root:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

On a new ROS 2 installation, initialize `rosdep` once before installing dependencies:

```bash
sudo rosdep init
rosdep update
```

The physical and virtual platforms use separate ROS domains:

- Physical Jetson: `ROS_DOMAIN_ID=0`
- Virtual Legion and Isaac Sim: `ROS_DOMAIN_ID=10`

## Experimental Datasets

The repository contains processed experimental results and analysis resources for three cross-domain validation stages:

| Stage | Experiment | Physical dataset | Virtual dataset |
|---|---|---|---|
| 1 | 1 m square trajectory | P01, runs R01–R10 | V16, runs R01–R10 |
| 2 | Figure-eight trajectory | P02, runs R01–R10 | V17, runs R01–R10 |
| 3 | Dynamic-cylinder avoidance | P03, runs R06–R15 | V18, runs R06–R15 |

Processed CSV files, figures, reports, and summary statistics are tracked under `data/processed/`.

The original ROS 2 bag archives are distributed separately through the [Raw ROS 2 Bag Datasets v1.0.0 release](https://github.com/jackson-robotics/Jackson-UGV-Digital-Twin/releases/tag/raw-datasets-v1.0.0).

## Integrity Verification

Verify the processed results from the repository root:

```bash
sha256sum -c checksums/processed_results.sha256
```

After downloading all raw dataset assets into the same directory, verify them with:

```bash
sha256sum -c raw_ros2_bag_archives.sha256
```

## Reproducibility

Experiment controllers and execution scripts are located under `scripts/experiments/`. Analysis and data-extraction utilities are located under `scripts/analysis/` and `scripts/data_extraction/`.

Machine-specific addresses and paths are not stored in the repository. Physical Jetson connection parameters must be supplied through environment variables.

## Citation

Formal citation metadata will be added in `CITATION.cff` after the complete author list and publication information have been finalized.

Until then, cite the repository and the corresponding versioned dataset release.

## Licensing

Software source code is licensed under the MIT License; see `LICENSE`.

Data and documentation are licensed under the Creative Commons Attribution 4.0 International License; see `LICENSE-DATA`.

Third-party components and dependencies remain subject to their respective licenses.
