# Data dictionary

## Processed run-level table

File: `processed_data/run_level_results_R06_R15.csv`

| Field | Type | Unit | Definition |
|---|---:|---:|---|
| `run_id` | text | - | Official run identifier, R06-R15. |
| `status` | category | - | Final Nav2 `NavigateToPose` status. |
| `obstacle_contact` | boolean-like category | - | Operator-observed contact with the cylinder (`yes`/`no`). |
| `navigation_time_s` | numeric | s | Nav2 navigation duration recorded in `navigate_to_pose_action.txt`. |
| `recoveries` | integer | count | Nav2-reported number of recoveries. |
| `dynamic_trigger_displacement_m` | numeric | m | Ground-truth forward displacement when the dynamic cylinder was inserted. |
| `global_plan_side_switches` | integer | count | Number of changes between upper/lower global-route classifications during obstacle negotiation. |
| `route_side` | category | - | Predominant route around the cylinder (`Lower` or `Upper`). |
| `ground_truth_path_length_m` | numeric | m | Integrated planar path length from PhysX ground-truth odometry. |
| `ground_truth_final_position_error_m` | numeric | m | Euclidean distance between final ground-truth position and goal. |
| `physical_surface_clearance_m` | numeric | m | Minimum center-to-center robot-cylinder distance minus 0.135 m physical robot radius and 0.125 m cylinder radius. |
| `data_source_note` | text | - | Provenance and qualification of the row values. |

## Aggregate table

File: `processed_data/aggregate_metrics_R06_R15.csv`

For each metric, the table gives the unit, arithmetic mean, sample standard deviation (`n-1` denominator), median, minimum, maximum, and provenance. Values are those reported by the verified full-precision analysis.

## Raw run directory files

| File pattern | Purpose |
|---|---|
| `*_0.db3` | Raw ROS 2 SQLite3 bag containing time-stamped topics. |
| `metadata.yaml` | ROS 2 bag metadata, topic types, counts, and duration. |
| `db3_sha256.txt` | Per-run DB3 integrity digest. |
| `RUN_NOTES.txt` | Official status, contact/intervention notes, geometry, configuration, and completion time. |
| `navigate_to_pose_action.txt` | Nav2 action feedback and final result. |
| `trigger_displacement.txt` | Recorded ground-truth displacement at cylinder insertion. |
| `initial_*` / `final_*` | Initial/final state snapshots used for quality control. |
| `*_active_params.yaml` | Runtime Nav2 parameters captured for reproducibility. |
| `ekf_stage3_virtual.yaml` | EKF configuration. |
| `nav2_params_stage3_dwb_infl025.yaml` | Nav2 configuration used in the experiment. |
| `stage3_dynamic_cylinder_qos1.py` | Isaac Sim dynamic-cylinder trigger script. |
| `jackson_map_02.yaml`, `jackson_map_02.pgm` | Occupancy-grid map definition and image. |
| `bag_info.txt` | Human-readable ROS bag summary. |

## Coordinate and geometry definitions

- Frame for start/goal coordinates: ROS 2 `map`.
- Start: `(-0.6909, -0.0364, yaw 0)`.
- Goal: `(1.2091, -0.0364, yaw 0)`.
- Nominal straight-line distance: 1.900 m.
- Cylinder center: `(0.6091, -0.0364)`; radius 0.125 m.
- Physical robot radius used for surface-clearance reporting: 0.135 m.
- Nav2 configured robot radius: 0.16 m.

