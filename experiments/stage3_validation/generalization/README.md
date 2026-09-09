# Jackson UGV – Stage 3 Robustness and Generalization Dataset

This archive contains the NVIDIA Isaac Sim Stage 3 robustness and
generalization experiments used to evaluate robustness in the calibrated Stage 3 environment:

"Real-to-Sim Calibration and Cross-Domain Trajectory Validation of a
Low-Cost Multi-Sensor UGV Digital Twin."

## Experimental campaign

Four controlled perturbation conditions were evaluated with 15 independent
navigation trials per condition:

- G01_CYL_PLUS_Y:
  Dynamic cylindrical obstacle shifted by +0.15 m along the map-frame Y axis.

- G02_CYL_MINUS_Y:
  Dynamic cylindrical obstacle shifted by -0.15 m along the map-frame Y axis.

- G03_LOW_FRICTION:
  Explicit low-friction Physics Material with
  static friction coefficient = 0.30 and
  dynamic friction coefficient = 0.25.

- G04_LIDAR_DEGRADED:
  Additional Gaussian range noise with sigma = 0.03 m and
  5% random valid-beam dropout.

Total new trials in this archive: 60
Successful trials: 60

## Nominal baseline

The validation analysis uses 75 trials for the complete robustness/generalization
analysis because the 15-run C00 nominal baseline is included in the comparison.

Those C00 baseline trials are not duplicated here. They are contained in:

Stage3_sensitivity_R01_R15_revision.zip

under:

runs/C00_BASE/

Therefore, the complete generalization comparison consists of:

- C00 nominal baseline: 15 trials
- G01: 15 trials
- G02: 15 trials
- G03: 15 trials
- G04: 15 trials

Total: 75 trials.

## Main files

- GENERALIZATION_RUN_LEVEL_METRICS.csv
  Run-level navigation metrics.

- GENERALIZATION_CONDITION_SUMMARY.csv
  Condition-level statistical summary.

- GENERALIZATION_METRICS_AUDIT.txt
  Metric-definition and analysis audit.

- G04_LIDAR_ACTUAL_AUDIT.csv
  Independent verification of the applied LiDAR perturbation.

- SURFACE_MATERIAL_AUDIT.txt
  Verification of the explicit G03 low-friction surface condition.

- scripts/
  Analysis and perturbation scripts used for the campaign.

- config/
  Stage 3 configuration files used for the tested conditions.

- runs/
  ROS 2 bag data and run-specific configuration/provenance records for G01-G04.

## LiDAR verification

The G04 perturbation was independently verified from the recorded ROS 2 bags.
The aggregate measured values obtained in the validation analysis were:

- actual valid-beam dropout rate: 4.997%
- additional range-noise standard deviation: 0.03001 m

## Exclusions

The C00 baseline is intentionally not duplicated because the same 15 baseline
runs are already contained in the Stage 3 sensitivity archive.

Developmental, rejected, or superseded runs are not included.
