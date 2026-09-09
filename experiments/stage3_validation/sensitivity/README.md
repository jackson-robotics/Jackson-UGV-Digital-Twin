# Jackson UGV – Stage 3 Local Sensitivity Dataset

This archive contains the NVIDIA Isaac Sim Stage 3 experiments
used to evaluate local parameter sensitivity in the calibrated Stage 3 environment:

"Real-to-Sim Calibration and Cross-Domain Trajectory Validation of a
Low-Cost Multi-Sensor UGV Digital Twin."

## Experimental campaign

A one-factor-at-a-time local sensitivity analysis was performed around the
active Stage 3 navigation operating point:

- C00_BASE: r = 0.03300 m, B = 0.19800 m, D = 10000
- C01_R_MINUS5: r -5%
- C02_R_PLUS5: r +5%
- C03_B_MINUS5: B -5%
- C04_B_PLUS5: B +5%
- C05_D_MINUS10: D -10%
- C06_D_PLUS10: D +10%

Fifteen trials were attempted for each condition.

Total attempted trials: 105
Successful trials: 104
Aborted trials: 1

The aborted C03_B_MINUS5 trial is retained because it was a valid experimental
outcome and was included in the reported task-completion statistics.

## Main files

- SENSITIVITY_RUN_LEVEL_METRICS.csv
  Run-level quantitative results.

- SENSITIVITY_CONDITION_SUMMARY.csv
  Condition-level statistical summary.

- SENSITIVITY_METRICS_AUDIT.txt
  Metric-definition and analysis audit.

- CAMPAIGN_FINAL_INTEGRITY_AUDIT*.txt
  Final campaign integrity checks.

- C03_R05_ABORT_DIAGNOSTIC.txt
  Diagnostic record for the valid aborted trial.

- runs/
  ROS 2 bag data and run-specific configuration/provenance records for the
  seven official experimental conditions.

## Exclusions

Developmental, rejected, and superseded preliminary runs are intentionally
excluded from this archive. Only the official experimental campaign used in
the reported analysis is included.
