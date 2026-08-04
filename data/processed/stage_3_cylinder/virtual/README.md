# V18 Virtual Cylinder-Avoidance Processed Results

This package contains the processed results and supporting evidence for the ten official Jackson Stage 3 virtual dynamic-cylinder runs (R06-R15), executed in Isaac Sim with ROS 2 Humble, Nav2, and the DWB local planner.

The original ROS 2 bags are distributed separately as `V18_virtual_cylinder_avoidance_R06_R15.zip`.

## Main results

- Navigation outcome: 10/10 runs `SUCCEEDED` (100%).
- Operator-recorded obstacle contacts: 0/10.
- Navigation time: 18.063 +/- 14.823 s (mean +/- sample SD); median 13.317 s.
- Ground-truth path length: 1.888 +/- 0.039 m.
- Ground-truth final position error: 0.212 +/- 0.030 m.
- Physical surface clearance: 0.074 +/- 0.016 m.
- Dynamic obstacle trigger displacement: 0.1217 +/- 0.0012 m.

R13 is retained as a valid operational outlier. It completed successfully in 60.150 s, recorded five recoveries and 25 route-side switches, and had no recorded obstacle contact or operator intervention.

## Contents

- `processed_data/run_level_results_R06_R15.csv`: one row per official run.
- `processed_data/aggregate_metrics_R06_R15.csv`: descriptive statistics used in the report.
- `processed_data/Jackson_Stage3_R06_R15_Results.xlsx`: spreadsheet version.
- `documentation/DATA_DICTIONARY.md`: field definitions and units.
- `documentation/EXPERIMENT_CONFIGURATION.md`: frozen experiment parameters.
- `scripts/recalculate_summary.py`: recalculates the principal descriptive statistics.
- `scripts/export_rosbag_topics.py`: exports selected topics from an extracted bag.
- `report/virtual_official_report_R06_R15.pdf`: detailed English report.
- `evidence/V18_virtual_run_evidence.txt`: consolidated acquisition notes and final-pose evidence.

## Verification

From the extracted package directory, run:

```bash
python3 scripts/recalculate_summary.py
```

The expected final line is:

```text
PASS: run order, statuses, and principal descriptive statistics verified.
```

## Interpretation note

Navigation time is not normally distributed because R13 is a valid high-duration run. Report both the mean and sample standard deviation and the median; do not remove R13 solely because it is an outlier.

