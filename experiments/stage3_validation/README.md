# Stage 3 Sensitivity and Generalization Experiments

This directory contains the lightweight, repository-tracked materials from the
NVIDIA Isaac Sim experiments used to evaluate sensitivity and robustness in the
calibrated Stage 3 environment of the Jackson UGV digital twin.

## Campaigns

| Campaign | Conditions | Attempted runs | Successful runs |
|---|---:|---:|---:|
| Stage 3 local sensitivity | C00–C06 | 105 | 104 |
| Stage 3 generalization | G01–G04 | 60 | 60 |

The unsuccessful `C03_B_MINUS5` trial was retained as a valid experimental
outcome and was not repeated.

The generalization comparison contains 75 observations: 60 G01–G04 trials plus
the 15-run C00 baseline reused from the sensitivity campaign. The complete
validation collection therefore contains 165 unique runs.

## Repository Contents

- `sensitivity/`: processed metrics, audits, conditions, and final scripts.
- `generalization/`: processed metrics, audits, configuration, and final scripts.

Raw ROS 2 bags are excluded from the Git history. They are distributed through
the [Stage 3 Sensitivity and Generalization Datasets v1.0.0 release](https://github.com/jackson-robotics/Jackson-UGV-Digital-Twin/releases/tag/stage3-validation-datasets-v1.0.0).

## Integrity Verification

Download the two ZIP archives and `MANIFEST_STAGE3_VALIDATION.sha256` into the
same directory, then run:

```bash
sha256sum -c MANIFEST_STAGE3_VALIDATION.sha256
```
