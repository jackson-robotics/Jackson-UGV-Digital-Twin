#!/usr/bin/env python3
"""Recalculate descriptive statistics from the reviewer-package CSV files."""

from __future__ import annotations

import csv
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / "processed_data" / "run_level_results_R06_R15.csv"
AGG = ROOT / "processed_data" / "aggregate_metrics_R06_R15.csv"

FIELDS = {
    "Navigation time": "navigation_time_s",
    "Recoveries": "recoveries",
    "Dynamic trigger displacement": "dynamic_trigger_displacement_m",
    "Ground-truth path length": "ground_truth_path_length_m",
    "Ground-truth final position error": "ground_truth_final_position_error_m",
    "Physical surface clearance": "physical_surface_clearance_m",
    "Global-plan side switches": "global_plan_side_switches",
}


def load_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main() -> None:
    runs = load_csv(RUNS)
    official = {row["metric"]: row for row in load_csv(AGG)}

    if [row["run_id"] for row in runs] != [f"R{i:02d}" for i in range(6, 16)]:
        raise SystemExit("ERROR: expected ordered official runs R06-R15")

    if any(row["status"] != "SUCCEEDED" for row in runs):
        raise SystemExit("ERROR: a run is not marked SUCCEEDED")

    print("Metric, recalculated mean, sample SD, median, min, max, report mean")
    for metric, field in FIELDS.items():
        values = [float(row[field]) for row in runs]
        values_sorted = sorted(values)
        mean = statistics.fmean(values)
        sd = statistics.stdev(values)
        median = statistics.median(values_sorted)
        row = official[metric]
        print(
            f"{metric}: {mean:.9f}, {sd:.9f}, {median:.9f}, "
            f"{min(values):.9f}, {max(values):.9f}; report mean={row['mean']}"
        )

    trigger = [float(row["dynamic_trigger_displacement_m"]) for row in runs]
    if abs(statistics.fmean(trigger) - 0.1217) > 0.0001:
        raise SystemExit("ERROR: trigger mean does not reconcile with the report")

    print("PASS: run order, statuses, and principal descriptive statistics verified.")


if __name__ == "__main__":
    main()
