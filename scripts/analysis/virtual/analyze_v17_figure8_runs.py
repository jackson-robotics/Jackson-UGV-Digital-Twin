#!/usr/bin/env python3
"""Analyze the ten V17 virtual figure-eight closed-loop ROS 2 bags.

The controller uses /odometry/filtered.  /ground_truth/odom is retained only
as an independent reference.  All trajectories are expressed in the local
frame of their first sample before metrics are calculated.
"""

import argparse
import csv
import math
import os
import re
import statistics
import zipfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message


RADIUS = 0.5
EXPECTED_RUNS = 10
ODOM_TOPICS = ("/ground_truth/odom", "/wheel/odom", "/odometry/filtered")
LABELS = {
    "/ground_truth/odom": "Ground truth",
    "/wheel/odom": "Wheel odometry",
    "/odometry/filtered": "EKF",
}
COLORS = {
    "/ground_truth/odom": "#111111",
    "/wheel/odom": "#2878b5",
    "/odometry/filtered": "#e4572e",
}


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cumulative_rotation(yaw):
    if len(yaw) < 2:
        return 0.0
    return float(np.sum(np.arctan2(np.sin(np.diff(yaw)), np.cos(np.diff(yaw)))))


def localize(points):
    """Return t, x, y, yaw relative to the initial pose."""
    a = np.asarray(points, dtype=float)
    x0, y0, yaw0 = a[0, 1], a[0, 2], a[0, 3]
    dx, dy = a[:, 1] - x0, a[:, 2] - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    x = c * dx + s * dy
    y = -s * dx + c * dy
    yaw = np.unwrap(a[:, 3]) - yaw0
    return np.column_stack((a[:, 0], x, y, yaw))


def clip_points(points, start, end):
    a = np.asarray(points, dtype=float)
    return a[(a[:, 0] >= start) & (a[:, 0] <= end)]


def path_length(a):
    if len(a) < 2:
        return float("nan")
    return float(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum())


def radial_metrics(a, center_y):
    if not len(a):
        return (float("nan"), float("nan"))
    errors = np.hypot(a[:, 1], a[:, 2] - center_y) - RADIUS
    return float(np.sqrt(np.mean(errors ** 2))), float(np.max(np.abs(errors)))


def source_metrics(a, crossover):
    first = a[a[:, 0] <= crossover]
    second = a[a[:, 0] >= crossover]
    closure = float(math.hypot(a[-1, 1], a[-1, 2]))
    ccw_rmse, ccw_max = radial_metrics(first, RADIUS)
    cw_rmse, cw_max = radial_metrics(second, -RADIUS)

    def span(loop, axis):
        return float(np.max(loop[:, axis]) - np.min(loop[:, axis]))

    return {
        "samples": int(len(a)),
        "path_length_m": path_length(a),
        "closure_error_m": closure,
        "final_x_m": float(a[-1, 1]),
        "final_y_m": float(a[-1, 2]),
        "net_yaw_deg": math.degrees(float(a[-1, 3])),
        "ccw_rotation_deg": math.degrees(cumulative_rotation(first[:, 3])),
        "cw_rotation_deg": math.degrees(cumulative_rotation(second[:, 3])),
        "ccw_x_diameter_m": span(first, 1),
        "ccw_y_diameter_m": span(first, 2),
        "cw_x_diameter_m": span(second, 1),
        "cw_y_diameter_m": span(second, 2),
        "ccw_radial_rmse_m": ccw_rmse,
        "ccw_max_radial_error_m": ccw_max,
        "cw_radial_rmse_m": cw_rmse,
        "cw_max_radial_error_m": cw_max,
    }


def interpolate_reference(reference, estimate):
    """Interpolate GT at estimator timestamps and return position/yaw errors."""
    times = estimate[:, 0]
    valid = (times >= reference[0, 0]) & (times <= reference[-1, 0])
    e = estimate[valid]
    gt_x = np.interp(e[:, 0], reference[:, 0], reference[:, 1])
    gt_y = np.interp(e[:, 0], reference[:, 0], reference[:, 2])
    gt_yaw = np.interp(e[:, 0], reference[:, 0], reference[:, 3])
    pos = np.hypot(e[:, 1] - gt_x, e[:, 2] - gt_y)
    yaw = np.arctan2(np.sin(e[:, 3] - gt_yaw), np.cos(e[:, 3] - gt_yaw))
    return pos, yaw


def bag_directory_candidates(root):
    pattern = re.compile(r"^V17_figure8_ekf_closed_loop_(R\d{2})_")
    found = []
    for child in Path(root).iterdir():
        match = pattern.match(child.name)
        if child.is_dir() and match and (child / "metadata.yaml").is_file():
            found.append((match.group(1), child))
    found.sort(key=lambda item: int(item[0][1:]))
    return found


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    odom = {topic: [] for topic in ODOM_TOPICS}
    commands, logs = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp * 1.0e-9
        if topic in odom:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            odom[topic].append((t, p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation)))
        elif topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            commands.append((t, msg.linear.x, msg.angular.z))
        elif topic == "/rosout":
            logs.append(deserialize_message(data, Log).msg)
    return odom, np.asarray(commands, dtype=float), logs


def motion_window(commands):
    speed = np.hypot(commands[:, 1], commands[:, 2])
    active = np.flatnonzero(speed > 1.0e-4)
    if not len(active):
        raise RuntimeError("No active /cmd_vel samples were found")
    start, end = commands[active[0], 0], commands[active[-1], 0]
    moving = commands[(commands[:, 0] >= start) & (commands[:, 0] <= end)]
    positive = np.flatnonzero(moving[:, 2] > 0.02)
    negative = np.flatnonzero(moving[:, 2] < -0.02)
    if not len(positive) or not len(negative):
        raise RuntimeError("Could not identify both CCW and CW command segments")
    first_negative = negative[negative > positive[0]]
    if not len(first_negative):
        raise RuntimeError("Could not identify figure-eight crossover")
    crossover = moving[first_negative[0], 0]
    return start, end, crossover


def parsed_log_values(logs):
    text = "\n".join(logs)
    values = {}
    patterns = {
        "controller_closure_m": r"Final EKF pose delta:.*?closure=([+-]?[\d.]+) m",
        "controller_final_yaw_deg": r"Final net EKF yaw:\s*([+-]?[\d.]+) deg",
        "controller_motion_duration_s": r"Motion duration:\s*([+-]?[\d.]+) s",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        values[key] = float(match.group(1)) if match else float("nan")
    values["completed"] = "FIGURE8 FINISHED: reason=completed" in text
    return values


def analyze_run(run, path):
    odom, commands, logs = read_bag(path)
    start, end, crossover = motion_window(commands)
    trajectories = {}
    metrics = {}
    for topic in ODOM_TOPICS:
        points = clip_points(odom[topic], start, end)
        if len(points) < 20:
            raise RuntimeError(f"{topic} has too few motion samples")
        trajectories[topic] = localize(points)
        metrics[topic] = source_metrics(trajectories[topic], crossover)

    gt = trajectories["/ground_truth/odom"]
    comparisons = {}
    for topic in ("/wheel/odom", "/odometry/filtered"):
        pos, yaw = interpolate_reference(gt, trajectories[topic])
        comparisons[topic] = {
            "position_rmse_m": float(np.sqrt(np.mean(pos ** 2))),
            "maximum_position_error_m": float(np.max(pos)),
            "yaw_rmse_deg": math.degrees(float(np.sqrt(np.mean(yaw ** 2)))),
            "maximum_yaw_error_deg": math.degrees(float(np.max(np.abs(yaw)))),
            "heading_error_std_deg": math.degrees(float(np.std(yaw, ddof=1))),
        }

    row = {"run": run, "bag": path.name, "completed": parsed_log_values(logs)["completed"]}
    row.update(parsed_log_values(logs))
    row["motion_duration_s"] = end - start
    for topic, prefix in (("/ground_truth/odom", "gt"), ("/wheel/odom", "wheel"), ("/odometry/filtered", "ekf")):
        for key, value in metrics[topic].items():
            row[f"{prefix}_{key}"] = value
    for topic, prefix in (("/wheel/odom", "wheel"), ("/odometry/filtered", "ekf")):
        for key, value in comparisons[topic].items():
            row[f"{prefix}_{key}"] = value

    for prefix in ("wheel", "ekf"):
        row[f"{prefix}_D_rot_deg"] = (
            abs(row[f"{prefix}_ccw_rotation_deg"] - row["gt_ccw_rotation_deg"])
            + abs(row[f"{prefix}_cw_rotation_deg"] - row["gt_cw_rotation_deg"])
        )
        row[f"{prefix}_D_trans_final_m"] = math.hypot(
            row[f"{prefix}_final_x_m"] - row["gt_final_x_m"],
            row[f"{prefix}_final_y_m"] - row["gt_final_y_m"],
        )
    return row, trajectories, crossover


def write_csv(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def t_critical_95(n):
    table = {2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571,
             7: 2.447, 8: 2.365, 9: 2.306, 10: 2.262, 11: 2.228,
             12: 2.201, 13: 2.179, 14: 2.160, 15: 2.145}
    return table.get(n, 1.96)


def statistics_rows(rows):
    preferred = [
        "motion_duration_s", "gt_path_length_m", "gt_closure_error_m",
        "gt_ccw_x_diameter_m", "gt_ccw_y_diameter_m",
        "gt_cw_x_diameter_m", "gt_cw_y_diameter_m",
        "gt_ccw_radial_rmse_m", "gt_cw_radial_rmse_m",
        "gt_ccw_rotation_deg", "gt_cw_rotation_deg",
        "ekf_path_length_m", "ekf_closure_error_m", "ekf_net_yaw_deg",
        "ekf_position_rmse_m", "ekf_maximum_position_error_m",
        "ekf_yaw_rmse_deg", "ekf_maximum_yaw_error_deg",
        "ekf_heading_error_std_deg", "ekf_D_rot_deg", "ekf_D_trans_final_m",
    ]
    output = []
    for key in preferred:
        values = [float(row[key]) for row in rows if math.isfinite(float(row[key]))]
        n = len(values)
        mean = statistics.fmean(values)
        std = statistics.stdev(values) if n > 1 else 0.0
        margin = t_critical_95(n) * std / math.sqrt(n) if n > 1 else 0.0
        output.append({
            "metric": key, "n": n, "mean": mean, "std": std,
            "median": statistics.median(values), "minimum": min(values),
            "maximum": max(values),
            "cv_percent": 100.0 * std / abs(mean) if mean else float("nan"),
            "ci95_lower": mean - margin, "ci95_upper": mean + margin,
        })
    return output


def ideal_figure8(samples=500):
    a = np.linspace(0.0, 2.0 * math.pi, samples)
    first = np.column_stack((RADIUS * np.sin(a), RADIUS - RADIUS * np.cos(a)))
    second = np.column_stack((RADIUS * np.sin(a), -RADIUS + RADIUS * np.cos(a)))
    return first, second


def save_trajectory_csv(path, trajectories):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["source", "time_s", "x_m", "y_m", "yaw_rad"])
        for topic, points in trajectories.items():
            for point in points:
                writer.writerow([LABELS[topic], *point])


def plot_trajectories(path, analyses):
    fig, ax = plt.subplots(figsize=(8.2, 7.0))
    ideal1, ideal2 = ideal_figure8()
    ax.plot(ideal1[:, 0], ideal1[:, 1], "--", color="#777777", lw=2, label="Ideal")
    ax.plot(ideal2[:, 0], ideal2[:, 1], "--", color="#777777", lw=2)
    for topic in ODOM_TOPICS:
        for index, (_, trajectories, _) in enumerate(analyses):
            p = trajectories[topic]
            ax.plot(p[:, 1], p[:, 2], color=COLORS[topic], alpha=0.20 if topic != "/ground_truth/odom" else 0.28,
                    lw=1.0, label=LABELS[topic] if index == 0 else None)
    ax.scatter([0], [0], marker="x", s=70, color="black", zorder=5, label="Start")
    ax.set(title="V17 figure-eight trajectories (N=10)", xlabel="Local x (m)", ylabel="Local y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25); ax.legend(loc="best")
    fig.tight_layout(); fig.savefig(path, dpi=220); plt.close(fig)


def grouped_plot(path, rows, keys, labels, ylabel, title, reference=None):
    x = np.arange(len(rows)); width = 0.78 / len(keys)
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    for i, (key, label) in enumerate(zip(keys, labels)):
        ax.bar(x + (i - (len(keys)-1)/2) * width, [r[key] for r in rows], width, label=label)
    if reference is not None:
        ax.axhline(reference, color="black", ls="--", lw=1.4, label="Target")
    ax.set_xticks(x, [r["run"] for r in rows]); ax.set_ylabel(ylabel); ax.set_title(title)
    ax.grid(axis="y", alpha=0.25); ax.legend(); fig.tight_layout(); fig.savefig(path, dpi=220); plt.close(fig)


def endpoint_plot(path, analyses):
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    for topic in ODOM_TOPICS:
        endpoints = np.array([a[1][topic][-1, 1:3] for a in analyses])
        ax.scatter(endpoints[:, 0], endpoints[:, 1], s=42, color=COLORS[topic], label=LABELS[topic])
    ax.scatter([0], [0], marker="x", s=80, color="black", label="Start")
    ax.set(title="Endpoint dispersion", xlabel="Final x (m)", ylabel="Final y (m)")
    ax.set_aspect("equal", adjustable="box"); ax.grid(True, alpha=0.25); ax.legend()
    fig.tight_layout(); fig.savefig(path, dpi=220); plt.close(fig)


def publication_readme(path, rows, stats):
    lookup = {r["metric"]: r for r in stats}
    def mean(key): return lookup[key]["mean"]
    text = f"""V17 Virtual Figure-8 EKF Closed-Loop Validation
================================================

Runs: {len(rows)} completed runs (R01-R10)
Target loop radius: {RADIUS:.3f} m (1.000 m target diameter)
Controller feedback: /odometry/filtered
Independent reference: /ground_truth/odom

Key results (mean across runs)
--------------------------------
Motion duration: {mean('motion_duration_s'):.3f} s
Ground-truth path length: {mean('gt_path_length_m'):.4f} m
Ground-truth closure error: {mean('gt_closure_error_m'):.4f} m
EKF closure error: {mean('ekf_closure_error_m'):.4f} m
EKF position RMSE vs ground truth: {mean('ekf_position_rmse_m'):.4f} m
EKF yaw RMSE vs ground truth: {mean('ekf_yaw_rmse_deg'):.3f} deg
Cumulative rotational discrepancy D_rot: {mean('ekf_D_rot_deg'):.3f} deg
Final cross-domain translational deviation D_trans: {mean('ekf_D_trans_final_m'):.4f} m

Definitions
-----------
D_rot is the sum of the absolute CCW and CW loop-rotation differences
between EKF and ground truth. D_trans is the Euclidean difference between
the final EKF and ground-truth displacement vectors. Ground truth is used
only for evaluation, never as controller feedback.
"""
    path.write_text(text, encoding="utf-8")


def create_zip(output, files):
    target = output / "V17_publication_results.zip"
    with zipfile.ZipFile(target, "w", zipfile.ZIP_DEFLATED) as archive:
        for file in files:
            archive.write(file, file.relative_to(output))
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags_directory")
    parser.add_argument("--output-directory", default="~/jackson_dt_ws/results_virtual/V17_figure8_ekf_closed_loop")
    parser.add_argument("--expected-runs", type=int, default=EXPECTED_RUNS)
    args = parser.parse_args()
    root = Path(os.path.expanduser(args.bags_directory)).resolve()
    output = Path(os.path.expanduser(args.output_directory)).resolve()
    output.mkdir(parents=True, exist_ok=True)
    trajectory_dir = output / "trajectories"; trajectory_dir.mkdir(exist_ok=True)

    candidates = bag_directory_candidates(root)
    runs = [run for run, _ in candidates]
    expected = [f"R{i:02d}" for i in range(1, args.expected_runs + 1)]
    if runs != expected:
        raise SystemExit(f"Expected {expected}, found {runs}. Resolve missing/duplicate runs first.")

    rows, analyses = [], []
    for run, bag in candidates:
        print(f"Processing {run}: {bag.name}")
        row, trajectories, crossover = analyze_run(run, bag)
        if not row["completed"]:
            raise RuntimeError(f"{run} did not contain the completed controller marker")
        rows.append(row); analyses.append((row, trajectories, crossover))
        save_trajectory_csv(trajectory_dir / f"{run}_trajectories.csv", trajectories)

    metrics_path = output / "V17_runs_metrics.csv"
    stats_path = output / "V17_statistics.csv"
    write_csv(metrics_path, rows)
    stats = statistics_rows(rows); write_csv(stats_path, stats)

    figures = [
        output / "trajectories_overlay.png", output / "endpoint_dispersion.png",
        output / "closure_error.png", output / "loop_diameters.png",
        output / "radial_rmse.png", output / "rotational_discrepancy.png",
        output / "cross_domain_translation.png",
    ]
    plot_trajectories(figures[0], analyses); endpoint_plot(figures[1], analyses)
    grouped_plot(figures[2], rows, ["gt_closure_error_m", "wheel_closure_error_m", "ekf_closure_error_m"],
                 ["Ground truth", "Wheel", "EKF"], "Closure error (m)", "Closure error by run")
    grouped_plot(figures[3], rows,
                 ["gt_ccw_x_diameter_m", "gt_ccw_y_diameter_m", "gt_cw_x_diameter_m", "gt_cw_y_diameter_m"],
                 ["CCW x", "CCW y", "CW x", "CW y"], "Diameter/span (m)", "Ground-truth loop diameters", 1.0)
    grouped_plot(figures[4], rows, ["gt_ccw_radial_rmse_m", "gt_cw_radial_rmse_m"],
                 ["CCW", "CW"], "Radial RMSE (m)", "Ground-truth radial tracking error")
    grouped_plot(figures[5], rows, ["wheel_D_rot_deg", "ekf_D_rot_deg"],
                 ["Wheel", "EKF"], "D_rot (deg)", "Cumulative rotational discrepancy")
    grouped_plot(figures[6], rows, ["wheel_D_trans_final_m", "ekf_D_trans_final_m"],
                 ["Wheel", "EKF"], "D_trans (m)", "Final cross-domain translational deviation")

    readme = output / "V17_results_summary.txt"; publication_readme(readme, rows, stats)
    zip_path = create_zip(output, [metrics_path, stats_path, readme, *figures])
    print("\nAnalysis completed")
    print(f"Run metrics: {metrics_path}")
    print(f"Statistics:  {stats_path}")
    print(f"Figures:     {output}")
    print(f"Publication: {zip_path}")


if __name__ == "__main__":
    main()
