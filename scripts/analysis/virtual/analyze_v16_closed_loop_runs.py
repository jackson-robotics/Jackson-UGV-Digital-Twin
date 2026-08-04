#!/usr/bin/env python3

"""Batch analysis for Jackson V16 closed-loop square experiments.

The script reads ten ROS 2 bags and compares:
  * Isaac Sim ground truth: /ground_truth/odom
  * calibrated joint-state odometry: /wheel/odom
  * EKF estimate: /odometry/filtered

It writes per-run metrics, aggregate statistics, publication figures, and a
ZIP bundle suitable for sharing for report generation.
"""

import argparse
import bisect
import csv
import math
import os
import re
import statistics
import zipfile

import rosbag2_py

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message


TRAJECTORY_NAMES = ("ground_truth", "wheel", "ekf")


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(after, before):
    return math.atan2(
        math.sin(after - before),
        math.cos(after - before),
    )


def unwrap_yaws(samples):
    if not samples:
        return []

    result = [samples[0][3]]
    for index in range(1, len(samples)):
        result.append(
            result[-1]
            + angle_delta(samples[index][3], samples[index - 1][3])
        )
    return result


def normalize_trajectory(samples):
    """Express a trajectory in its initial planar coordinate frame."""
    if not samples:
        return []

    initial_x = samples[0][1]
    initial_y = samples[0][2]
    initial_yaw = samples[0][3]
    cosine = math.cos(initial_yaw)
    sine = math.sin(initial_yaw)
    unwrapped = unwrap_yaws(samples)
    normalized = []

    for sample, continuous_yaw in zip(samples, unwrapped):
        timestamp, x_value, y_value, _ = sample
        delta_x = x_value - initial_x
        delta_y = y_value - initial_y
        normalized.append(
            (
                timestamp,
                cosine * delta_x + sine * delta_y,
                -sine * delta_x + cosine * delta_y,
                continuous_yaw - initial_yaw,
            )
        )

    return normalized


def crop(samples, start_time, end_time):
    return [
        sample
        for sample in samples
        if start_time <= sample[0] <= end_time
    ]


def point_to_segment_distance(px, py, ax, ay, bx, by):
    segment_x = bx - ax
    segment_y = by - ay
    length_squared = segment_x * segment_x + segment_y * segment_y

    if length_squared <= 1.0e-15:
        return math.hypot(px - ax, py - ay)

    projection = (
        (px - ax) * segment_x + (py - ay) * segment_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    nearest_x = ax + projection * segment_x
    nearest_y = ay + projection * segment_y
    return math.hypot(px - nearest_x, py - nearest_y)


def square_cross_track_distances(samples):
    segments = (
        (0.0, 0.0, 1.0, 0.0),
        (1.0, 0.0, 1.0, 1.0),
        (1.0, 1.0, 0.0, 1.0),
        (0.0, 1.0, 0.0, 0.0),
    )
    distances = []

    for _, x_value, y_value, _ in samples:
        distances.append(
            min(
                point_to_segment_distance(
                    x_value, y_value, *segment
                )
                for segment in segments
            )
        )

    return distances


def trajectory_metrics(samples):
    if len(samples) < 2:
        raise RuntimeError("A trajectory contains fewer than two samples.")

    path_length = sum(
        math.hypot(
            current[1] - previous[1],
            current[2] - previous[2],
        )
        for previous, current in zip(samples[:-1], samples[1:])
    )
    final_x = samples[-1][1]
    final_y = samples[-1][2]
    closure = math.hypot(final_x, final_y)
    final_yaw_deg = math.degrees(samples[-1][3])
    cross_track = square_cross_track_distances(samples)

    return {
        "samples": len(samples),
        "path_length_m": path_length,
        "closure_error_m": closure,
        "final_x_m": final_x,
        "final_y_m": final_y,
        "yaw_error_deg": final_yaw_deg,
        "abs_yaw_error_deg": abs(final_yaw_deg),
        "cumulative_rotation_deg": abs(final_yaw_deg),
        "cross_track_rmse_m": math.sqrt(
            sum(value * value for value in cross_track)
            / len(cross_track)
        ),
        "maximum_cross_track_m": max(cross_track),
    }


def interpolate_pose(samples, timestamps, timestamp):
    if not samples or timestamp < timestamps[0] or timestamp > timestamps[-1]:
        return None

    right = bisect.bisect_left(timestamps, timestamp)
    if right == 0:
        return samples[0][1:4]
    if right == len(samples):
        return samples[-1][1:4]
    if timestamps[right] == timestamp:
        return samples[right][1:4]

    left = right - 1
    interval = timestamps[right] - timestamps[left]
    if interval <= 0.0:
        return samples[left][1:4]

    fraction = (timestamp - timestamps[left]) / interval
    return tuple(
        samples[left][index]
        + fraction * (samples[right][index] - samples[left][index])
        for index in (1, 2, 3)
    )


def estimator_error_metrics(ground_truth, estimator):
    timestamps = [sample[0] for sample in ground_truth]
    position_errors = []
    yaw_errors = []

    for timestamp, x_value, y_value, yaw_value in estimator:
        reference = interpolate_pose(ground_truth, timestamps, timestamp)
        if reference is None:
            continue
        gt_x, gt_y, gt_yaw = reference
        position_errors.append(math.hypot(x_value - gt_x, y_value - gt_y))
        yaw_errors.append(abs(math.degrees(yaw_value - gt_yaw)))

    if not position_errors:
        raise RuntimeError("No overlapping estimator and ground-truth samples.")

    return {
        "position_rmse_m": math.sqrt(
            sum(value * value for value in position_errors)
            / len(position_errors)
        ),
        "maximum_position_error_m": max(position_errors),
        "yaw_rmse_deg": math.sqrt(
            sum(value * value for value in yaw_errors)
            / len(yaw_errors)
        ),
        "maximum_yaw_error_deg": max(yaw_errors),
    }


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    trajectories = {name: [] for name in TRAJECTORY_NAMES}
    active_command_times = []
    log_messages = []

    topic_to_name = {
        "/ground_truth/odom": "ground_truth",
        "/wheel/odom": "wheel",
        "/odometry/filtered": "ekf",
    }

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns * 1.0e-9

        if topic == "/cmd_vel":
            message = deserialize_message(data, Twist)
            if (
                abs(message.linear.x) > 1.0e-4
                or abs(message.angular.z) > 1.0e-4
            ):
                active_command_times.append(timestamp)
        elif topic in topic_to_name:
            message = deserialize_message(data, Odometry)
            trajectories[topic_to_name[topic]].append(
                (
                    timestamp,
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw_from_quaternion(message.pose.pose.orientation),
                )
            )
        elif topic == "/rosout":
            message = deserialize_message(data, Log)
            log_messages.append(message.msg)

    if not active_command_times:
        raise RuntimeError(f"No active /cmd_vel samples in {bag_path}")

    active_start = min(active_command_times)
    active_end = max(active_command_times)
    crop_start = active_start - 0.25
    crop_end = active_end + 2.0

    normalized = {}
    for name, samples in trajectories.items():
        window = crop(samples, crop_start, crop_end)
        if len(window) < 2:
            raise RuntimeError(f"Missing {name} trajectory in {bag_path}")
        normalized[name] = normalize_trajectory(window)

    completed = any("CUADRADO TERMINADO" in text for text in log_messages)
    closure_log = None
    yaw_log = None
    for text in log_messages:
        closure_match = re.search(r"Error de cierre EKF:\s*([-+0-9.eE]+)", text)
        yaw_match = re.search(r"Error final de yaw EKF:\s*([-+0-9.eE]+)", text)
        if closure_match:
            closure_log = float(closure_match.group(1))
        if yaw_match:
            yaw_log = float(yaw_match.group(1))

    return {
        "trajectories": normalized,
        "motion_duration_s": active_end - active_start,
        "completed": completed,
        "controller_closure_m": closure_log,
        "controller_yaw_error_deg": yaw_log,
    }


def discover_bags(root, expected_runs):
    expression = re.compile(
        r"^V16_square_ekf_closed_loop_(R\d{2})_\d{8}_\d{6}$"
    )
    discovered = {}

    for entry in os.scandir(root):
        if not entry.is_dir():
            continue
        match = expression.match(entry.name)
        if not match:
            continue
        if not os.path.isfile(os.path.join(entry.path, "metadata.yaml")):
            continue
        run = match.group(1)
        if run in discovered:
            raise RuntimeError(f"Duplicate {run}: {discovered[run]} and {entry.path}")
        discovered[run] = entry.path

    expected = [f"R{index:02d}" for index in range(1, expected_runs + 1)]
    missing = [run for run in expected if run not in discovered]
    unexpected = sorted(run for run in discovered if run not in expected)
    if missing or unexpected:
        raise RuntimeError(
            f"Run-set mismatch. Missing={missing}; unexpected={unexpected}"
        )
    return [(run, discovered[run]) for run in expected]


def flatten_run_metrics(run, bag_path, analysis):
    row = {
        "run": run,
        "bag": os.path.basename(bag_path),
        "completed": analysis["completed"],
        "motion_duration_s": analysis["motion_duration_s"],
        "controller_closure_m": analysis["controller_closure_m"],
        "controller_yaw_error_deg": analysis["controller_yaw_error_deg"],
    }
    all_metrics = {}

    for name in TRAJECTORY_NAMES:
        metrics = trajectory_metrics(analysis["trajectories"][name])
        all_metrics[name] = metrics
        for key, value in metrics.items():
            row[f"{name}_{key}"] = value

    for name in ("wheel", "ekf"):
        errors = estimator_error_metrics(
            analysis["trajectories"]["ground_truth"],
            analysis["trajectories"][name],
        )
        for key, value in errors.items():
            row[f"{name}_{key}"] = value

    row["ekf_closure_difference_m"] = (
        all_metrics["ekf"]["closure_error_m"]
        - all_metrics["ground_truth"]["closure_error_m"]
    )
    row["ekf_path_length_ratio"] = (
        all_metrics["ekf"]["path_length_m"]
        / all_metrics["ground_truth"]["path_length_m"]
    )
    row["ekf_rotation_ratio"] = (
        all_metrics["ekf"]["cumulative_rotation_deg"]
        / all_metrics["ground_truth"]["cumulative_rotation_deg"]
    )
    return row


def write_csv(path, rows, fields=None):
    if not rows:
        return
    if fields is None:
        fields = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def critical_t_95(n):
    table = {
        2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776,
        6: 2.571, 7: 2.447, 8: 2.365, 9: 2.306,
        10: 2.262, 11: 2.228, 12: 2.201, 13: 2.179,
        14: 2.160, 15: 2.145, 16: 2.131, 17: 2.120,
        18: 2.110, 19: 2.101, 20: 2.093,
    }
    return table.get(n, 1.96)


def aggregate_statistics(rows):
    excluded = {"run", "bag", "completed"}
    statistics_rows = []

    for field in rows[0]:
        if field in excluded or field.endswith("_samples"):
            continue
        values = [
            float(row[field])
            for row in rows
            if row[field] is not None and row[field] != ""
        ]
        if not values:
            continue
        count = len(values)
        mean = statistics.fmean(values)
        deviation = statistics.stdev(values) if count > 1 else 0.0
        margin = (
            critical_t_95(count) * deviation / math.sqrt(count)
            if count > 1 else 0.0
        )
        coefficient = (
            100.0 * deviation / abs(mean)
            if abs(mean) > 1.0e-15 else float("nan")
        )
        statistics_rows.append(
            {
                "metric": field,
                "n": count,
                "mean": mean,
                "std": deviation,
                "median": statistics.median(values),
                "minimum": min(values),
                "maximum": max(values),
                "cv_percent": coefficient,
                "ci95_lower": mean - margin,
                "ci95_upper": mean + margin,
            }
        )
    return statistics_rows


def configure_plotting():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def plot_trajectories(path, analyses):
    plt = configure_plotting()
    figure, axes = plt.subplots(1, 3, figsize=(15, 5))
    ideal_x = [0.0, 1.0, 1.0, 0.0, 0.0]
    ideal_y = [0.0, 0.0, 1.0, 1.0, 0.0]
    titles = {
        "ground_truth": "Ground truth",
        "wheel": "Calibrated wheel odometry",
        "ekf": "EKF estimate",
    }

    for axis, name in zip(axes, TRAJECTORY_NAMES):
        axis.plot(ideal_x, ideal_y, "k--", linewidth=2.0, label="Ideal 1 m square")
        for run, analysis in analyses:
            samples = analysis["trajectories"][name]
            axis.plot(
                [sample[1] for sample in samples],
                [sample[2] for sample in samples],
                linewidth=1.0,
                alpha=0.65,
                label=run if name == "ground_truth" else None,
            )
        axis.set_title(titles[name])
        axis.set_xlabel("x [m]")
        axis.axis("equal")
        axis.grid(True, alpha=0.3)
    axes[0].set_ylabel("y [m]")
    axes[0].legend(fontsize=7, ncol=2)
    figure.suptitle("V16 closed-loop square trajectories (N = 10)")
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_endpoints(path, analyses):
    plt = configure_plotting()
    figure, axis = plt.subplots(figsize=(7, 7))
    colors = {"ground_truth": "tab:green", "wheel": "tab:orange", "ekf": "tab:blue"}
    labels = {"ground_truth": "Ground truth", "wheel": "Wheel odometry", "ekf": "EKF"}

    for name in TRAJECTORY_NAMES:
        points = [analysis["trajectories"][name][-1] for _, analysis in analyses]
        axis.scatter(
            [point[1] for point in points],
            [point[2] for point in points],
            s=55,
            alpha=0.8,
            color=colors[name],
            label=labels[name],
        )
        for (run, _), point in zip(analyses, points):
            axis.annotate(run[1:], (point[1], point[2]), fontsize=7)

    axis.scatter([0.0], [0.0], marker="x", s=100, color="black", label="Start")
    axis.set_xlabel("Final x [m]")
    axis.set_ylabel("Final y [m]")
    axis.set_title("V16 endpoint dispersion")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_run_metrics(path, rows):
    plt = configure_plotting()
    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    runs = [row["run"] for row in rows]
    x_values = list(range(len(runs)))

    panels = (
        ("ground_truth_closure_error_m", "GT closure error", "m"),
        ("ekf_closure_error_m", "EKF closure error", "m"),
        ("ekf_position_rmse_m", "EKF position RMSE vs GT", "m"),
        ("ekf_yaw_rmse_deg", "EKF yaw RMSE vs GT", "deg"),
    )
    for axis, (field, title, unit) in zip(axes.flat, panels):
        axis.bar(x_values, [row[field] for row in rows], color="tab:blue", alpha=0.8)
        axis.set_xticks(x_values, runs)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.grid(True, axis="y", alpha=0.3)
    figure.suptitle("V16 per-run validation metrics")
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def plot_path_rotation(path, rows):
    plt = configure_plotting()
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    runs = [row["run"] for row in rows]
    x_values = list(range(len(runs)))
    width = 0.25
    colors = {"ground_truth": "tab:green", "wheel": "tab:orange", "ekf": "tab:blue"}

    for offset, name in zip((-width, 0.0, width), TRAJECTORY_NAMES):
        axes[0].bar(
            [value + offset for value in x_values],
            [row[f"{name}_path_length_m"] for row in rows],
            width=width,
            label=name.replace("_", " ").title(),
            color=colors[name],
        )
        axes[1].bar(
            [value + offset for value in x_values],
            [row[f"{name}_cumulative_rotation_deg"] for row in rows],
            width=width,
            label=name.replace("_", " ").title(),
            color=colors[name],
        )

    axes[0].axhline(4.0, color="black", linestyle="--", linewidth=1.2)
    axes[1].axhline(360.0, color="black", linestyle="--", linewidth=1.2)
    axes[0].set_title("Path length")
    axes[0].set_ylabel("m")
    axes[1].set_title("Cumulative rotation")
    axes[1].set_ylabel("deg")
    for axis in axes:
        axis.set_xticks(x_values, runs)
        axis.grid(True, axis="y", alpha=0.3)
    axes[0].legend(fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def create_bundle(output_directory, files):
    bundle = os.path.join(output_directory, "V16_publication_results.zip")
    with zipfile.ZipFile(bundle, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for filename in files:
            full_path = os.path.join(output_directory, filename)
            archive.write(full_path, arcname=filename)
    return bundle


def main():
    parser = argparse.ArgumentParser(
        description="Analyze ten Jackson V16 EKF closed-loop square bags."
    )
    parser.add_argument("bag_root", help="Directory containing V16 bag folders")
    parser.add_argument("--expected-runs", type=int, default=10)
    parser.add_argument(
        "--output",
        default=os.path.expanduser(
            "~/jackson_dt_ws/results_virtual/V16_square_ekf_closed_loop"
        ),
    )
    args = parser.parse_args()

    bag_root = os.path.abspath(os.path.expanduser(args.bag_root))
    output_directory = os.path.abspath(os.path.expanduser(args.output))
    if not os.path.isdir(bag_root):
        raise RuntimeError(f"Bag root does not exist: {bag_root}")
    os.makedirs(output_directory, exist_ok=True)

    bags = discover_bags(bag_root, args.expected_runs)
    analyses = []
    rows = []

    for run, bag_path in bags:
        print(f"Processing {run}: {os.path.basename(bag_path)}")
        analysis = read_bag(bag_path)
        if not analysis["completed"]:
            raise RuntimeError(f"{run} does not contain CUADRADO TERMINADO")
        analyses.append((run, analysis))
        rows.append(flatten_run_metrics(run, bag_path, analysis))

    metrics_name = "V16_runs_metrics.csv"
    statistics_name = "V16_statistics.csv"
    trajectories_name = "V16_trajectories_overlay.png"
    endpoints_name = "V16_endpoint_dispersion.png"
    run_metrics_name = "V16_validation_metrics.png"
    path_rotation_name = "V16_path_length_rotation.png"

    write_csv(os.path.join(output_directory, metrics_name), rows)
    write_csv(
        os.path.join(output_directory, statistics_name),
        aggregate_statistics(rows),
    )
    plot_trajectories(os.path.join(output_directory, trajectories_name), analyses)
    plot_endpoints(os.path.join(output_directory, endpoints_name), analyses)
    plot_run_metrics(os.path.join(output_directory, run_metrics_name), rows)
    plot_path_rotation(os.path.join(output_directory, path_rotation_name), rows)

    output_files = [
        metrics_name,
        statistics_name,
        trajectories_name,
        endpoints_name,
        run_metrics_name,
        path_rotation_name,
    ]
    bundle = create_bundle(output_directory, output_files)

    gt_closures = [row["ground_truth_closure_error_m"] for row in rows]
    ekf_closures = [row["ekf_closure_error_m"] for row in rows]
    position_rmse = [row["ekf_position_rmse_m"] for row in rows]
    yaw_rmse = [row["ekf_yaw_rmse_deg"] for row in rows]

    print("\nAnalysis completed successfully")
    print(f"Runs: {len(rows)} / {args.expected_runs}")
    print(f"Mean GT closure:  {statistics.fmean(gt_closures):.4f} m")
    print(f"Mean EKF closure: {statistics.fmean(ekf_closures):.4f} m")
    print(f"Mean EKF position RMSE vs GT: {statistics.fmean(position_rmse):.4f} m")
    print(f"Mean EKF yaw RMSE vs GT:      {statistics.fmean(yaw_rmse):.3f} deg")
    print(f"Results: {output_directory}")
    print(f"Upload this file for the English report: {bundle}")


if __name__ == "__main__":
    main()
