#!/usr/bin/env python3

"""Analyze repeated 1 m square trajectories recorded in ROS 2 bags.

Expected topics:
  /cmd_vel  geometry_msgs/msg/Twist
  /odom     nav_msgs/msg/Odometry

The script crops each bag to the commanded-motion interval, normalizes the
odometry pose to the initial pose, computes repeatability metrics, and creates
CSV tables and publication-ready figures.
"""

import argparse
import bisect
import csv
import glob
import math
import os
import re
import statistics

import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock


T_CRITICAL_95_DF9 = 2.262157


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_trajectory(samples):
    if not samples:
        return []

    t0, x0, y0, yaw0 = samples[0]
    cosine = math.cos(yaw0)
    sine = math.sin(yaw0)
    normalized = []

    for timestamp, x, y, yaw in samples:
        dx = x - x0
        dy = y - y0
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        local_yaw = normalize_angle(yaw - yaw0)
        normalized.append(
            (timestamp - t0, local_x, local_y, local_yaw)
        )

    return normalized


def point_to_segment_distance(px, py, ax, ay, bx, by):
    ab_x = bx - ax
    ab_y = by - ay
    length_squared = ab_x * ab_x + ab_y * ab_y

    if length_squared <= 0.0:
        return math.hypot(px - ax, py - ay)

    projection = (
        (px - ax) * ab_x + (py - ay) * ab_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest_x = ax + projection * ab_x
    closest_y = ay + projection * ab_y
    return math.hypot(px - closest_x, py - closest_y)


def ideal_square_distance(x, y, side=1.0):
    segments = [
        (0.0, 0.0, side, 0.0),
        (side, 0.0, side, side),
        (side, side, 0.0, side),
        (0.0, side, 0.0, 0.0),
    ]
    return min(
        point_to_segment_distance(x, y, *segment)
        for segment in segments
    )


def interpolate_scalar(samples, timestamps, target_time):
    if not samples:
        return None

    index = bisect.bisect_left(timestamps, target_time)
    if index == 0:
        return samples[0][1]
    if index >= len(samples):
        return samples[-1][1]

    before = samples[index - 1]
    after = samples[index]
    duration = after[0] - before[0]
    if duration <= 0.0:
        return before[1]

    ratio = (target_time - before[0]) / duration
    return before[1] + ratio * (after[1] - before[1])


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    odom_samples = []
    motion_times = []
    clock_samples = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns * 1.0e-9

        if topic == "/cmd_vel":
            msg = deserialize_message(data, Twist)
            if (
                abs(msg.linear.x) > 1.0e-4
                or abs(msg.angular.z) > 1.0e-4
            ):
                motion_times.append(timestamp)

        elif topic == "/clock":
            msg = deserialize_message(data, Clock)
            simulation_time = (
                msg.clock.sec + msg.clock.nanosec * 1.0e-9
            )
            clock_samples.append((timestamp, simulation_time))

        elif topic == "/odom":
            msg = deserialize_message(data, Odometry)
            message_time = (
                msg.header.stamp.sec
                + msg.header.stamp.nanosec * 1.0e-9
            )
            odom_samples.append(
                (
                    timestamp,
                    message_time,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    quaternion_to_yaw(msg.pose.pose.orientation),
                )
            )

    if not motion_times:
        raise RuntimeError("No non-zero /cmd_vel messages found")
    if not odom_samples:
        raise RuntimeError("No /odom messages found")
    if not clock_samples:
        raise RuntimeError("No /clock messages found")

    motion_start = min(motion_times)
    motion_end = max(motion_times)
    window_start = motion_start - 0.5
    window_end = motion_end + 1.0
    window = [
        (message_time, x, y, yaw)
        for bag_time, message_time, x, y, yaw in odom_samples
        if window_start <= bag_time <= window_end
    ]

    if len(window) < 2:
        raise RuntimeError("Insufficient /odom samples in motion window")

    clock_samples.sort(key=lambda sample: sample[0])
    clock_timestamps = [sample[0] for sample in clock_samples]
    simulation_start = interpolate_scalar(
        clock_samples, clock_timestamps, motion_start
    )
    simulation_end = interpolate_scalar(
        clock_samples, clock_timestamps, motion_end
    )

    if simulation_start is None or simulation_end is None:
        raise RuntimeError("Unable to map motion window to simulation time")

    return (
        normalize_trajectory(window),
        simulation_end - simulation_start,
    )


def calculate_metrics(samples, motion_duration):
    steps = []
    squared_distances = []

    for index, (_, x, y, _) in enumerate(samples):
        distance = ideal_square_distance(x, y)
        squared_distances.append(distance * distance)

        if index > 0:
            previous = samples[index - 1]
            steps.append(
                math.hypot(x - previous[1], y - previous[2])
            )

    _, final_x, final_y, final_yaw = samples[-1]
    path_length = sum(steps)
    yaw_error_deg = math.degrees(normalize_angle(final_yaw))

    return {
        "samples": len(samples),
        "motion_duration_s": motion_duration,
        "closure_error_m": math.hypot(final_x, final_y),
        "yaw_error_deg": yaw_error_deg,
        "abs_yaw_error_deg": abs(yaw_error_deg),
        "path_length_m": path_length,
        "path_length_error_m": path_length - 4.0,
        "cross_track_rmse_m": math.sqrt(
            sum(squared_distances) / len(squared_distances)
        ),
        "maximum_cross_track_m": max(
            math.sqrt(value) for value in squared_distances
        ),
        "final_x_m": final_x,
        "final_y_m": final_y,
    }


def write_trajectory(path, samples):
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "x_m", "y_m", "yaw_rad", "yaw_deg"])
        for timestamp, x, y, yaw in samples:
            writer.writerow(
                [timestamp, x, y, yaw, math.degrees(yaw)]
            )


def write_run_metrics(path, rows):
    fields = [
        "run",
        "bag",
        "samples",
        "motion_duration_s",
        "closure_error_m",
        "yaw_error_deg",
        "abs_yaw_error_deg",
        "path_length_m",
        "path_length_error_m",
        "cross_track_rmse_m",
        "maximum_cross_track_m",
        "final_x_m",
        "final_y_m",
    ]
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def calculate_statistics(rows):
    metrics = [
        "motion_duration_s",
        "closure_error_m",
        "abs_yaw_error_deg",
        "path_length_m",
        "path_length_error_m",
        "cross_track_rmse_m",
        "maximum_cross_track_m",
    ]
    result = []

    for metric in metrics:
        values = [float(row[metric]) for row in rows]
        mean = statistics.mean(values)
        std = statistics.stdev(values) if len(values) > 1 else 0.0
        half_width = (
            T_CRITICAL_95_DF9 * std / math.sqrt(len(values))
            if len(values) == 10
            else float("nan")
        )
        cv = (
            100.0 * std / abs(mean)
            if abs(mean) > 1.0e-12
            else float("nan")
        )
        result.append(
            {
                "metric": metric,
                "n": len(values),
                "mean": mean,
                "std": std,
                "median": statistics.median(values),
                "minimum": min(values),
                "maximum": max(values),
                "cv_percent": cv,
                "ci95_lower": mean - half_width,
                "ci95_upper": mean + half_width,
            }
        )

    return result


def write_statistics(path, rows):
    fields = [
        "metric",
        "n",
        "mean",
        "std",
        "median",
        "minimum",
        "maximum",
        "cv_percent",
        "ci95_lower",
        "ci95_upper",
    ]
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def create_plots(output_dir, trajectories, rows, experiment_id):
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(
            "matplotlib is required: sudo apt install python3-matplotlib"
        ) from error

    runs = [row["run"] for row in rows]
    ideal_x = [0.0, 1.0, 1.0, 0.0, 0.0]
    ideal_y = [0.0, 0.0, 1.0, 1.0, 0.0]

    figure, axis = plt.subplots(figsize=(8, 8))
    axis.plot(ideal_x, ideal_y, "k--", linewidth=2.2, label="Ideal 1 x 1 m")
    for run, samples in trajectories.items():
        axis.plot(
            [sample[1] for sample in samples],
            [sample[2] for sample in samples],
            linewidth=1.0,
            alpha=0.70,
            label=run,
        )
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(
        f"Isaac Sim {experiment_id}: "
        f"{len(rows)} square trajectory run(s)"
    )
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=2, fontsize=8)
    figure.tight_layout()
    figure.savefig(
        os.path.join(output_dir, "trajectories_overlay.png"), dpi=220
    )
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7, 7))
    tolerance = plt.Circle(
        (0.0, 0.0), 0.03, fill=False, linestyle="--", color="black",
        label="3 cm tolerance",
    )
    axis.add_patch(tolerance)
    for row in rows:
        axis.scatter(row["final_x_m"], row["final_y_m"], s=45)
        axis.annotate(
            row["run"],
            (row["final_x_m"], row["final_y_m"]),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
        )
    axis.scatter(0.0, 0.0, marker="x", color="black", s=60, label="Start")
    axis.set_xlabel("Final x [m]")
    axis.set_ylabel("Final y [m]")
    axis.set_title("Final-position dispersion")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(
        os.path.join(output_dir, "endpoint_dispersion.png"), dpi=220
    )
    plt.close(figure)

    plot_specs = [
        ("closure_error_m", 100.0, "Closure error [cm]", "closure_error.png"),
        ("abs_yaw_error_deg", 1.0, "Absolute yaw error [deg]", "yaw_error.png"),
        ("cross_track_rmse_m", 100.0, "Nominal cross-track RMSE [cm]", "cross_track_rmse.png"),
        ("path_length_m", 1.0, "Path length [m]", "path_length.png"),
    ]

    for metric, scale, ylabel, filename in plot_specs:
        values = [float(row[metric]) * scale for row in rows]
        figure, axis = plt.subplots(figsize=(9, 5))
        axis.bar(runs, values, color="tab:blue", alpha=0.85)
        if metric == "path_length_m":
            axis.axhline(4.0, color="black", linestyle="--", label="Nominal 4 m")
            axis.legend()
        axis.set_xlabel("Run")
        axis.set_ylabel(ylabel)
        axis.set_title(f"Isaac Sim: {ylabel}")
        axis.grid(True, axis="y", alpha=0.3)
        figure.tight_layout()
        figure.savefig(os.path.join(output_dir, filename), dpi=220)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bags_directory",
        help="Directory containing Vxx_square_1m_Rxx_* bags",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output directory (default: <bags_directory>/../results_virtual/<experiment>_square_1m)",
    )
    parser.add_argument(
        "--expected-runs",
        type=int,
        default=10,
        help="Required number of bags (default: 10; use 1 for a pilot)",
    )
    args = parser.parse_args()

    bags_dir = os.path.abspath(os.path.expanduser(args.bags_directory))
    if not os.path.isdir(bags_dir):
        raise RuntimeError(f"Directory not found: {bags_dir}")

    candidates = glob.glob(
        os.path.join(bags_dir, "V??_square_1m_R??_*")
    )
    bags = []
    experiment_ids = set()
    for path in candidates:
        match = re.match(
            r"^(V\d{2})_square_1m_R(\d{2})_",
            os.path.basename(path),
        )
        if match and os.path.isfile(os.path.join(path, "metadata.yaml")):
            experiment_ids.add(match.group(1))
            bags.append((int(match.group(2)), match.group(2), path))
    bags.sort(key=lambda item: item[0])

    if len(experiment_ids) != 1:
        raise RuntimeError(
            "Expected bags from exactly one experiment id, found: "
            f"{sorted(experiment_ids)}"
        )

    experiment_id = next(iter(experiment_ids))

    if len(bags) != args.expected_runs:
        raise RuntimeError(
            f"Expected {args.expected_runs} valid bags, found {len(bags)}"
        )

    if args.output:
        output_dir = os.path.abspath(os.path.expanduser(args.output))
    else:
        workspace = os.path.dirname(bags_dir)
        output_dir = os.path.join(
            workspace,
            "results_virtual",
            f"{experiment_id}_square_1m",
        )

    trajectories_dir = os.path.join(output_dir, "trajectories")
    os.makedirs(trajectories_dir, exist_ok=True)

    trajectories = {}
    rows = []

    for _, run_number, bag_path in bags:
        run = f"R{run_number}"
        print(f"Processing {run}: {os.path.basename(bag_path)}")
        samples, motion_duration = read_bag(bag_path)
        metrics = calculate_metrics(samples, motion_duration)
        trajectories[run] = samples
        write_trajectory(
            os.path.join(trajectories_dir, f"{run}_trajectory.csv"),
            samples,
        )
        row = {
            "run": run,
            "bag": os.path.basename(bag_path),
        }
        row.update(metrics)
        rows.append(row)

    metrics_path = os.path.join(
        output_dir, f"{experiment_id}_runs_metrics.csv"
    )
    statistics_path = os.path.join(
        output_dir, f"{experiment_id}_statistics.csv"
    )
    write_run_metrics(metrics_path, rows)
    write_statistics(statistics_path, calculate_statistics(rows))
    create_plots(output_dir, trajectories, rows, experiment_id)

    print()
    print("Analysis completed")
    print(f"Run metrics: {metrics_path}")
    print(f"Statistics:  {statistics_path}")
    print(f"Figures:     {output_dir}")


if __name__ == "__main__":
    main()
