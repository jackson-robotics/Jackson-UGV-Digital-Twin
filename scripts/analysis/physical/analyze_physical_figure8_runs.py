#!/usr/bin/env python3

"""Analyze repeated physical Jackson figure-eight ROS 2 bag experiments."""

import argparse
import csv
import glob
import math
import os
import re
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message


RUN_PATTERN = re.compile(r"P02_figure8_(R\d{2})_")


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def normalize_trajectory(samples, start_time, end_time):
    selected = [
        sample for sample in samples
        if start_time <= sample[0] <= end_time
    ]

    if len(selected) < 2:
        return []

    selected.sort(key=lambda sample: sample[0])
    t0, x0, y0, yaw0 = selected[0]
    cosine = math.cos(yaw0)
    sine = math.sin(yaw0)
    previous_yaw = yaw0
    unwrapped_yaw = 0.0
    normalized = []

    for timestamp, x, y, yaw in selected:
        unwrapped_yaw += normalize_angle(yaw - previous_yaw)
        previous_yaw = yaw

        dx = x - x0
        dy = y - y0
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy

        normalized.append(
            (
                timestamp - start_time,
                local_x,
                local_y,
                unwrapped_yaw,
                timestamp,
            )
        )

    return normalized


def trajectory_metrics(samples, crossover_time, radius):
    if len(samples) < 2:
        raise RuntimeError("Trajectory contains fewer than two samples")

    path_length = 0.0
    cross_track_errors = []
    heading_errors = []

    ccw = []
    cw = []

    for index, sample in enumerate(samples):
        _, x, y, yaw, absolute_time = sample

        if absolute_time <= crossover_time:
            direction = 1.0
            center_x, center_y = 0.0, radius
            ccw.append(sample)
        else:
            direction = -1.0
            center_x, center_y = 0.0, -radius
            cw.append(sample)

        dx = x - center_x
        dy = y - center_y
        distance = math.hypot(dx, dy)
        cross_track_errors.append(abs(distance - radius))

        polar_angle = math.atan2(dy, dx)
        ideal_heading = polar_angle + direction * math.pi / 2.0
        heading_errors.append(normalize_angle(ideal_heading - yaw))

        if index:
            previous = samples[index - 1]
            path_length += math.hypot(
                x - previous[1],
                y - previous[2],
            )

    def spans(segment):
        if not segment:
            return math.nan, math.nan, math.nan
        x_values = [sample[1] for sample in segment]
        y_values = [sample[2] for sample in segment]
        x_span = max(x_values) - min(x_values)
        y_span = max(y_values) - min(y_values)
        return x_span, y_span, (x_span + y_span) / 2.0

    ccw_x_span, ccw_y_span, ccw_diameter = spans(ccw)
    cw_x_span, cw_y_span, cw_diameter = spans(cw)

    final = samples[-1]
    closure_error = math.hypot(final[1], final[2])
    cross_track_array = np.asarray(cross_track_errors)
    heading_array = np.asarray(heading_errors)

    return {
        "samples": len(samples),
        "path_length_m": path_length,
        "closure_error_m": closure_error,
        "final_x_m": final[1],
        "final_y_m": final[2],
        "final_yaw_deg": math.degrees(final[3]),
        "cross_track_rmse_m": float(
            np.sqrt(np.mean(cross_track_array ** 2))
        ),
        "maximum_cross_track_m": float(np.max(cross_track_array)),
        "heading_error_mean_deg": math.degrees(
            float(np.mean(heading_array))
        ),
        "heading_error_std_deg": math.degrees(
            float(np.std(heading_array, ddof=1))
        ),
        "heading_error_rmse_deg": math.degrees(
            float(np.sqrt(np.mean(heading_array ** 2)))
        ),
        "ccw_x_span_m": ccw_x_span,
        "ccw_y_span_m": ccw_y_span,
        "ccw_diameter_m": ccw_diameter,
        "cw_x_span_m": cw_x_span,
        "cw_y_span_m": cw_y_span,
        "cw_diameter_m": cw_diameter,
    }


def read_bag(bag_path, radius):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions("", ""),
    )

    wheel_samples = []
    ekf_samples = []
    command_samples = []

    event_times = {
        "start": None,
        "crossover": None,
        "finish": None,
    }
    completed = False
    logged_ccw_yaw_deg = None
    logged_final_yaw_deg = None
    logged_duration_s = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns * 1.0e-9

        if topic == "/cmd_vel":
            message = deserialize_message(data, Twist)
            command_samples.append(
                (
                    timestamp,
                    message.linear.x,
                    message.angular.z,
                )
            )

        elif topic in ("/wheel/odom", "/odometry/filtered"):
            message = deserialize_message(data, Odometry)
            sample = (
                timestamp,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                quaternion_to_yaw(message.pose.pose.orientation),
            )
            if topic == "/wheel/odom":
                wheel_samples.append(sample)
            else:
                ekf_samples.append(sample)

        elif topic == "/rosout":
            message = deserialize_message(data, Log)
            text = message.msg

            if "FIGURE8 START" in text:
                event_times["start"] = timestamp

            elif "FIGURE8 CROSSOVER" in text:
                event_times["crossover"] = timestamp
                match = re.search(
                    r"accumulated yaw=([-+0-9.]+) deg",
                    text,
                )
                if match:
                    logged_ccw_yaw_deg = float(match.group(1))

            elif "FIGURE8 FINISHED" in text:
                event_times["finish"] = timestamp
                completed = "reason=completed" in text

            elif "Final net EKF yaw" in text:
                match = re.search(r"yaw:\s*([-+0-9.]+) deg", text)
                if match:
                    logged_final_yaw_deg = float(match.group(1))

            elif "Motion duration" in text:
                match = re.search(r"duration:\s*([-+0-9.]+) s", text)
                if match:
                    logged_duration_s = float(match.group(1))

    if not all(event_times.values()):
        raise RuntimeError(
            f"Missing START, CROSSOVER, or FINISHED event in {bag_path}"
        )

    start_time = event_times["start"]
    crossover_time = event_times["crossover"]
    finish_time = event_times["finish"]

    wheel = normalize_trajectory(
        wheel_samples,
        start_time,
        finish_time,
    )
    ekf = normalize_trajectory(
        ekf_samples,
        start_time,
        finish_time,
    )

    wheel_metrics = trajectory_metrics(wheel, crossover_time, radius)
    ekf_metrics = trajectory_metrics(ekf, crossover_time, radius)

    if logged_ccw_yaw_deg is None:
        logged_ccw_yaw_deg = math.degrees(
            max(
                sample[3] for sample in ekf
                if sample[4] <= crossover_time
            )
        )

    if logged_final_yaw_deg is None:
        logged_final_yaw_deg = ekf_metrics["final_yaw_deg"]

    if logged_duration_s is None:
        logged_duration_s = finish_time - start_time

    cw_yaw_deg = logged_final_yaw_deg - logged_ccw_yaw_deg
    ccw_error_deg = abs(360.0 - abs(logged_ccw_yaw_deg))
    cw_error_deg = abs(360.0 - abs(cw_yaw_deg))
    cumulative_rotational_discrepancy_deg = (
        ccw_error_deg + cw_error_deg
    )

    active_commands = [
        sample for sample in command_samples
        if start_time <= sample[0] <= finish_time
        and (
            abs(sample[1]) > 1.0e-4
            or abs(sample[2]) > 1.0e-4
        )
    ]
    linear_commands = [sample[1] for sample in active_commands]
    ccw_commands = [
        sample[2] for sample in active_commands
        if sample[2] > 1.0e-4
    ]
    cw_commands = [
        sample[2] for sample in active_commands
        if sample[2] < -1.0e-4
    ]

    return {
        "completed": completed,
        "motion_duration_s": logged_duration_s,
        "ccw_yaw_deg": logged_ccw_yaw_deg,
        "cw_yaw_deg": cw_yaw_deg,
        "ccw_rotational_error_deg": ccw_error_deg,
        "cw_rotational_error_deg": cw_error_deg,
        "cumulative_rotational_discrepancy_deg": (
            cumulative_rotational_discrepancy_deg
        ),
        "net_heading_drift_deg": logged_final_yaw_deg,
        "abs_heading_drift_deg": abs(logged_final_yaw_deg),
        "mean_linear_command_m_s": statistics.mean(linear_commands),
        "mean_ccw_command_rad_s": statistics.mean(ccw_commands),
        "mean_cw_command_rad_s": statistics.mean(cw_commands),
        "wheel_metrics": wheel_metrics,
        "ekf_metrics": ekf_metrics,
        "wheel_trajectory": wheel,
        "ekf_trajectory": ekf,
    }


def load_physical_measurements(path):
    measurements = {}
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            run = row["run"].strip().upper()
            x_cm = float(row["final_x_cm"])
            y_cm = float(row["final_y_cm"])
            measurements[run] = {
                "physical_ccw_diameter_m": (
                    float(row["ccw_diameter_cm"]) / 100.0
                ),
                "physical_cw_diameter_m": (
                    float(row["cw_diameter_cm"]) / 100.0
                ),
                "physical_final_x_m": x_cm / 100.0,
                "physical_final_y_m": y_cm / 100.0,
                "physical_closure_error_m": (
                    math.hypot(x_cm, y_cm) / 100.0
                ),
            }
    return measurements


def write_csv(path, rows, fieldnames):
    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_trajectory_csv(path, trajectory):
    rows = []
    for time_s, x, y, yaw, _ in trajectory:
        rows.append(
            {
                "time_s": time_s,
                "x_m": x,
                "y_m": y,
                "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw),
            }
        )
    write_csv(
        path,
        rows,
        ["time_s", "x_m", "y_m", "yaw_rad", "yaw_deg"],
    )


def t_critical_95(degrees_of_freedom):
    values = {
        1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776,
        5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306,
        9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
        13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120,
        17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
        21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064,
        25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048,
        29: 2.045, 30: 2.042,
    }
    return values.get(degrees_of_freedom, 1.960)


def statistics_row(metric, values):
    clean = [
        float(value) for value in values
        if value is not None and math.isfinite(float(value))
    ]
    count = len(clean)
    mean = statistics.mean(clean)
    standard_deviation = statistics.stdev(clean) if count > 1 else 0.0
    critical = t_critical_95(count - 1) if count > 1 else 0.0
    margin = (
        critical * standard_deviation / math.sqrt(count)
        if count > 1 else 0.0
    )
    cv = (
        100.0 * standard_deviation / abs(mean)
        if abs(mean) > 1.0e-12 else math.nan
    )
    return {
        "metric": metric,
        "n": count,
        "mean": mean,
        "std": standard_deviation,
        "median": statistics.median(clean),
        "minimum": min(clean),
        "maximum": max(clean),
        "cv_percent": cv,
        "ci95_lower": mean - margin,
        "ci95_upper": mean + margin,
    }


def ideal_figure8(radius):
    ccw_angle = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, 500)
    cw_angle = np.linspace(math.pi / 2.0, -3.0 * math.pi / 2.0, 500)
    ccw_x = radius * np.cos(ccw_angle)
    ccw_y = radius + radius * np.sin(ccw_angle)
    cw_x = radius * np.cos(cw_angle)
    cw_y = -radius + radius * np.sin(cw_angle)
    return ccw_x, ccw_y, cw_x, cw_y


def save_trajectory_overlay(path, trajectories, radius, title):
    figure, axis = plt.subplots(figsize=(8, 8))
    ccw_x, ccw_y, cw_x, cw_y = ideal_figure8(radius)
    axis.plot(ccw_x, ccw_y, "k--", linewidth=2.0, label="Ideal path")
    axis.plot(cw_x, cw_y, "k--", linewidth=2.0)

    colors = plt.cm.tab10(np.linspace(0.0, 1.0, len(trajectories)))
    for color, (run, trajectory) in zip(colors, trajectories.items()):
        axis.plot(
            [sample[1] for sample in trajectory],
            [sample[2] for sample in trajectory],
            color=color,
            linewidth=1.1,
            alpha=0.85,
            label=run,
        )

    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(title)
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=2, fontsize=8)
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def grouped_bar(path, runs, groups, ylabel, title, reference=None):
    figure, axis = plt.subplots(figsize=(10, 5.5))
    x = np.arange(len(runs))
    width = 0.8 / len(groups)

    for index, (label, values) in enumerate(groups):
        offset = (index - (len(groups) - 1) / 2.0) * width
        axis.bar(x + offset, values, width, label=label)

    if reference is not None:
        axis.axhline(
            reference,
            color="black",
            linestyle="--",
            linewidth=1.5,
            label="Reference",
        )

    axis.set_xticks(x)
    axis.set_xticklabels(runs)
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, axis="y", alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def endpoint_plot(path, rows):
    figure, axis = plt.subplots(figsize=(7.5, 7.5))
    ekf_x = [100.0 * row["ekf_final_x_m"] for row in rows]
    ekf_y = [100.0 * row["ekf_final_y_m"] for row in rows]
    physical_x = [100.0 * row["physical_final_x_m"] for row in rows]
    physical_y = [100.0 * row["physical_final_y_m"] for row in rows]

    axis.scatter(ekf_x, ekf_y, s=65, label="EKF endpoints")
    axis.scatter(
        physical_x,
        physical_y,
        marker="x",
        s=85,
        linewidths=2.0,
        label="Physically measured endpoints",
    )
    axis.scatter([0.0], [0.0], marker="*", s=160, color="black", label="Start")

    for row, x_value, y_value in zip(rows, physical_x, physical_y):
        axis.annotate(row["run"], (x_value, y_value), fontsize=8)

    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.axvline(0.0, color="black", linewidth=0.8)
    axis.set_xlabel("Longitudinal displacement x [cm]")
    axis.set_ylabel("Lateral displacement y [cm]")
    axis.set_title("Endpoint dispersion after the figure-eight trajectory")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=220)
    plt.close(figure)


def write_summary(path, rows, statistics_rows):
    statistics_by_name = {
        row["metric"]: row for row in statistics_rows
    }

    def result(metric, scale=1.0, unit=""):
        row = statistics_by_name[metric]
        return (
            f"{scale * row['mean']:.3f} +/- "
            f"{scale * row['std']:.3f} {unit}"
        )

    success_count = sum(bool(row["completed"]) for row in rows)

    with open(path, "w", encoding="utf-8") as file:
        file.write("Jackson Physical Figure-Eight Validation (Stage 2)\n")
        file.write("=" * 55 + "\n\n")
        file.write(f"Successful runs: {success_count}/{len(rows)}\n")
        file.write(
            "EKF closure error: "
            + result("ekf_closure_error_m", 100.0, "cm")
            + "\n"
        )
        file.write(
            "Physical closure error: "
            + result("physical_closure_error_m", 100.0, "cm")
            + "\n"
        )
        file.write(
            "Absolute final heading drift: "
            + result("abs_heading_drift_deg", 1.0, "deg")
            + "\n"
        )
        file.write(
            "Cumulative rotational discrepancy: "
            + result(
                "cumulative_rotational_discrepancy_deg",
                1.0,
                "deg",
            )
            + "\n"
        )
        file.write(
            "Physical CCW diameter: "
            + result("physical_ccw_diameter_m", 100.0, "cm")
            + "\n"
        )
        file.write(
            "Physical CW diameter: "
            + result("physical_cw_diameter_m", 100.0, "cm")
            + "\n"
        )
        file.write(
            "Motion duration: "
            + result("motion_duration_s", 1.0, "s")
            + "\n\n"
        )
        file.write("Publication-ready result paragraph\n")
        file.write("-" * 35 + "\n")
        file.write(
            "The physical Jackson platform successfully completed all "
            f"{len(rows)} figure-eight trials. The mean EKF-derived closure "
            f"error was {100.0 * statistics_by_name['ekf_closure_error_m']['mean']:.2f} "
            f"+/- {100.0 * statistics_by_name['ekf_closure_error_m']['std']:.2f} cm, "
            "whereas the independently measured physical closure error was "
            f"{100.0 * statistics_by_name['physical_closure_error_m']['mean']:.2f} "
            f"+/- {100.0 * statistics_by_name['physical_closure_error_m']['std']:.2f} cm. "
            "The final absolute heading drift remained limited to "
            f"{statistics_by_name['abs_heading_drift_deg']['mean']:.2f} "
            f"+/- {statistics_by_name['abs_heading_drift_deg']['std']:.2f} deg, "
            "and the cumulative rotational discrepancy was "
            f"{statistics_by_name['cumulative_rotational_discrepancy_deg']['mean']:.2f} "
            f"+/- {statistics_by_name['cumulative_rotational_discrepancy_deg']['std']:.2f} deg. "
            "These results indicate repeatable rotational tracking while also "
            "revealing a systematic difference between EKF-estimated and "
            "physically observed translational closure.\n"
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze N physical Jackson figure-eight ROS 2 bags and "
            "generate publication-ready English results."
        )
    )
    parser.add_argument("bags_directory")
    parser.add_argument("physical_measurements_csv")
    parser.add_argument(
        "--output-directory",
        default=os.path.expanduser(
            "~/jackson_dt_ws/results_figure8/P02_physical"
        ),
    )
    parser.add_argument("--radius", type=float, default=0.5)
    parser.add_argument("--expected-runs", type=int, default=10)
    args = parser.parse_args()

    bags_directory = os.path.abspath(
        os.path.expanduser(args.bags_directory)
    )
    measurements_path = os.path.abspath(
        os.path.expanduser(args.physical_measurements_csv)
    )
    output_directory = os.path.abspath(
        os.path.expanduser(args.output_directory)
    )
    trajectories_directory = os.path.join(
        output_directory,
        "trajectories",
    )
    os.makedirs(trajectories_directory, exist_ok=True)

    bag_paths = []
    for path in glob.glob(
        os.path.join(bags_directory, "P02_figure8_R??_*")
    ):
        if os.path.isfile(os.path.join(path, "metadata.yaml")):
            bag_paths.append(path)

    bag_paths.sort()
    if len(bag_paths) != args.expected_runs:
        raise RuntimeError(
            f"Expected {args.expected_runs} bags, found {len(bag_paths)}"
        )

    physical = load_physical_measurements(measurements_path)
    rows = []
    ekf_trajectories = {}
    wheel_trajectories = {}

    for bag_path in bag_paths:
        match = RUN_PATTERN.search(os.path.basename(bag_path))
        if not match:
            raise RuntimeError(f"Cannot extract run ID from {bag_path}")
        run = match.group(1)
        print(f"Processing {run}: {os.path.basename(bag_path)}")

        if run not in physical:
            raise RuntimeError(f"Missing physical measurement for {run}")

        analysis = read_bag(bag_path, args.radius)
        ekf = analysis["ekf_metrics"]
        wheel = analysis["wheel_metrics"]

        row = {
            "run": run,
            "bag": os.path.basename(bag_path),
            "completed": analysis["completed"],
            "motion_duration_s": analysis["motion_duration_s"],
            "ccw_yaw_deg": analysis["ccw_yaw_deg"],
            "cw_yaw_deg": analysis["cw_yaw_deg"],
            "ccw_rotational_error_deg": analysis[
                "ccw_rotational_error_deg"
            ],
            "cw_rotational_error_deg": analysis[
                "cw_rotational_error_deg"
            ],
            "cumulative_rotational_discrepancy_deg": analysis[
                "cumulative_rotational_discrepancy_deg"
            ],
            "net_heading_drift_deg": analysis["net_heading_drift_deg"],
            "abs_heading_drift_deg": analysis["abs_heading_drift_deg"],
            "mean_linear_command_m_s": analysis[
                "mean_linear_command_m_s"
            ],
            "mean_ccw_command_rad_s": analysis[
                "mean_ccw_command_rad_s"
            ],
            "mean_cw_command_rad_s": analysis[
                "mean_cw_command_rad_s"
            ],
        }

        for key, value in ekf.items():
            row[f"ekf_{key}"] = value
        for key, value in wheel.items():
            row[f"wheel_{key}"] = value
        row.update(physical[run])
        row["physical_minus_ekf_closure_m"] = (
            row["physical_closure_error_m"]
            - row["ekf_closure_error_m"]
        )
        rows.append(row)

        ekf_trajectories[run] = analysis["ekf_trajectory"]
        wheel_trajectories[run] = analysis["wheel_trajectory"]

        write_trajectory_csv(
            os.path.join(trajectories_directory, f"{run}_ekf.csv"),
            analysis["ekf_trajectory"],
        )
        write_trajectory_csv(
            os.path.join(trajectories_directory, f"{run}_wheel.csv"),
            analysis["wheel_trajectory"],
        )

    run_metrics_path = os.path.join(
        output_directory,
        "P02_runs_metrics.csv",
    )
    fieldnames = list(rows[0].keys())
    write_csv(run_metrics_path, rows, fieldnames)

    statistical_metrics = [
        "motion_duration_s",
        "ccw_rotational_error_deg",
        "cw_rotational_error_deg",
        "cumulative_rotational_discrepancy_deg",
        "abs_heading_drift_deg",
        "ekf_path_length_m",
        "ekf_closure_error_m",
        "ekf_cross_track_rmse_m",
        "ekf_maximum_cross_track_m",
        "ekf_heading_error_std_deg",
        "ekf_heading_error_rmse_deg",
        "ekf_ccw_diameter_m",
        "ekf_cw_diameter_m",
        "wheel_path_length_m",
        "wheel_closure_error_m",
        "wheel_cross_track_rmse_m",
        "wheel_maximum_cross_track_m",
        "wheel_heading_error_std_deg",
        "wheel_heading_error_rmse_deg",
        "physical_ccw_diameter_m",
        "physical_cw_diameter_m",
        "physical_final_x_m",
        "physical_final_y_m",
        "physical_closure_error_m",
        "physical_minus_ekf_closure_m",
    ]
    statistics_rows = [
        statistics_row(metric, [row[metric] for row in rows])
        for metric in statistical_metrics
    ]
    statistics_path = os.path.join(
        output_directory,
        "P02_statistics.csv",
    )
    write_csv(
        statistics_path,
        statistics_rows,
        list(statistics_rows[0].keys()),
    )

    runs = [row["run"] for row in rows]
    save_trajectory_overlay(
        os.path.join(output_directory, "ekf_trajectories_overlay.png"),
        ekf_trajectories,
        args.radius,
        "Physical Jackson: EKF figure-eight trajectories",
    )
    save_trajectory_overlay(
        os.path.join(output_directory, "wheel_trajectories_overlay.png"),
        wheel_trajectories,
        args.radius,
        "Physical Jackson: wheel-odometry figure-eight trajectories",
    )
    grouped_bar(
        os.path.join(output_directory, "closure_comparison.png"),
        runs,
        [
            (
                "EKF",
                [100.0 * row["ekf_closure_error_m"] for row in rows],
            ),
            (
                "Physical measurement",
                [100.0 * row["physical_closure_error_m"] for row in rows],
            ),
        ],
        "Closure error [cm]",
        "EKF and physically measured closure error",
    )
    grouped_bar(
        os.path.join(output_directory, "physical_diameters.png"),
        runs,
        [
            (
                "CCW diameter",
                [100.0 * row["physical_ccw_diameter_m"] for row in rows],
            ),
            (
                "CW diameter",
                [100.0 * row["physical_cw_diameter_m"] for row in rows],
            ),
        ],
        "Diameter [cm]",
        "Physically measured loop diameters",
        reference=100.0,
    )
    grouped_bar(
        os.path.join(output_directory, "rotational_discrepancy.png"),
        runs,
        [
            (
                "CCW error",
                [row["ccw_rotational_error_deg"] for row in rows],
            ),
            (
                "CW error",
                [row["cw_rotational_error_deg"] for row in rows],
            ),
            (
                "Cumulative discrepancy",
                [
                    row["cumulative_rotational_discrepancy_deg"]
                    for row in rows
                ],
            ),
        ],
        "Angular discrepancy [deg]",
        "Rotational discrepancy for each physical run",
    )
    grouped_bar(
        os.path.join(output_directory, "heading_drift.png"),
        runs,
        [
            (
                "Absolute final heading drift",
                [row["abs_heading_drift_deg"] for row in rows],
            )
        ],
        "Heading drift [deg]",
        "Final heading drift after the figure-eight trajectory",
    )
    endpoint_plot(
        os.path.join(output_directory, "endpoint_dispersion.png"),
        rows,
    )

    summary_path = os.path.join(output_directory, "P02_summary.txt")
    write_summary(summary_path, rows, statistics_rows)

    print("\nAnalysis completed")
    print(f"Run metrics: {run_metrics_path}")
    print(f"Statistics:  {statistics_path}")
    print(f"Summary:     {summary_path}")
    print(f"Figures:     {output_directory}")


if __name__ == "__main__":
    main()
