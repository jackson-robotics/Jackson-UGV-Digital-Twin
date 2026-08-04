#!/usr/bin/env python3
"""Offline analysis of Jackson Stage-3 physical ROS 2 bags (P03, R06-R15).

The script keeps reference frames separate:
* EKF odometry is used for path and motion metrics in the odom frame.
* AMCL is used for final goal error in the map frame.
* LaserScan is reported as observed clearance/proximity, not as collision proof.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


RUN_RE = re.compile(r"_R(\d{2})(?:_|$)")


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def path_length(samples: list[dict], start_ns: int, end_ns: int) -> float:
    selected = [s for s in samples if start_ns <= s["timestamp_ns"] <= end_ns]
    return sum(
        math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"])
        for a, b in zip(selected, selected[1:])
    )


def write_csv(path: Path, rows: list[dict], fieldnames: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    names = fieldnames or list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=names)
        writer.writeheader()
        writer.writerows(rows)


def finite_or_blank(value):
    return value if isinstance(value, (int, float)) and math.isfinite(value) else ""


def analyse_bag(
    bag: Path,
    typestore,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    goal_tolerance: float,
    linear_threshold: float,
    angular_threshold: float,
    proximity_threshold: float,
    front_half_angle_deg: float,
) -> tuple[dict, list[dict], list[dict]]:
    match = RUN_RE.search(bag.name)
    if not match:
        raise ValueError(f"Cannot obtain run identifier from {bag.name}")
    run = f"R{match.group(1)}"

    odom: list[dict] = []
    amcl: list[dict] = []
    commands: list[dict] = []
    scan_minima: list[tuple[int, float, float]] = []
    first_timestamp_ns = None
    last_timestamp_ns = None

    with AnyReader([bag], default_typestore=typestore) as reader:
        wanted = {
            c.topic: c
            for c in reader.connections
            if c.topic in {"/odometry/filtered", "/amcl_pose", "/cmd_vel", "/scan"}
        }
        required = {"/odometry/filtered", "/cmd_vel", "/scan"}
        missing = required.difference(wanted)
        if missing:
            raise RuntimeError(f"{run}: missing required topics: {sorted(missing)}")

        for connection, timestamp_ns, rawdata in reader.messages(connections=list(wanted.values())):
            first_timestamp_ns = timestamp_ns if first_timestamp_ns is None else min(first_timestamp_ns, timestamp_ns)
            last_timestamp_ns = timestamp_ns if last_timestamp_ns is None else max(last_timestamp_ns, timestamp_ns)
            msg = reader.deserialize(rawdata, connection.msgtype)

            if connection.topic == "/odometry/filtered":
                pose = msg.pose.pose
                twist = msg.twist.twist
                odom.append({
                    "run": run,
                    "timestamp_ns": timestamp_ns,
                    "x_m": float(pose.position.x),
                    "y_m": float(pose.position.y),
                    "yaw_rad": yaw_from_quaternion(pose.orientation),
                    "linear_x_mps": float(twist.linear.x),
                    "angular_z_radps": float(twist.angular.z),
                })
            elif connection.topic == "/amcl_pose":
                pose = msg.pose.pose
                amcl.append({
                    "run": run,
                    "timestamp_ns": timestamp_ns,
                    "x_map_m": float(pose.position.x),
                    "y_map_m": float(pose.position.y),
                    "yaw_map_rad": yaw_from_quaternion(pose.orientation),
                })
            elif connection.topic == "/cmd_vel":
                commands.append({
                    "timestamp_ns": timestamp_ns,
                    "linear_x_mps": float(msg.linear.x),
                    "angular_z_radps": float(msg.angular.z),
                })
            elif connection.topic == "/scan":
                valid_all = []
                valid_front = []
                half_angle = math.radians(front_half_angle_deg)
                for index, value in enumerate(msg.ranges):
                    distance = float(value)
                    if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                        continue
                    valid_all.append(distance)
                    angle = msg.angle_min + index * msg.angle_increment
                    wrapped = math.atan2(math.sin(angle), math.cos(angle))
                    if abs(wrapped) <= half_angle:
                        valid_front.append(distance)
                if valid_all:
                    scan_minima.append((
                        timestamp_ns,
                        min(valid_all),
                        min(valid_front) if valid_front else math.nan,
                    ))

    if first_timestamp_ns is None or last_timestamp_ns is None or not odom:
        raise RuntimeError(f"{run}: bag has no usable messages")

    active_commands = [
        c for c in commands
        if abs(c["linear_x_mps"]) >= linear_threshold
        or abs(c["angular_z_radps"]) >= angular_threshold
    ]
    if active_commands:
        movement_start_ns = active_commands[0]["timestamp_ns"]
        movement_end_ns = active_commands[-1]["timestamp_ns"]
    else:
        movement_start_ns = first_timestamp_ns
        movement_end_ns = last_timestamp_ns

    movement_odom = [s for s in odom if movement_start_ns <= s["timestamp_ns"] <= movement_end_ns]
    if len(movement_odom) < 2:
        movement_odom = odom
        movement_start_ns = odom[0]["timestamp_ns"]
        movement_end_ns = odom[-1]["timestamp_ns"]

    start_pose = movement_odom[0]
    end_pose = movement_odom[-1]
    trajectory_length = path_length(odom, movement_start_ns, movement_end_ns)
    displacement = math.hypot(end_pose["x_m"] - start_pose["x_m"], end_pose["y_m"] - start_pose["y_m"])

    moving_linear = [abs(c["linear_x_mps"]) for c in active_commands]
    moving_angular = [abs(c["angular_z_radps"]) for c in active_commands]
    min_scan = min((v[1] for v in scan_minima), default=math.nan)
    min_front_scan = min((v[2] for v in scan_minima if math.isfinite(v[2])), default=math.nan)
    proximity_scans = sum(1 for _, minimum, _ in scan_minima if minimum <= proximity_threshold)

    final_amcl = amcl[-1] if amcl else None
    if final_amcl:
        goal_error = math.hypot(final_amcl["x_map_m"] - goal_x, final_amcl["y_map_m"] - goal_y)
        final_yaw_error = abs(angle_error(final_amcl["yaw_map_rad"], goal_yaw))
        amcl_age_s = (last_timestamp_ns - final_amcl["timestamp_ns"]) / 1e9
        inferred_success = int(goal_error <= goal_tolerance)
    else:
        goal_error = math.nan
        final_yaw_error = math.nan
        amcl_age_s = math.nan
        inferred_success = 0

    for sample in odom:
        sample["time_from_bag_start_s"] = (sample["timestamp_ns"] - first_timestamp_ns) / 1e9
        sample["x_relative_m"] = sample["x_m"] - start_pose["x_m"]
        sample["y_relative_m"] = sample["y_m"] - start_pose["y_m"]
    for sample in amcl:
        sample["time_from_bag_start_s"] = (sample["timestamp_ns"] - first_timestamp_ns) / 1e9

    metrics = {
        "run": run,
        "bag_directory": bag.name,
        "bag_duration_s": (last_timestamp_ns - first_timestamp_ns) / 1e9,
        "movement_duration_s": (movement_end_ns - movement_start_ns) / 1e9,
        "ekf_path_length_m": trajectory_length,
        "ekf_start_to_end_displacement_m": displacement,
        "mean_abs_cmd_linear_mps": statistics.fmean(moving_linear) if moving_linear else 0.0,
        "max_abs_cmd_linear_mps": max(moving_linear, default=0.0),
        "mean_abs_cmd_angular_radps": statistics.fmean(moving_angular) if moving_angular else 0.0,
        "max_abs_cmd_angular_radps": max(moving_angular, default=0.0),
        "minimum_observed_lidar_range_m": min_scan,
        "minimum_front_sector_range_m": min_front_scan,
        "scans_at_or_below_proximity_threshold": proximity_scans,
        "scan_message_count": len(scan_minima),
        "amcl_message_count": len(amcl),
        "final_amcl_x_map_m": final_amcl["x_map_m"] if final_amcl else math.nan,
        "final_amcl_y_map_m": final_amcl["y_map_m"] if final_amcl else math.nan,
        "final_amcl_yaw_rad": final_amcl["yaw_map_rad"] if final_amcl else math.nan,
        "final_position_error_to_goal_m": goal_error,
        "final_abs_yaw_error_rad": final_yaw_error,
        "final_amcl_age_at_bag_end_s": amcl_age_s,
        "goal_reached_inferred_0_1": inferred_success,
    }
    return metrics, odom, amcl


def make_summary(metrics: list[dict]) -> list[dict]:
    excluded = {"run", "bag_directory", "goal_reached_inferred_0_1"}
    rows = []
    for key in metrics[0]:
        if key in excluded:
            continue
        values = [float(row[key]) for row in metrics if isinstance(row[key], (int, float)) and math.isfinite(row[key])]
        if not values:
            continue
        rows.append({
            "metric": key,
            "n": len(values),
            "mean": statistics.fmean(values),
            "sample_std": statistics.stdev(values) if len(values) > 1 else 0.0,
            "median": statistics.median(values),
            "minimum": min(values),
            "maximum": max(values),
        })
    successes = sum(int(row["goal_reached_inferred_0_1"]) for row in metrics)
    rows.append({
        "metric": "inferred_goal_success_rate",
        "n": len(metrics),
        "mean": successes / len(metrics),
        "sample_std": "",
        "median": "",
        "minimum": successes,
        "maximum": len(metrics),
    })
    return rows


def make_plots(output_dir: Path, trajectories: list[dict], metrics: list[dict]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("WARNING: matplotlib is unavailable; CSV files were still generated.")
        return

    fig, ax = plt.subplots(figsize=(7.2, 6.0))
    for run in sorted({row["run"] for row in trajectories}):
        data = [row for row in trajectories if row["run"] == run]
        ax.plot([r["x_relative_m"] for r in data], [r["y_relative_m"] for r in data], label=run, linewidth=1.2)
    ax.set_xlabel("Relative EKF x (m)")
    ax.set_ylabel("Relative EKF y (m)")
    ax.set_title("P03 physical trajectories (odom frame, start-normalized)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=8)
    fig.tight_layout()
    fig.savefig(output_dir / "P03_ekf_trajectories.png", dpi=300)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 4.5))
    runs = [m["run"] for m in metrics]
    errors = [m["final_position_error_to_goal_m"] for m in metrics]
    ax.bar(runs, errors, color="#4472C4")
    ax.axhline(0.25, color="#C00000", linestyle="--", linewidth=1.2, label="0.25 m criterion")
    ax.set_ylabel("Final AMCL position error (m)")
    ax.set_title("P03 inferred goal-reaching criterion")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "P03_final_goal_errors.png", dpi=300)
    plt.close(fig)


def write_methods_note(output_dir: Path, args) -> None:
    text = f"""# P03 physical Stage-3 analysis: methods and limitations

## Inputs and reference frames

- `/odometry/filtered` supplies path length, displacement, and start-normalized trajectories in the `odom` frame.
- `/amcl_pose` supplies final pose and final goal error in the `map` frame.
- `/cmd_vel` defines the effective motion interval using |linear x| >= {args.linear_threshold} m/s or |angular z| >= {args.angular_threshold} rad/s.
- `/scan` supplies minimum valid observed ranges. The front sector is +/-{args.front_half_angle_deg} degrees about the LaserScan zero angle.

## Goal criterion

The known goal is ({args.goal_x}, {args.goal_y}) m with yaw {args.goal_yaw} rad. `goal_reached_inferred_0_1` is 1 when the final AMCL position error is <= {args.goal_tolerance} m. This is an inferred criterion because the bags do not contain the Nav2 action result/status topic.

## Proximity limitation

The threshold {args.proximity_threshold} m is reported only as a proximity indicator. A small LiDAR range does not by itself prove robot-cylinder contact or collision, because the scan may correspond to another surface and the bags do not contain contact sensing or independent obstacle ground truth.

## Timing limitation

`final_amcl_age_at_bag_end_s` reports how old the last AMCL observation is at bag end. It should be inspected before interpreting final goal error because AMCL is recorded sparsely.
"""
    (output_dir / "METHODS_AND_LIMITATIONS.md").write_text(text, encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags-dir", type=Path, default=Path("work/P03_physical"))
    parser.add_argument("--output-dir", type=Path, default=Path("work/P03_processed"))
    parser.add_argument("--goal-x", type=float, default=1.2091)
    parser.add_argument("--goal-y", type=float, default=-0.0364)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.25)
    parser.add_argument("--linear-threshold", type=float, default=0.01)
    parser.add_argument("--angular-threshold", type=float, default=0.05)
    parser.add_argument("--proximity-threshold", type=float, default=0.16)
    parser.add_argument("--front-half-angle-deg", type=float, default=30.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bags = sorted(
        (path for path in args.bags_dir.iterdir() if path.is_dir() and (path / "metadata.yaml").exists()),
        key=lambda path: int(RUN_RE.search(path.name).group(1)) if RUN_RE.search(path.name) else 999,
    )
    bags = [bag for bag in bags if RUN_RE.search(bag.name) and 6 <= int(RUN_RE.search(bag.name).group(1)) <= 15]
    if len(bags) != 10:
        raise SystemExit(f"Expected exactly 10 bags R06-R15 in {args.bags_dir.resolve()}, found {len(bags)}")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    metrics = []
    trajectories = []
    amcl_poses = []
    for bag in bags:
        print(f"Analysing {bag.name} ...")
        run_metrics, run_trajectory, run_amcl = analyse_bag(
            bag, typestore, args.goal_x, args.goal_y, args.goal_yaw,
            args.goal_tolerance, args.linear_threshold, args.angular_threshold,
            args.proximity_threshold, args.front_half_angle_deg,
        )
        metrics.append(run_metrics)
        trajectories.extend(run_trajectory)
        amcl_poses.extend(run_amcl)

    clean_metrics = [{k: finite_or_blank(v) if k not in {"run", "bag_directory"} else v for k, v in row.items()} for row in metrics]
    write_csv(args.output_dir / "P03_run_metrics.csv", clean_metrics)
    write_csv(args.output_dir / "P03_summary_statistics.csv", make_summary(metrics))
    write_csv(args.output_dir / "P03_ekf_trajectories.csv", trajectories)
    write_csv(args.output_dir / "P03_amcl_poses.csv", amcl_poses)
    write_methods_note(args.output_dir, args)
    make_plots(args.output_dir, trajectories, metrics)

    successes = sum(row["goal_reached_inferred_0_1"] for row in metrics)
    print(f"Completed: {len(metrics)} runs; inferred goal successes: {successes}/{len(metrics)}")
    print(f"Results: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
