import os
import glob
import csv
import math
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT_GEN = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization"
)

ROOT_BASE = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_1_sensitivity"
)

OUT_RUN = os.path.join(
    ROOT_GEN,
    "GENERALIZATION_RUN_LEVEL_METRICS.csv"
)

OUT_SUM = os.path.join(
    ROOT_GEN,
    "GENERALIZATION_CONDITION_SUMMARY.csv"
)

OUT_TXT = os.path.join(
    ROOT_GEN,
    "GENERALIZATION_METRICS_AUDIT.txt"
)

# Cylinder center expressed in the local GT frame.
# Baseline centerline is y = 0.
CONDITIONS = {
    "C00_BASE": {
        "root": os.path.join(ROOT_BASE, "runs", "C00_BASE"),
        "cyl_x": 1.3000,
        "cyl_y": 0.0000,
        "label": "Baseline",
    },
    "G01_CYL_PLUS_Y": {
        "root": os.path.join(ROOT_GEN, "runs", "G01_CYL_PLUS_Y"),
        "cyl_x": 1.3000,
        "cyl_y": 0.1500,
        "label": "Obstacle +0.15 m Y",
    },
    "G02_CYL_MINUS_Y": {
        "root": os.path.join(ROOT_GEN, "runs", "G02_CYL_MINUS_Y"),
        "cyl_x": 1.3000,
        "cyl_y": -0.1500,
        "label": "Obstacle -0.15 m Y",
    },
    "G03_LOW_FRICTION": {
        "root": os.path.join(ROOT_GEN, "runs", "G03_LOW_FRICTION"),
        "cyl_x": 1.3000,
        "cyl_y": 0.0000,
        "label": "Low-friction surface",
    },
    "G04_LIDAR_DEGRADED": {
        "root": os.path.join(ROOT_GEN, "runs", "G04_LIDAR_DEGRADED"),
        "cyl_x": 1.3000,
        "cyl_y": 0.0000,
        "label": "Degraded LiDAR",
    },
}

# Conservative USD collision-envelope rectangle relative to base_link.
RECT_X_MIN = -0.135000
RECT_X_MAX =  0.135177
RECT_Y_MIN = -0.135439
RECT_Y_MAX =  0.135000

CYLINDER_RADIUS = 0.125


def stamp_s(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(vals, p):
    vals = sorted(vals)
    if len(vals) == 1:
        return vals[0]

    z = (len(vals) - 1) * p
    lo = math.floor(z)
    hi = math.ceil(z)

    if lo == hi:
        return vals[lo]

    return vals[lo] * (hi-z) + vals[hi] * (z-lo)


def summary(vals):
    vals = list(vals)
    return {
        "n": len(vals),
        "median": statistics.median(vals),
        "q1": percentile(vals, 0.25),
        "q3": percentile(vals, 0.75),
        "mean": statistics.mean(vals),
        "sd": statistics.stdev(vals) if len(vals) > 1 else 0.0,
        "min": min(vals),
        "max": max(vals),
    }


def get_result(run_dir):
    notes = os.path.join(run_dir, "RUN_NOTES.txt")
    if not os.path.isfile(notes):
        return "UNKNOWN"

    for line in open(notes, encoding="utf-8", errors="replace"):
        if line.startswith("Result:"):
            return line.split(":", 1)[1].strip()

    return "UNKNOWN"


def point_to_rect_gap(robot_x, robot_y, yaw, cyl_x, cyl_y):

    # Cylinder center in robot coordinates.
    dx = cyl_x - robot_x
    dy = cyl_y - robot_y

    c = math.cos(yaw)
    s = math.sin(yaw)

    xr =  c * dx + s * dy
    yr = -s * dx + c * dy

    qx = min(max(xr, RECT_X_MIN), RECT_X_MAX)
    qy = min(max(yr, RECT_Y_MIN), RECT_Y_MAX)

    d = math.hypot(xr - qx, yr - qy)

    return d - CYLINDER_RADIUS


def analyze_run(run_dir, cyl_x, cyl_y):

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=run_dir,
            storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    types = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }

    wanted = {
        "/ground_truth/odom",
        "/navigate_to_pose/_action/feedback",
        "/stage3/cylinder_inserted",
        "/clock",
    }

    gt = []
    feedback = []
    clocks = []
    inserted_receipt_ns = None

    while reader.has_next():
        topic, raw, recv_ns = reader.read_next()

        if topic not in wanted:
            continue

        msg = deserialize_message(
            raw,
            get_message(types[topic])
        )

        if topic == "/ground_truth/odom":
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            yaw = math.atan2(
                2.0*(q.w*q.z + q.x*q.y),
                1.0 - 2.0*(q.y*q.y + q.z*q.z)
            )

            gt.append(
                (
                    stamp_s(msg.header.stamp),
                    float(p.x),
                    float(p.y),
                    yaw,
                )
            )

        elif topic == "/navigate_to_pose/_action/feedback":
            fb = msg.feedback

            feedback.append(
                (
                    stamp_s(fb.current_pose.header.stamp),
                    stamp_s(fb.navigation_time),
                )
            )

        elif topic == "/clock":
            clocks.append(
                (
                    int(recv_ns),
                    stamp_s(msg.clock),
                )
            )

        elif topic == "/stage3/cylinder_inserted":
            if bool(msg.data) and inserted_receipt_ns is None:
                inserted_receipt_ns = int(recv_ns)

    if not feedback:
        raise RuntimeError(
            f"No NavigateToPose feedback in {run_dir}"
        )

    if not gt:
        raise RuntimeError(
            f"No ground truth in {run_dir}"
        )

    # Navigation interval in simulation time.
    starts = [
        pose_t - nav_t
        for pose_t, nav_t in feedback
        if pose_t > 0.0
    ]

    nav_start = statistics.median(starts)
    nav_time = max(nav_t for _, nav_t in feedback)
    nav_end = nav_start + nav_time

    gt_nav = [
        row for row in gt
        if nav_start <= row[0] <= nav_end
    ]

    if len(gt_nav) < 2:
        raise RuntimeError(
            f"Insufficient GT samples in nav window: {run_dir}"
        )

    # GT path length.
    path_length = 0.0

    for a, b in zip(gt_nav[:-1], gt_nav[1:]):
        path_length += math.hypot(
            b[1] - a[1],
            b[2] - a[2]
        )

    # Map insertion receipt time -> simulator clock time.
    insert_sim_time = None

    if inserted_receipt_ns is not None and clocks:
        insert_sim_time = min(
            clocks,
            key=lambda x: abs(x[0] - inserted_receipt_ns)
        )[1]

    # Minimum clearance only after cylinder insertion.
    min_clearance = float("nan")

    if insert_sim_time is not None:

        candidates = [
            row for row in gt_nav
            if row[0] >= insert_sim_time
        ]

        if candidates:
            min_clearance = min(
                point_to_rect_gap(
                    x, y, yaw,
                    cyl_x, cyl_y
                )
                for _, x, y, yaw in candidates
            )

    return {
        "navigation_time_s": nav_time,
        "gt_path_length_m": path_length,
        "minimum_clearance_m": min_clearance,
        "nav_start_sim_s": nav_start,
        "nav_end_sim_s": nav_end,
        "cylinder_insert_sim_s": insert_sim_time,
    }


run_rows = []

for condition, cfg in CONDITIONS.items():

    for n in range(1, 16):
        run_id = f"R{n:02d}"

        matches = sorted(
            glob.glob(
                os.path.join(
                    cfg["root"],
                    f"{condition}_{run_id}_*"
                )
            )
        )

        if len(matches) != 1:
            raise RuntimeError(
                f"{condition}/{run_id}: found {len(matches)} run directories"
            )

        run_dir = matches[0]
        result = get_result(run_dir)

        metrics = analyze_run(
            run_dir,
            cfg["cyl_x"],
            cfg["cyl_y"],
        )

        row = {
            "condition": condition,
            "label": cfg["label"],
            "run": run_id,
            "result": result,
            **metrics,
        }

        run_rows.append(row)

        print(
            f"{condition} {run_id} | "
            f"{result} | "
            f"T={metrics['navigation_time_s']:.3f}s | "
            f"L={metrics['gt_path_length_m']:.3f}m | "
            f"clear={metrics['minimum_clearance_m']:.3f}m"
        )


with open(
    OUT_RUN,
    "w",
    newline="",
    encoding="utf-8"
) as f:
    writer = csv.DictWriter(
        f,
        fieldnames=list(run_rows[0].keys())
    )
    writer.writeheader()
    writer.writerows(run_rows)


condition_rows = []

for condition, cfg in CONDITIONS.items():

    rr = [
        r for r in run_rows
        if r["condition"] == condition
    ]

    success = [
        r for r in rr
        if r["result"] == "SUCCEEDED"
    ]

    nav = summary(
        r["navigation_time_s"]
        for r in success
    )

    path = summary(
        r["gt_path_length_m"]
        for r in success
    )

    clear_vals = [
        r["minimum_clearance_m"]
        for r in success
        if math.isfinite(r["minimum_clearance_m"])
    ]

    clear = summary(clear_vals)

    condition_rows.append({
        "condition": condition,
        "label": cfg["label"],
        "runs": len(rr),
        "successful": len(success),
        "success_rate_pct": 100.0 * len(success) / len(rr),

        "nav_median_s": nav["median"],
        "nav_q1_s": nav["q1"],
        "nav_q3_s": nav["q3"],
        "nav_mean_s": nav["mean"],
        "nav_sd_s": nav["sd"],

        "path_median_m": path["median"],
        "path_q1_m": path["q1"],
        "path_q3_m": path["q3"],
        "path_mean_m": path["mean"],
        "path_sd_m": path["sd"],

        "clear_median_m": clear["median"],
        "clear_q1_m": clear["q1"],
        "clear_q3_m": clear["q3"],
        "clear_mean_m": clear["mean"],
        "clear_sd_m": clear["sd"],
        "clear_min_m": clear["min"],
    })


with open(
    OUT_SUM,
    "w",
    newline="",
    encoding="utf-8"
) as f:
    writer = csv.DictWriter(
        f,
        fieldnames=list(condition_rows[0].keys())
    )
    writer.writeheader()
    writer.writerows(condition_rows)


with open(
    OUT_TXT,
    "w",
    encoding="utf-8"
) as f:

    f.write(
        "REVIEWER 1 COMMENT 5 — GENERALIZATION METRIC AUDIT\n"
    )
    f.write("=" * 90 + "\n\n")

    f.write(
        "Total trials: 75 = 15 baseline + 60 new generalization trials.\n"
    )
    f.write(
        "Continuous metrics use SUCCEEDED trials. "
        "All runs contribute to success rate.\n\n"
    )

    f.write(
        "Navigation time: maximum NavigateToPose feedback.navigation_time "
        "using simulation time.\n"
    )
    f.write(
        "GT path length: integrated planar distance from "
        "/ground_truth/odom inside the navigation interval.\n"
    )
    f.write(
        "Minimum clearance: post-insertion circle-to-oriented-rectangle "
        "gap using the audited conservative USD collision envelope.\n\n"
    )

    for r in condition_rows:

        f.write(f"{r['label']} ({r['condition']})\n")
        f.write(
            f"  Success: {r['successful']}/{r['runs']} "
            f"({r['success_rate_pct']:.1f}%)\n"
        )
        f.write(
            "  Navigation time median [IQR]: "
            f"{r['nav_median_s']:.2f} "
            f"[{r['nav_q1_s']:.2f}, {r['nav_q3_s']:.2f}] s\n"
        )
        f.write(
            "  GT path length median [IQR]: "
            f"{r['path_median_m']:.3f} "
            f"[{r['path_q1_m']:.3f}, {r['path_q3_m']:.3f}] m\n"
        )
        f.write(
            "  Minimum clearance median [IQR]: "
            f"{100*r['clear_median_m']:.2f} "
            f"[{100*r['clear_q1_m']:.2f}, "
            f"{100*r['clear_q3_m']:.2f}] cm\n"
        )
        f.write(
            f"  Minimum observed clearance: "
            f"{100*r['clear_min_m']:.2f} cm\n\n"
        )


print()
print("=" * 118)
print("GENERALIZATION SUMMARY")
print("=" * 118)

print(
    f"{'CONDITION':<24}"
    f"{'SUCCESS':>10}"
    f"{'NAV median[IQR] s':>25}"
    f"{'PATH median[IQR] m':>26}"
    f"{'CLEAR median[IQR] cm':>29}"
)

print("-" * 118)

for r in condition_rows:

    succ = f"{r['successful']}/{r['runs']}"

    nav_txt = (
        f"{r['nav_median_s']:.2f}"
        f"[{r['nav_q1_s']:.2f},{r['nav_q3_s']:.2f}]"
    )

    path_txt = (
        f"{r['path_median_m']:.3f}"
        f"[{r['path_q1_m']:.3f},{r['path_q3_m']:.3f}]"
    )

    clear_txt = (
        f"{100*r['clear_median_m']:.2f}"
        f"[{100*r['clear_q1_m']:.2f},"
        f"{100*r['clear_q3_m']:.2f}]"
    )

    print(
        f"{r['condition']:<24}"
        f"{succ:>10}"
        f"{nav_txt:>25}"
        f"{path_txt:>26}"
        f"{clear_txt:>29}"
    )

print()
print("Run-level CSV:", OUT_RUN)
print("Summary CSV:", OUT_SUM)
print("Audit:", OUT_TXT)
