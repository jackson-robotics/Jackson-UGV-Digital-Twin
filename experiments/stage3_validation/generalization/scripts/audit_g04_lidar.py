import os, glob, math, statistics, csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization/runs/G04_LIDAR_DEGRADED"
)

OUT = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization/G04_LIDAR_ACTUAL_AUDIT.csv"
)

rows = []

for n in range(1, 16):
    run = f"R{n:02d}"
    matches = glob.glob(os.path.join(ROOT, f"G04_LIDAR_DEGRADED_{run}_*"))

    if len(matches) != 1:
        raise RuntimeError(f"{run}: found {len(matches)} directories")

    bag = matches[0]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    types = {x.name: x.type for x in reader.get_all_topics_and_types()}

    raw = {}
    deg = {}

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic not in ("/scan", "/scan_degraded"):
            continue

        msg = deserialize_message(data, get_message(types[topic]))

        stamp = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        if topic == "/scan":
            raw[stamp] = msg
        else:
            deg[stamp] = msg

    common = sorted(set(raw) & set(deg))

    valid_raw = 0
    dropped = 0
    diffs = []

    for stamp in common:
        a = raw[stamp]
        b = deg[stamp]

        for r0, r1 in zip(a.ranges, b.ranges):
            r0 = float(r0)
            r1 = float(r1)

            if not math.isfinite(r0):
                continue

            if r0 < a.range_min or r0 > a.range_max:
                continue

            valid_raw += 1

            if not math.isfinite(r1):
                dropped += 1
                continue

            diffs.append(r1 - r0)

    dropout = 100.0 * dropped / valid_raw

    mean_diff = statistics.mean(diffs)
    sd_diff = statistics.stdev(diffs)

    rows.append({
        "run": run,
        "matched_scans": len(common),
        "valid_raw_beams": valid_raw,
        "dropped_beams": dropped,
        "dropout_pct": dropout,
        "noise_mean_m": mean_diff,
        "noise_sd_m": sd_diff,
    })

with open(OUT, "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=rows[0].keys())
    w.writeheader()
    w.writerows(rows)

print("=" * 82)
print("G04 LiDAR — ACTUAL BAG AUDIT")
print("=" * 82)
print(f"{'RUN':<6}{'MATCHED':>10}{'DROPOUT %':>14}{'NOISE MEAN m':>18}{'NOISE SD m':>16}")
print("-" * 82)

for r in rows:
    print(
        f"{r['run']:<6}"
        f"{r['matched_scans']:>10}"
        f"{r['dropout_pct']:>14.3f}"
        f"{r['noise_mean_m']:>18.5f}"
        f"{r['noise_sd_m']:>16.5f}"
    )

print("-" * 82)
print(
    "Overall mean dropout: "
    f"{statistics.mean(r['dropout_pct'] for r in rows):.3f}%"
)
print(
    "Overall mean noise SD: "
    f"{statistics.mean(r['noise_sd_m'] for r in rows):.5f} m"
)
print()
print("CSV:", OUT)
