import os, glob, csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization/runs/G04_LIDAR_DEGRADED"
)

METRICS = os.path.expanduser(
    "~/jackson_dt_ws/revision_r1_5_generalization/GENERALIZATION_RUN_LEVEL_METRICS.csv"
)

nav = {}
with open(METRICS, newline="", encoding="utf-8") as f:
    for r in csv.DictReader(f):
        if r["condition"] == "G04_LIDAR_DEGRADED":
            nav[r["run"]] = float(r["navigation_time_s"])

rows = []

for n in range(1, 16):
    run = f"R{n:02d}"
    bag = glob.glob(
        os.path.join(ROOT, f"G04_LIDAR_DEGRADED_{run}_*")
    )[0]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    max_recoveries = 0

    while reader.has_next():
        topic, raw, _ = reader.read_next()

        if topic != "/navigate_to_pose/_action/feedback":
            continue

        msg = deserialize_message(
            raw,
            get_message(types[topic])
        )

        max_recoveries = max(
            max_recoveries,
            int(msg.feedback.number_of_recoveries)
        )

    rows.append(
        (run, nav[run], max_recoveries)
    )

rows.sort(key=lambda x: x[1], reverse=True)

print("=" * 60)
print("G04 — NAVIGATION TIME VS RECOVERIES")
print("=" * 60)
print(f"{'RUN':<6}{'NAV TIME s':>14}{'RECOVERIES':>14}")
print("-" * 60)

for run, t, rec in rows:
    print(f"{run:<6}{t:>14.2f}{rec:>14d}")
