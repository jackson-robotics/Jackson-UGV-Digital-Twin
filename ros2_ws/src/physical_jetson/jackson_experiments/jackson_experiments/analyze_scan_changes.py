#!/usr/bin/env python3

import math
import os
import sys
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


bag_path = os.path.abspath(os.path.expanduser(sys.argv[1]))

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3",
    ),
    rosbag2_py.ConverterOptions("", ""),
)

previous_ranges = None
total_scans = 0
identical_scans = 0
valid_differences = []
header_offsets = []
frame_ids = set()

while reader.has_next():
    topic, data, timestamp_ns = reader.read_next()

    if topic != "/scan":
        continue

    scan = deserialize_message(data, LaserScan)
    total_scans += 1
    frame_ids.add(scan.header.frame_id)

    header_time = (
        scan.header.stamp.sec
        + scan.header.stamp.nanosec * 1.0e-9
    )
    record_time = timestamp_ns * 1.0e-9
    header_offsets.append(header_time - record_time)

    current_ranges = list(scan.ranges)

    if previous_ranges is not None:
        differences = []

        for previous, current in zip(
            previous_ranges,
            current_ranges,
        ):
            if math.isfinite(previous) and math.isfinite(current):
                differences.append(abs(current - previous))

        if current_ranges == previous_ranges:
            identical_scans += 1

        if differences:
            valid_differences.append(
                statistics.median(differences)
            )

    previous_ranges = current_ranges

print()
print(f"Escaneos analizados: {total_scans}")
print(f"Frame IDs: {sorted(frame_ids)}")
print(f"Escaneos consecutivos idénticos: {identical_scans}")

if total_scans > 1:
    percentage = 100.0 * identical_scans / (total_scans - 1)
    print(f"Porcentaje idéntico: {percentage:.2f} %")

if valid_differences:
    print(
        "Mediana global del cambio entre escaneos: "
        f"{statistics.median(valid_differences):.6f} m"
    )
    print(
        "Máximo de las medianas de cambio: "
        f"{max(valid_differences):.6f} m"
    )

if header_offsets:
    print(
        "Diferencia mediana header−registro: "
        f"{statistics.median(header_offsets):.6f} s"
    )
    print(
        "Diferencia mínima/máxima: "
        f"{min(header_offsets):.6f} / "
        f"{max(header_offsets):.6f} s"
    )
