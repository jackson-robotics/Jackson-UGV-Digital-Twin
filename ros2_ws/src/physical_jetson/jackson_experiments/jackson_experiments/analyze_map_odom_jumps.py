#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
import os

import rosbag2_py

from geometry_msgs.msg import Twist
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


def command_state(linear_x, angular_z):
    if abs(angular_z) > 1.0e-3:
        return "TURN"

    if abs(linear_x) > 1.0e-3:
        return "STRAIGHT"

    return "STOPPED"


def main():
    parser = argparse.ArgumentParser(
        description="Analyze map to odom changes in a ROS 2 bag."
    )

    parser.add_argument(
        "bag_path",
        help="Ruta al directorio del ROS bag",
    )

    args = parser.parse_args()

    bag_path = os.path.abspath(
        os.path.expanduser(args.bag_path)
    )

    if not os.path.isdir(bag_path):
        raise RuntimeError(
            f"No existe el bag: {bag_path}"
        )

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=bag_path,
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions("", ""),
    )

    map_odom_samples = []
    command_samples = []

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp = timestamp_ns * 1.0e-9

        if topic == "/cmd_vel":
            message = deserialize_message(
                data,
                Twist,
            )

            command_samples.append(
                (
                    timestamp,
                    message.linear.x,
                    message.angular.z,
                )
            )

        elif topic == "/tf":
            message = deserialize_message(
                data,
                TFMessage,
            )

            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")

                if parent == "map" and child == "odom":
                    map_odom_samples.append(
                        (
                            timestamp,
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            quaternion_to_yaw(
                                transform.transform.rotation
                            ),
                        )
                    )

    map_odom_samples.sort(key=lambda sample: sample[0])
    command_samples.sort(key=lambda sample: sample[0])

    if len(map_odom_samples) < 2:
        raise RuntimeError(
            "No hay suficientes transformaciones map -> odom."
        )

    command_times = [
        sample[0]
        for sample in command_samples
    ]

    start_time = map_odom_samples[0][0]
    changes = []

    for index in range(1, len(map_odom_samples)):
        previous = map_odom_samples[index - 1]
        current = map_odom_samples[index]

        delta_time = current[0] - previous[0]
        delta_x = current[1] - previous[1]
        delta_y = current[2] - previous[2]

        translation_change = math.hypot(
            delta_x,
            delta_y,
        )

        yaw_change = normalize_angle(
            current[3] - previous[3]
        )

        yaw_change_degrees = math.degrees(
            yaw_change
        )

        linear_x = 0.0
        angular_z = 0.0

        if command_times:
            command_index = (
                bisect.bisect_right(
                    command_times,
                    current[0],
                )
                - 1
            )

            if command_index >= 0:
                command = command_samples[
                    command_index
                ]

                linear_x = command[1]
                angular_z = command[2]

        changes.append(
            {
                "time_s": current[0] - start_time,
                "delta_time_s": delta_time,
                "map_odom_x_m": current[1],
                "map_odom_y_m": current[2],
                "map_odom_yaw_deg": math.degrees(
                    current[3]
                ),
                "translation_change_m": (
                    translation_change
                ),
                "yaw_change_deg": yaw_change_degrees,
                "cmd_linear_x": linear_x,
                "cmd_angular_z": angular_z,
                "motion_state": command_state(
                    linear_x,
                    angular_z,
                ),
            }
        )

    output_directory = bag_path + "_analysis"

    os.makedirs(
        output_directory,
        exist_ok=True,
    )

    csv_path = os.path.join(
        output_directory,
        "map_odom_changes.csv",
    )

    with open(
        csv_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(changes[0].keys()),
        )

        writer.writeheader()
        writer.writerows(changes)

    ranked = sorted(
        changes,
        key=lambda row: max(
            row["translation_change_m"] / 0.05,
            abs(row["yaw_change_deg"]) / 5.0,
        ),
        reverse=True,
    )

    translation_jumps = sum(
        row["translation_change_m"] >= 0.05
        for row in changes
    )

    yaw_jumps = sum(
        abs(row["yaw_change_deg"]) >= 5.0
        for row in changes
    )

    print()
    print(
        f"Muestras map -> odom: "
        f"{len(map_odom_samples)}"
    )

    print(
        "Cambios de traslación >= 0.05 m: "
        f"{translation_jumps}"
    )

    print(
        "Cambios de yaw >= 5 grados: "
        f"{yaw_jumps}"
    )

    print()
    print("Los 20 mayores cambios:")
    print()
    print(
        " tiempo[s]    cambio[m]    cambio_yaw[deg]"
        "    estado       vx       wz"
    )

    for row in ranked[:20]:
        print(
            f"{row['time_s']:10.3f}"
            f"{row['translation_change_m']:13.4f}"
            f"{row['yaw_change_deg']:19.2f}"
            f"    {row['motion_state']:8s}"
            f"{row['cmd_linear_x']:9.3f}"
            f"{row['cmd_angular_z']:9.3f}"
        )

    print()
    print(f"CSV guardado en: {csv_path}")


if __name__ == "__main__":
    main()
