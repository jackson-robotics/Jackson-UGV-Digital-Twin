#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
import os

import rosbag2_py

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (
        q.w * q.z + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y + q.z * q.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


def crop(samples, start_time, end_time):
    return [
        sample
        for sample in samples
        if start_time <= sample[0] <= end_time
    ]


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
            (
                timestamp - t0,
                local_x,
                local_y,
                local_yaw,
            )
        )

    return normalized


def interpolate_transform(samples, timestamps, target_time):
    if not samples:
        return None

    index = bisect.bisect_left(
        timestamps,
        target_time,
    )

    if index == 0:
        return samples[0][1:]

    if index >= len(samples):
        return samples[-1][1:]

    before = samples[index - 1]
    after = samples[index]

    duration = after[0] - before[0]

    if duration <= 0.0:
        return before[1:]

    ratio = (
        target_time - before[0]
    ) / duration

    x = before[1] + ratio * (
        after[1] - before[1]
    )

    y = before[2] + ratio * (
        after[2] - before[2]
    )

    yaw_delta = normalize_angle(
        after[3] - before[3]
    )

    yaw = normalize_angle(
        before[3] + ratio * yaw_delta
    )

    return x, y, yaw


def compose_pose(map_odom, odom_base):
    map_x, map_y, map_yaw = map_odom
    odom_x, odom_y, odom_yaw = odom_base

    cosine = math.cos(map_yaw)
    sine = math.sin(map_yaw)

    x = (
        map_x
        + cosine * odom_x
        - sine * odom_y
    )

    y = (
        map_y
        + sine * odom_x
        + cosine * odom_y
    )

    yaw = normalize_angle(
        map_yaw + odom_yaw
    )

    return x, y, yaw


def point_to_segment_distance(
    px,
    py,
    ax,
    ay,
    bx,
    by,
):
    ab_x = bx - ax
    ab_y = by - ay

    length_squared = (
        ab_x * ab_x + ab_y * ab_y
    )

    if length_squared <= 0.0:
        return math.hypot(
            px - ax,
            py - ay,
        )

    projection = (
        (px - ax) * ab_x
        + (py - ay) * ab_y
    ) / length_squared

    projection = max(
        0.0,
        min(1.0, projection),
    )

    closest_x = ax + projection * ab_x
    closest_y = ay + projection * ab_y

    return math.hypot(
        px - closest_x,
        py - closest_y,
    )


def ideal_square_distance(x, y, side=1.0):
    segments = [
        (0.0, 0.0, side, 0.0),
        (side, 0.0, side, side),
        (side, side, 0.0, side),
        (0.0, side, 0.0, 0.0),
    ]

    return min(
        point_to_segment_distance(
            x, y,
            ax, ay,
            bx, by,
        )
        for ax, ay, bx, by in segments
    )


def calculate_metrics(samples):
    if len(samples) < 2:
        return None

    path_length = 0.0
    cross_track_squared = []
    maximum_cross_track = 0.0

    for index, sample in enumerate(samples):
        _, x, y, _ = sample

        distance_to_square = (
            ideal_square_distance(x, y)
        )

        cross_track_squared.append(
            distance_to_square ** 2
        )

        maximum_cross_track = max(
            maximum_cross_track,
            distance_to_square,
        )

        if index > 0:
            previous = samples[index - 1]

            path_length += math.hypot(
                x - previous[1],
                y - previous[2],
            )

    final = samples[-1]

    closure_error = math.hypot(
        final[1],
        final[2],
    )

    yaw_error_degrees = math.degrees(
        normalize_angle(final[3])
    )

    cross_track_rmse = math.sqrt(
        sum(cross_track_squared)
        / len(cross_track_squared)
    )

    return {
        "samples": len(samples),
        "duration_s": final[0],
        "path_length_m": path_length,
        "closure_error_m": closure_error,
        "yaw_error_deg": yaw_error_degrees,
        "cross_track_rmse_m": cross_track_rmse,
        "maximum_cross_track_m": maximum_cross_track,
    }


def write_trajectory(path, samples):
    with open(
        path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.writer(file)

        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "yaw_rad",
                "yaw_deg",
            ]
        )

        for timestamp, x, y, yaw in samples:
            writer.writerow(
                [
                    f"{timestamp:.9f}",
                    f"{x:.9f}",
                    f"{y:.9f}",
                    f"{yaw:.9f}",
                    f"{math.degrees(yaw):.6f}",
                ]
            )


def write_summary(path, results):
    fields = [
        "trajectory",
        "samples",
        "duration_s",
        "path_length_m",
        "closure_error_m",
        "yaw_error_deg",
        "cross_track_rmse_m",
        "maximum_cross_track_m",
    ]

    with open(
        path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fields,
        )

        writer.writeheader()

        for trajectory, metrics in results.items():
            row = {
                "trajectory": trajectory,
            }
            row.update(metrics)
            writer.writerow(row)


def create_plot(path, trajectories):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print(
            "matplotlib no disponible; "
            "se omite la gráfica."
        )
        return

    figure, axis = plt.subplots(
        figsize=(8, 8)
    )

    ideal_x = [0.0, 1.0, 1.0, 0.0, 0.0]
    ideal_y = [0.0, 0.0, 1.0, 1.0, 0.0]

    axis.plot(
        ideal_x,
        ideal_y,
        "k--",
        linewidth=2.0,
        label="Ideal 1 x 1 m",
    )

    colors = {
        "wheel": "tab:orange",
        "ekf": "tab:blue",
        "slam": "tab:green",
    }

    labels = {
        "wheel": "Wheel odometry",
        "ekf": "EKF",
        "slam": "SLAM",
    }

    for name, samples in trajectories.items():
        if not samples:
            continue

        axis.plot(
            [sample[1] for sample in samples],
            [sample[2] for sample in samples],
            linewidth=1.5,
            color=colors[name],
            label=labels[name],
        )

        axis.scatter(
            samples[-1][1],
            samples[-1][2],
            color=colors[name],
            s=35,
        )

    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title(
        "Jackson physical square trajectory"
    )

    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()

    figure.tight_layout()
    figure.savefig(
        path,
        dpi=200,
    )

    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Extract wheel, EKF and SLAM "
            "trajectories from a Jackson ROS bag."
        )
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

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3",
    )

    converter_options = (
        rosbag2_py.ConverterOptions("", "")
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        storage_options,
        converter_options,
    )

    wheel_samples = []
    ekf_samples = []
    map_odom_samples = []
    command_motion_times = []

    while reader.has_next():
        topic, data, timestamp_ns = (
            reader.read_next()
        )

        timestamp = timestamp_ns * 1.0e-9

        if topic == "/cmd_vel":
            msg = deserialize_message(
                data,
                Twist,
            )

            if (
                abs(msg.linear.x) > 1.0e-4
                or abs(msg.angular.z) > 1.0e-4
            ):
                command_motion_times.append(
                    timestamp
                )

        elif topic == "/wheel/odom":
            msg = deserialize_message(
                data,
                Odometry,
            )

            wheel_samples.append(
                (
                    timestamp,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    quaternion_to_yaw(
                        msg.pose.pose.orientation
                    ),
                )
            )

        elif topic == "/odometry/filtered":
            msg = deserialize_message(
                data,
                Odometry,
            )

            ekf_samples.append(
                (
                    timestamp,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    quaternion_to_yaw(
                        msg.pose.pose.orientation
                    ),
                )
            )

        elif topic == "/tf":
            msg = deserialize_message(
                data,
                TFMessage,
            )

            for transform in msg.transforms:
                if (
                    transform.header.frame_id
                    == "map"
                    and transform.child_frame_id
                    == "odom"
                ):

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

    if not command_motion_times:
        raise RuntimeError(
            "No se encontraron comandos de movimiento."
        )

    start_time = min(
        command_motion_times
    ) - 1.0

    end_time = max(
        command_motion_times
    ) + 2.0

    wheel_window = crop(
        wheel_samples,
        start_time,
        end_time,
    )

    ekf_window = crop(
        ekf_samples,
        start_time,
        end_time,
    )

    map_odom_samples.sort(
        key=lambda sample: sample[0]
    )

    map_timestamps = [
        sample[0]
        for sample in map_odom_samples
    ]

    slam_window = []

    for timestamp, x, y, yaw in ekf_window:
        map_odom = interpolate_transform(
            map_odom_samples,
            map_timestamps,
            timestamp,
        )

        if map_odom is None:
            continue

        slam_x, slam_y, slam_yaw = compose_pose(
            map_odom,
            (x, y, yaw),
        )

        slam_window.append(
            (
                timestamp,
                slam_x,
                slam_y,
                slam_yaw,
            )
        )

    trajectories = {
        "wheel": normalize_trajectory(
            wheel_window
        ),
        "ekf": normalize_trajectory(
            ekf_window
        ),
        "slam": normalize_trajectory(
            slam_window
        ),
    }

    output_directory = (
        bag_path + "_analysis"
    )

    os.makedirs(
        output_directory,
        exist_ok=True,
    )

    results = {}

    for name, samples in trajectories.items():
        if not samples:
            continue

        write_trajectory(
            os.path.join(
                output_directory,
                f"{name}_trajectory.csv",
            ),
            samples,
        )

        metrics = calculate_metrics(samples)

        if metrics is not None:
            results[name] = metrics

    write_summary(
        os.path.join(
            output_directory,
            "summary.csv",
        ),
        results,
    )

    create_plot(
        os.path.join(
            output_directory,
            "trajectories.png",
        ),
        trajectories,
    )

    print()
    print(
        f"Ventana del experimento: "
        f"{end_time - start_time:.3f} s"
    )

    for name, metrics in results.items():
        print()
        print(name.upper())
        print(
            f"  Error de cierre: "
            f"{metrics['closure_error_m']:.4f} m"
        )
        print(
            f"  Error yaw: "
            f"{metrics['yaw_error_deg']:.2f} grados"
        )
        print(
            f"  RMSE respecto al cuadrado: "
            f"{metrics['cross_track_rmse_m']:.4f} m"
        )
        print(
            f"  Longitud de trayectoria: "
            f"{metrics['path_length_m']:.4f} m"
        )

    print()
    print(
        f"Resultados guardados en: "
        f"{output_directory}"
    )


if __name__ == "__main__":
    main()
