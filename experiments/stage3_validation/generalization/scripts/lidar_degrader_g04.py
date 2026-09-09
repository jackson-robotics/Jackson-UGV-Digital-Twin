#!/usr/bin/env python3

import math
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


INPUT_TOPIC = "/scan"
OUTPUT_TOPIC = "/scan_degraded"

ADDITIONAL_NOISE_SIGMA_M = 0.030
DROPOUT_PROBABILITY = 0.050

BASE_SEED = 20260827

AUDIT_FILE = (
    "/home/carlos/jackson_dt_ws/"
    "revision_r1_5_generalization/runs/"
    "G04_LIDAR_DEGRADED/"
    "LIDAR_DEGRADATION_SETUP.txt"
)


class LidarDegrader(Node):

    def __init__(self):
        super().__init__("stage3_lidar_degrader_g04")

        self.pub = self.create_publisher(
            LaserScan,
            OUTPUT_TOPIC,
            qos_profile_sensor_data,
        )

        self.sub = self.create_subscription(
            LaserScan,
            INPUT_TOPIC,
            self.callback,
            qos_profile_sensor_data,
        )

        os.makedirs(
            os.path.dirname(AUDIT_FILE),
            exist_ok=True
        )

        with open(AUDIT_FILE, "w", encoding="utf-8") as f:
            f.write(
                "JACKSON STAGE 3 — G04 LiDAR DEGRADATION\n"
            )
            f.write("=" * 70 + "\n")
            f.write(f"Input topic: {INPUT_TOPIC}\n")
            f.write(f"Output topic: {OUTPUT_TOPIC}\n")
            f.write(
                "Additional Gaussian range noise sigma: "
                f"{ADDITIONAL_NOISE_SIGMA_M:.3f} m\n"
            )
            f.write(
                "Per-beam dropout probability: "
                f"{100*DROPOUT_PROBABILITY:.1f}%\n"
            )
            f.write(f"Base deterministic seed: {BASE_SEED}\n")
            f.write(
                "Per-scan RNG seed: deterministic function "
                "of ROS scan timestamp.\n"
            )
            f.write(
                "Dropped valid beams are published as +inf.\n"
            )
            f.write(
                "The raw /scan topic remains unchanged and "
                "is recorded together with /scan_degraded.\n"
            )

        print("G04 LiDAR degrader ready")
        print(
            f"Input={INPUT_TOPIC}, Output={OUTPUT_TOPIC}"
        )
        print(
            f"Additional sigma={ADDITIONAL_NOISE_SIGMA_M:.3f} m, "
            f"dropout={100*DROPOUT_PROBABILITY:.1f}%"
        )
        print(f"Base seed={BASE_SEED}")

    def callback(self, msg):

        stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

        seed = BASE_SEED ^ (stamp_ns & 0xFFFFFFFFFFFFFFFF)
        rng = random.Random(seed)

        out = LaserScan()

        out.header = msg.header

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        degraded = []

        for r in msg.ranges:

            r = float(r)

            if not math.isfinite(r):
                degraded.append(r)
                continue

            if r < msg.range_min or r > msg.range_max:
                degraded.append(r)
                continue

            if rng.random() < DROPOUT_PROBABILITY:
                degraded.append(float("inf"))
                continue

            rn = r + rng.gauss(
                0.0,
                ADDITIONAL_NOISE_SIGMA_M
            )

            if rn < msg.range_min:
                rn = msg.range_min

            if rn > msg.range_max:
                rn = float("inf")

            degraded.append(rn)

        out.ranges = degraded
        out.intensities = list(msg.intensities)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarDegrader()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
