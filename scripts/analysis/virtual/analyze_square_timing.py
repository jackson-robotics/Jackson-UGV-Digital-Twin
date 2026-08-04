#!/usr/bin/env python3
"""Compare square-test timing directly from ROS 2 bag data.

The motion window is defined identically for every bag: from the first
non-zero /cmd_vel message through the first zero /cmd_vel message after the
last non-zero command. Durations are reported in bag-record time and, when
/clock exists, in simulation time.
"""

import argparse
import bisect
import math
import os
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


EPS = 1.0e-4


def is_motion(v, w):
    return abs(v) > EPS or abs(w) > EPS


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def clock_seconds(msg):
    return float(msg.clock.sec) + float(msg.clock.nanosec) * 1.0e-9


def analyze(path):
    reader = open_reader(path)
    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    wanted = {
        t
        for t in (
            "/cmd_vel",
            "/clock",
            "/ground_truth/odom",
            "/odometry/filtered",
            "/wheel/odom",
        )
        if t in topic_types
    }
    classes = {t: get_message(topic_types[t]) for t in wanted}

    cmd = []
    clocks = []
    odom = {t: [] for t in wanted if t.endswith("odom") or t == "/odometry/filtered"}

    first_bag_ns = None
    last_bag_ns = None
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if first_bag_ns is None:
            first_bag_ns = stamp_ns
        last_bag_ns = stamp_ns
        if topic not in wanted:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == "/cmd_vel":
            cmd.append((stamp_ns, float(msg.linear.x), float(msg.angular.z)))
        elif topic == "/clock":
            clocks.append((stamp_ns, clock_seconds(msg)))
        else:
            odom[topic].append(
                (
                    stamp_ns,
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )
            )

    if not cmd:
        raise RuntimeError("The bag does not contain /cmd_vel")

    moving_indices = [i for i, (_, v, w) in enumerate(cmd) if is_motion(v, w)]
    if not moving_indices:
        raise RuntimeError("No non-zero /cmd_vel messages were found")

    first_i = moving_indices[0]
    last_i = moving_indices[-1]
    end_i = last_i + 1 if last_i + 1 < len(cmd) else last_i
    start_ns = cmd[first_i][0]
    end_ns = cmd[end_i][0]
    if end_ns <= start_ns and last_i > first_i:
        end_ns = cmd[last_i][0]

    straight_s = 0.0
    turn_s = 0.0
    stopped_s = 0.0
    cmd_distance = 0.0
    cmd_rotation = 0.0
    max_v = 0.0
    max_w = 0.0
    linear_weight = 0.0
    angular_weight = 0.0

    for i in range(first_i, min(end_i, len(cmd) - 1)):
        t_ns, v, w = cmd[i]
        next_ns = min(cmd[i + 1][0], end_ns)
        dt = max(0.0, (next_ns - t_ns) * 1.0e-9)
        max_v = max(max_v, abs(v))
        max_w = max(max_w, abs(w))
        cmd_distance += abs(v) * dt
        cmd_rotation += abs(w) * dt
        if abs(v) > EPS:
            straight_s += dt
            linear_weight += abs(v) * dt
        elif abs(w) > EPS:
            turn_s += dt
            angular_weight += abs(w) * dt
        else:
            stopped_s += dt

    record_duration = (end_ns - start_ns) * 1.0e-9

    sim_duration = None
    if clocks:
        clock_bag_times = [x[0] for x in clocks]

        def sim_at(bag_ns):
            j = bisect.bisect_right(clock_bag_times, bag_ns) - 1
            j = max(0, min(j, len(clocks) - 1))
            return clocks[j][1]

        sim_duration = sim_at(end_ns) - sim_at(start_ns)

    reference_topic = None
    for candidate in ("/ground_truth/odom", "/odometry/filtered", "/wheel/odom"):
        if odom.get(candidate):
            reference_topic = candidate
            break

    measured_linear = []
    measured_turn = []
    if reference_topic:
        cmd_times = [x[0] for x in cmd]
        for stamp_ns, vx, wz in odom[reference_topic]:
            if stamp_ns < start_ns or stamp_ns > end_ns:
                continue
            j = bisect.bisect_right(cmd_times, stamp_ns) - 1
            if j < 0:
                continue
            _, cv, cw = cmd[j]
            if abs(cv) > EPS:
                measured_linear.append(abs(vx))
            elif abs(cw) > EPS:
                measured_turn.append(abs(wz))

    print("\n" + os.path.basename(os.path.abspath(path)))
    print(f"  Complete bag duration:       {(last_bag_ns-first_bag_ns)*1e-9:8.3f} s")
    print(f"  Motion window (bag time):    {record_duration:8.3f} s")
    if sim_duration is not None:
        print(f"  Motion window (/clock):      {sim_duration:8.3f} s")
        if record_duration > 0.0:
            print(f"  Mean real-time factor:       {sim_duration/record_duration:8.3f}")
    print(f"  Linear-command time:         {straight_s:8.3f} s")
    print(f"  Pure-turn-command time:      {turn_s:8.3f} s")
    print(f"  Zero-command time in window: {stopped_s:8.3f} s")
    print(f"  Max commanded linear speed:  {max_v:8.4f} m/s")
    print(f"  Mean commanded linear speed: {linear_weight/straight_s if straight_s else math.nan:8.4f} m/s")
    print(f"  Max commanded angular speed: {max_w:8.4f} rad/s")
    print(f"  Mean pure-turn command:      {angular_weight/turn_s if turn_s else math.nan:8.4f} rad/s")
    print(f"  Integrated |linear command|: {cmd_distance:8.4f} m")
    print(f"  Integrated |angular command|:{cmd_rotation:8.4f} rad")
    if reference_topic:
        print(f"  Velocity reference:          {reference_topic}")
        if measured_linear:
            print(f"  Mean measured |vx|, straights:{statistics.fmean(measured_linear):7.4f} m/s")
            print(f"  Max measured |vx|, straights: {max(measured_linear):7.4f} m/s")
        if measured_turn:
            print(f"  Mean measured |wz|, turns:    {statistics.fmean(measured_turn):7.4f} rad/s")
            print(f"  Max measured |wz|, turns:     {max(measured_turn):7.4f} rad/s")


def main():
    parser = argparse.ArgumentParser(
        description="Measure comparable /cmd_vel timing in ROS 2 square-test bags."
    )
    parser.add_argument("bags", nargs="+", help="One or more ROS 2 bag directories")
    args = parser.parse_args()
    for bag in args.bags:
        analyze(os.path.expanduser(bag))


if __name__ == "__main__":
    main()
