#!/usr/bin/env python3
"""Export selected ROS 2 bag topics to JSONL without modifying the bag.

Run in a sourced ROS 2 Humble environment. The first argument may be a bag
directory or its metadata.yaml. Output messages preserve the ROS timestamp,
topic, ROS interface type, and recursively converted message payload.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPICS = {
    "/ground_truth/odom",
    "/odometry/filtered",
    "/amcl_pose",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/scan",
    "/tf",
    "/tf_static",
    "/navigate_to_pose/_action/status",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="Bag directory or metadata.yaml")
    parser.add_argument("output", type=Path, help="Output directory")
    parser.add_argument("--topic", action="append", dest="topics")
    args = parser.parse_args()

    bag_dir = args.bag.parent if args.bag.name == "metadata.yaml" else args.bag
    selected = set(args.topics or DEFAULT_TOPICS)
    args.output.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    handles = {}
    counts = {}

    try:
        while reader.has_next():
            topic, raw, timestamp_ns = reader.read_next()
            if topic not in selected:
                continue
            ros_type = topic_types[topic]
            msg = deserialize_message(raw, get_message(ros_type))
            safe_name = topic.strip("/").replace("/", "__") or "root"
            handle = handles.get(topic)
            if handle is None:
                handle = (args.output / f"{safe_name}.jsonl").open("w", encoding="utf-8")
                handles[topic] = handle
            record = {
                "timestamp_ns": timestamp_ns,
                "topic": topic,
                "type": ros_type,
                "message": message_to_ordereddict(msg),
            }
            handle.write(json.dumps(record, separators=(",", ":")) + "\n")
            counts[topic] = counts.get(topic, 0) + 1
    finally:
        for handle in handles.values():
            handle.close()

    manifest = {
        "bag": str(bag_dir),
        "selected_topics": sorted(selected),
        "exported_message_counts": counts,
    }
    (args.output / "export_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
