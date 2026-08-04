import csv
import math
import os
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
import omni.kit.app

from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from isaacsim.core.prims import SingleXFormPrim


ROBOT_PATH = "/World/Jackson_mobile_robot"
ODOM_TOPIC = "/odometry/filtered"

# Puede definirse antes de cargar el archivo:
# _jackson_requested_run_id = "R01"
RUN_ID = globals().get("_jackson_requested_run_id", "TEST")

GAP_THRESHOLD_S = 0.150
FLUSH_EVERY_N_SAMPLES = 20

RESULTS_DIRECTORY = os.path.expanduser(
    "~/jackson_dt_sync/results/mirror"
)


# ============================================================
# Limpiar una ejecución anterior
# ============================================================
if "_jackson_update_subscription" in globals():
    try:
        _jackson_update_subscription.unsubscribe()
    except Exception:
        pass

if "_jackson_ros_node" in globals():
    try:
        _jackson_ros_node.destroy_node()
    except Exception:
        pass

if "_jackson_csv_file" in globals():
    try:
        if _jackson_csv_file is not None:
            _jackson_csv_file.flush()
            _jackson_csv_file.close()
    except Exception:
        pass


# ============================================================
# Funciones matemáticas
# ============================================================
def yaw_from_ros_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_multiply(q1, q2):
    """
    Quaternion scalar-first: [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ],
        dtype=np.float64,
    )


def rotate_vector_by_quaternion(vector, quaternion):
    quaternion = quaternion / np.linalg.norm(quaternion)

    w = quaternion[0]
    xyz = quaternion[1:]

    return (
        2.0 * np.dot(xyz, vector) * xyz
        + (w*w - np.dot(xyz, xyz)) * vector
        + 2.0 * w * np.cross(xyz, vector)
    )


def heading_from_orientation(orientation):
    forward = rotate_vector_by_quaternion(
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
        orientation,
    )

    return math.atan2(forward[1], forward[0])


# ============================================================
# Robot virtual
# ============================================================
_jackson_robot = SingleXFormPrim(
    prim_path=ROBOT_PATH,
    name="jackson_mirror_robot",
)

virtual_position_0, virtual_orientation_0 = (
    _jackson_robot.get_world_pose()
)

virtual_position_0 = np.asarray(
    virtual_position_0,
    dtype=np.float64,
).copy()

virtual_orientation_0 = np.asarray(
    virtual_orientation_0,
    dtype=np.float64,
).copy()

virtual_orientation_0 /= np.linalg.norm(
    virtual_orientation_0
)

virtual_heading_0 = heading_from_orientation(
    virtual_orientation_0
)


# ============================================================
# Archivo CSV
# ============================================================
os.makedirs(RESULTS_DIRECTORY, exist_ok=True)

_start_utc = datetime.now(timezone.utc)
_timestamp_text = _start_utc.strftime("%Y%m%dT%H%M%S_%fZ")

_jackson_csv_path = os.path.join(
    RESULTS_DIRECTORY,
    f"jackson_mirror_{RUN_ID}_{_timestamp_text}.csv",
)

_jackson_csv_file = open(
    _jackson_csv_path,
    "x",
    newline="",
    encoding="utf-8",
)

_jackson_csv_writer = csv.writer(_jackson_csv_file)

_jackson_csv_writer.writerow(
    [
        "run_id",
        "sample_id",
        "reference_id",
        "ros_stamp_ns",
        "legion_receive_ns",
        "isaac_apply_ns",
        "ros_to_receive_ms",
        "receive_to_apply_ms",
        "ros_to_apply_ms",
        "message_interval_ms",
        "receive_frequency_hz",
        "communication_gap",
        "physical_x_m",
        "physical_y_m",
        "physical_yaw_rad",
        "physical_yaw_deg",
        "linear_velocity_x_mps",
        "angular_velocity_z_radps",
        "relative_x_body_m",
        "relative_y_body_m",
        "relative_yaw_rad",
        "relative_yaw_deg",
        "virtual_x_m",
        "virtual_y_m",
        "virtual_z_m",
        "virtual_yaw_rad",
        "virtual_yaw_deg",
        "virtual_qw",
        "virtual_qx",
        "virtual_qy",
        "virtual_qz",
    ]
)

_jackson_csv_file.flush()


# ============================================================
# ROS 2
# ============================================================
if not rclpy.ok():
    rclpy.init()

_jackson_ros_node = rclpy.create_node(
    "jackson_isaac_mirror_logger"
)

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_jackson_latest_sample = None
_jackson_received_counter = 0
_jackson_processed_counter = 0
_jackson_previous_receive_ns = None

_jackson_reference_set = False
_jackson_reference_id = 0
_jackson_x0 = 0.0
_jackson_y0 = 0.0
_jackson_yaw0 = 0.0

_jackson_running = True


def odometry_callback(msg):
    global _jackson_latest_sample
    global _jackson_received_counter

    _jackson_received_counter += 1

    _jackson_latest_sample = {
        "sequence": _jackson_received_counter,
        "receive_ns": time.time_ns(),
        "message": msg,
    }


_jackson_odom_subscription = (
    _jackson_ros_node.create_subscription(
        Odometry,
        ODOM_TOPIC,
        odometry_callback,
        qos,
    )
)


# ============================================================
# Actualización y registro
# ============================================================
def update_mirror(event):
    global _jackson_processed_counter
    global _jackson_previous_receive_ns

    global _jackson_reference_set
    global _jackson_reference_id
    global _jackson_x0
    global _jackson_y0
    global _jackson_yaw0

    if not _jackson_running:
        return

    rclpy.spin_once(
        _jackson_ros_node,
        timeout_sec=0.0,
    )

    sample = _jackson_latest_sample

    if sample is None:
        return

    sequence = sample["sequence"]

    # Impide registrar repetidamente el mismo mensaje.
    if sequence == _jackson_processed_counter:
        return

    _jackson_processed_counter = sequence

    msg = sample["message"]
    receive_ns = sample["receive_ns"]

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    yaw = yaw_from_ros_quaternion(
        msg.pose.pose.orientation
    )

    ros_stamp_ns = (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )

    # Primera muestra: referencia física.
    if not _jackson_reference_set:
        _jackson_x0 = x
        _jackson_y0 = y
        _jackson_yaw0 = yaw

        _jackson_reference_id += 1
        _jackson_reference_set = True

        print("====================================")
        print("JACKSON MIRROR: REFERENCE ESTABLISHED")
        print("Reference ID:", _jackson_reference_id)
        print("Physical initial pose:")
        print("  x =", _jackson_x0)
        print("  y =", _jackson_y0)
        print("  yaw =", _jackson_yaw0)
        print("Virtual initial position:")
        print(" ", virtual_position_0)
        print("====================================")

    dx_world = x - _jackson_x0
    dy_world = y - _jackson_y0

    # Odom físico -> sistema inicial del robot físico.
    c0 = math.cos(_jackson_yaw0)
    s0 = math.sin(_jackson_yaw0)

    dx_body = c0 * dx_world + s0 * dy_world
    dy_body = -s0 * dx_world + c0 * dy_world

    # Sistema inicial físico -> orientación inicial virtual.
    cv = math.cos(virtual_heading_0)
    sv = math.sin(virtual_heading_0)

    dx_virtual = cv * dx_body - sv * dy_body
    dy_virtual = sv * dx_body + cv * dy_body

    new_position = virtual_position_0.copy()
    new_position[0] += dx_virtual
    new_position[1] += dy_virtual

    delta_yaw = wrap_angle(yaw - _jackson_yaw0)

    yaw_quaternion = np.array(
        [
            math.cos(delta_yaw / 2.0),
            0.0,
            0.0,
            math.sin(delta_yaw / 2.0),
        ],
        dtype=np.float64,
    )

    new_orientation = quaternion_multiply(
        yaw_quaternion,
        virtual_orientation_0,
    )

    new_orientation /= np.linalg.norm(
        new_orientation
    )

    _jackson_robot.set_world_pose(
        position=new_position,
        orientation=new_orientation,
    )

    apply_ns = time.time_ns()

    # Métricas temporales.
    if ros_stamp_ns > 0:
        ros_to_receive_ms = (
            receive_ns - ros_stamp_ns
        ) / 1_000_000.0

        ros_to_apply_ms = (
            apply_ns - ros_stamp_ns
        ) / 1_000_000.0
    else:
        ros_to_receive_ms = float("nan")
        ros_to_apply_ms = float("nan")

    receive_to_apply_ms = (
        apply_ns - receive_ns
    ) / 1_000_000.0

    if _jackson_previous_receive_ns is None:
        interval_ms = float("nan")
        frequency_hz = float("nan")
        communication_gap = 0
    else:
        interval_s = (
            receive_ns - _jackson_previous_receive_ns
        ) / 1_000_000_000.0

        interval_ms = interval_s * 1000.0

        frequency_hz = (
            1.0 / interval_s
            if interval_s > 0.0
            else float("nan")
        )

        communication_gap = int(
            interval_s > GAP_THRESHOLD_S
        )

    _jackson_previous_receive_ns = receive_ns

    virtual_yaw = wrap_angle(
        virtual_heading_0 + delta_yaw
    )

    _jackson_csv_writer.writerow(
        [
            RUN_ID,
            sequence,
            _jackson_reference_id,
            ros_stamp_ns,
            receive_ns,
            apply_ns,
            ros_to_receive_ms,
            receive_to_apply_ms,
            ros_to_apply_ms,
            interval_ms,
            frequency_hz,
            communication_gap,
            x,
            y,
            yaw,
            math.degrees(yaw),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
            dx_body,
            dy_body,
            delta_yaw,
            math.degrees(delta_yaw),
            new_position[0],
            new_position[1],
            new_position[2],
            virtual_yaw,
            math.degrees(virtual_yaw),
            new_orientation[0],
            new_orientation[1],
            new_orientation[2],
            new_orientation[3],
        ]
    )

    if sequence % FLUSH_EVERY_N_SAMPLES == 0:
        _jackson_csv_file.flush()

    if sequence % 60 == 0:
        print(
            "Mirror:",
            f"x={dx_body:.3f} m,",
            f"y={dy_body:.3f} m,",
            f"yaw={math.degrees(delta_yaw):.2f} deg,",
            f"end-to-end={ros_to_apply_ms:.2f} ms",
        )


# ============================================================
# Controles
# ============================================================
def jackson_mirror_reset_reference():
    """
    Establece una nueva correspondencia entre la pose física
    actual y la pose virtual actual.
    """
    global _jackson_reference_set
    global virtual_position_0
    global virtual_orientation_0
    global virtual_heading_0

    virtual_position_0, virtual_orientation_0 = (
        _jackson_robot.get_world_pose()
    )

    virtual_position_0 = np.asarray(
        virtual_position_0,
        dtype=np.float64,
    ).copy()

    virtual_orientation_0 = np.asarray(
        virtual_orientation_0,
        dtype=np.float64,
    ).copy()

    virtual_orientation_0 /= np.linalg.norm(
        virtual_orientation_0
    )

    virtual_heading_0 = heading_from_orientation(
        virtual_orientation_0
    )

    _jackson_reference_set = False

    print("JACKSON MIRROR: waiting for new reference")


def jackson_mirror_stop():
    """
    Detiene el seguidor y cierra el CSV de forma segura.
    No ejecuta rclpy.shutdown().
    """
    global _jackson_running
    global _jackson_update_subscription

    _jackson_running = False

    try:
        _jackson_update_subscription.unsubscribe()
    except Exception as exc:
        print("Update subscription warning:", exc)

    try:
        _jackson_ros_node.destroy_node()
    except Exception as exc:
        print("ROS node warning:", exc)

    try:
        _jackson_csv_file.flush()
        _jackson_csv_file.close()
    except Exception as exc:
        print("CSV close warning:", exc)

    _jackson_update_subscription = None

    print("JACKSON MIRROR LOGGER STOPPED")
    print("CSV saved at:")
    print(_jackson_csv_path)


_jackson_update_subscription = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        update_mirror,
        name="jackson_mirror_logger_update",
    )
)

print("JACKSON MIRROR LOGGER ACTIVE")
print("Run ID:", RUN_ID)
print("Topic:", ODOM_TOPIC)
print("Robot:", ROBOT_PATH)
print("CSV:", _jackson_csv_path)
print("Keep the Isaac Sim timeline stopped.")
print("Waiting for the first odometry sample...")
