import math
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


# ---------------------------------------------------------
# Limpiar una ejecución anterior del seguidor, si existiera
# ---------------------------------------------------------
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


# ---------------------------------------------------------
# Funciones matemáticas
# ---------------------------------------------------------
def yaw_from_ros_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_multiply(q1, q2):
    """
    Quaternion scalar-first:
    [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def rotate_vector_by_quaternion(v, q):
    """
    Rota el vector v usando q=[w,x,y,z].
    """
    q = q / np.linalg.norm(q)
    w = q[0]
    xyz = q[1:]

    return (
        2.0 * np.dot(xyz, v) * xyz
        + (w*w - np.dot(xyz, xyz)) * v
        + 2.0 * w * np.cross(xyz, v)
    )


# ---------------------------------------------------------
# Inicialización del robot virtual
# ---------------------------------------------------------
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

# Dirección frontal inicial del robot virtual proyectada en XY
virtual_forward = rotate_vector_by_quaternion(
    np.array([1.0, 0.0, 0.0]),
    virtual_orientation_0,
)

virtual_heading_0 = math.atan2(
    virtual_forward[1],
    virtual_forward[0],
)


# ---------------------------------------------------------
# ROS 2
# ---------------------------------------------------------
if not rclpy.ok():
    rclpy.init()

_jackson_ros_node = rclpy.create_node(
    "jackson_isaac_mirror_follower"
)

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_jackson_latest_odom = None
_jackson_reference_set = False
_jackson_x0 = 0.0
_jackson_y0 = 0.0
_jackson_yaw0 = 0.0
_jackson_update_counter = 0


def odometry_callback(msg):
    global _jackson_latest_odom
    _jackson_latest_odom = msg


_jackson_odom_subscription = (
    _jackson_ros_node.create_subscription(
        Odometry,
        "/odometry/filtered",
        odometry_callback,
        qos,
    )
)


# ---------------------------------------------------------
# Actualización continua dentro de Isaac Sim
# ---------------------------------------------------------
def update_mirror(event):
    global _jackson_reference_set
    global _jackson_x0
    global _jackson_y0
    global _jackson_yaw0
    global _jackson_update_counter

    rclpy.spin_once(
        _jackson_ros_node,
        timeout_sec=0.0,
    )

    msg = _jackson_latest_odom

    if msg is None:
        return

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw = yaw_from_ros_quaternion(
        msg.pose.pose.orientation
    )

    # Primera muestra: establecer referencia física
    if not _jackson_reference_set:
        _jackson_x0 = x
        _jackson_y0 = y
        _jackson_yaw0 = yaw
        _jackson_reference_set = True

        print("====================================")
        print("JACKSON MIRROR: REFERENCE ESTABLISHED")
        print("Physical initial pose:")
        print("  x =", _jackson_x0)
        print("  y =", _jackson_y0)
        print("  yaw =", _jackson_yaw0)
        print("Virtual initial position:")
        print(" ", virtual_position_0)
        print("Move the physical robot slowly.")
        print("====================================")
        return

    # Desplazamiento físico en el sistema odom
    dx_world = x - _jackson_x0
    dy_world = y - _jackson_y0

    # Convertirlo al sistema inicial del robot físico
    c0 = math.cos(_jackson_yaw0)
    s0 = math.sin(_jackson_yaw0)

    dx_body = c0 * dx_world + s0 * dy_world
    dy_body = -s0 * dx_world + c0 * dy_world

    # Aplicarlo según la orientación inicial virtual
    cv = math.cos(virtual_heading_0)
    sv = math.sin(virtual_heading_0)

    dx_virtual = cv * dx_body - sv * dy_body
    dy_virtual = sv * dx_body + cv * dy_body

    new_position = virtual_position_0.copy()
    new_position[0] += dx_virtual
    new_position[1] += dy_virtual

    # Giro relativo alrededor del eje vertical Z
    delta_yaw = wrap_angle(yaw - _jackson_yaw0)

    yaw_quaternion = np.array([
        math.cos(delta_yaw / 2.0),
        0.0,
        0.0,
        math.sin(delta_yaw / 2.0),
    ])

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

    _jackson_update_counter += 1

    if _jackson_update_counter % 60 == 0:
        print(
            "Mirror displacement:",
            f"x={dx_body:.3f} m,",
            f"y={dy_body:.3f} m,",
            f"yaw={math.degrees(delta_yaw):.2f} deg",
        )


_jackson_update_subscription = (
    omni.kit.app.get_app()
    .get_update_event_stream()
    .create_subscription_to_pop(
        update_mirror,
        name="jackson_mirror_update",
    )
)

print("JACKSON MIRROR FOLLOWER ACTIVE")
print("Topic: /odometry/filtered")
print("Robot:", ROBOT_PATH)
print("Keep the Isaac Sim timeline stopped.")
print("Waiting for the first odometry sample...")
