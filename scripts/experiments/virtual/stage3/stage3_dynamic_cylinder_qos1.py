import omni.kit.app
import omni.usd
import rclpy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from pxr import Gf, UsdGeom

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


# ============================================================
# GEOMETRÍA DEL EXPERIMENTO
# ============================================================

CYLINDER_PRIM_PATH = "/World/World_C1_Cylinder"

TRIGGER_FORWARD_DISPLACEMENT = 0.120

CYLINDER_FINAL_X = 0.6091
CYLINDER_FINAL_Y = -0.0364
CYLINDER_FINAL_Z = 0.1900

CYLINDER_PARK_X = 0.6091
CYLINDER_PARK_Y = -0.0364
CYLINDER_PARK_Z = -1.0000


class Stage3DynamicCylinder:
    """Insert the Stage 3 cylinder using the latest GT odometry."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self.stage = omni.usd.get_context().get_stage()

        self.prim = self.stage.GetPrimAtPath(
            CYLINDER_PRIM_PATH
        )

        if not self.prim.IsValid():
            raise RuntimeError(
                f"No existe el prim: {CYLINDER_PRIM_PATH}"
            )

        self.node = rclpy.create_node(
            "stage3_dynamic_cylinder"
        )

        # Solo conserva la muestra más reciente.
        # Esto evita acumular mensajes antiguos de ground truth.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.odom_callback,
            sensor_qos,
        )

        self.displacement_pub = self.node.create_publisher(
            Float64,
            "/stage3/forward_displacement",
            10,
        )

        self.inserted_pub = self.node.create_publisher(
            Bool,
            "/stage3/cylinder_inserted",
            10,
        )

        self.trigger_displacement_pub = (
            self.node.create_publisher(
                Float64,
                "/stage3/trigger_displacement",
                10,
            )
        )

        self.update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self.on_update,
                name="stage3_dynamic_cylinder_qos1_update",
            )
        )

        self.initial_x = None
        self.current_displacement = 0.0
        self.trigger_displacement = -1.0
        self.inserted = False

        self.set_translation(
            CYLINDER_PARK_X,
            CYLINDER_PARK_Y,
            CYLINDER_PARK_Z,
        )

        print("================================================")
        print("STAGE 3 DYNAMIC CYLINDER ARMED — QOS DEPTH 1")
        print("================================================")
        print(f"Prim: {CYLINDER_PRIM_PATH}")
        print(
            "Trigger displacement: "
            f"{TRIGGER_FORWARD_DISPLACEMENT:.3f} m"
        )
        print("QoS: BEST_EFFORT, KEEP_LAST, depth=1")
        print(
            "Cylinder parked at: "
            f"({CYLINDER_PARK_X:.4f}, "
            f"{CYLINDER_PARK_Y:.4f}, "
            f"{CYLINDER_PARK_Z:.4f})"
        )
        print("Waiting for the latest /ground_truth/odom...")

    def get_translate_op(self):
        xformable = UsdGeom.Xformable(self.prim)

        for operation in xformable.GetOrderedXformOps():
            if (
                operation.GetOpType()
                == UsdGeom.XformOp.TypeTranslate
            ):
                return operation

        return xformable.AddTranslateOp()

    def set_translation(self, x, y, z):
        translate_op = self.get_translate_op()

        try:
            translate_op.Set(
                Gf.Vec3d(float(x), float(y), float(z))
            )
        except Exception:
            translate_op.Set(
                Gf.Vec3f(float(x), float(y), float(z))
            )

    def publish_state(self):
        displacement_msg = Float64()
        displacement_msg.data = float(
            self.current_displacement
        )

        inserted_msg = Bool()
        inserted_msg.data = bool(
            self.inserted
        )

        trigger_msg = Float64()
        trigger_msg.data = float(
            self.trigger_displacement
        )

        self.displacement_pub.publish(
            displacement_msg
        )

        self.inserted_pub.publish(
            inserted_msg
        )

        self.trigger_displacement_pub.publish(
            trigger_msg
        )

    def odom_callback(self, msg):
        current_x = float(
            msg.pose.pose.position.x
        )

        if self.initial_x is None:
            self.initial_x = current_x
            self.current_displacement = 0.0

            print(
                "Ground-truth initial X registered: "
                f"{self.initial_x:.6f} m"
            )

            self.publish_state()
            return

        self.current_displacement = (
            current_x - self.initial_x
        )

        if (
            not self.inserted
            and self.current_displacement
            >= TRIGGER_FORWARD_DISPLACEMENT
        ):
            self.trigger_displacement = (
                self.current_displacement
            )

            self.set_translation(
                CYLINDER_FINAL_X,
                CYLINDER_FINAL_Y,
                CYLINDER_FINAL_Z,
            )

            self.inserted = True

            print("================================================")
            print("CYLINDER INSERTED — QOS DEPTH 1")
            print("================================================")
            print(
                "Trigger displacement: "
                f"{self.trigger_displacement:.6f} m"
            )
            print(
                "Ground-truth X used: "
                f"{current_x:.6f} m"
            )
            print(
                "Cylinder position: "
                f"({CYLINDER_FINAL_X:.4f}, "
                f"{CYLINDER_FINAL_Y:.4f}, "
                f"{CYLINDER_FINAL_Z:.4f})"
            )

        self.publish_state()

    def on_update(self, event):
        if not rclpy.ok():
            return

        # Procesa todos los callbacks disponibles sin bloquear.
        # Con depth=1 nunca se acumula una cola antigua.
        for _ in range(4):
            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

    def stop(self):
        try:
            self.update_subscription = None
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass


# Cerrar la instancia anterior antes de crear una nueva.
if "_stage3_dynamic_cylinder_instance" in globals():
    try:
        _stage3_dynamic_cylinder_instance.stop()
    except Exception:
        pass

_stage3_dynamic_cylinder_instance = Stage3DynamicCylinder()
