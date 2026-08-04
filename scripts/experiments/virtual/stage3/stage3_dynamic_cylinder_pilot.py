import math

import omni.kit.app
import omni.usd
import rclpy

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from pxr import Gf, UsdGeom


# ============================================================
# CAMBIAR ÚNICAMENTE ESTA RUTA
# ============================================================

CYLINDER_PRIM_PATH = "/World/World_C1_Cylinder"


# ============================================================
# PARÁMETROS DEL EXPERIMENTO
# ============================================================

TRIGGER_FORWARD_DISPLACEMENT = 0.120

CYLINDER_FINAL_X = 0.6091
CYLINDER_FINAL_Y = -0.0364
CYLINDER_FINAL_Z = 0.1900

CYLINDER_PARK_X = 0.6091
CYLINDER_PARK_Y = -0.0364
CYLINDER_PARK_Z = -1.0000


class Stage3DynamicCylinder:
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

        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self.odom_callback,
            10,
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

        self.state_timer = self.node.create_timer(
            0.05,
            self.publish_state,
        )

        self.update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self.on_update,
                name="stage3_dynamic_cylinder_update",
            )
        )

        self.initial_x = None
        self.current_displacement = 0.0
        self.inserted = False

        self.set_translation(
            CYLINDER_PARK_X,
            CYLINDER_PARK_Y,
            CYLINDER_PARK_Z,
        )

        print("================================================")
        print("STAGE 3 DYNAMIC CYLINDER ARMED")
        print("================================================")
        print(f"Prim: {CYLINDER_PRIM_PATH}")
        print(
            "Trigger displacement: "
            f"{TRIGGER_FORWARD_DISPLACEMENT:.3f} m"
        )
        print(
            "Initial cylinder position: "
            f"({CYLINDER_PARK_X:.4f}, "
            f"{CYLINDER_PARK_Y:.4f}, "
            f"{CYLINDER_PARK_Z:.4f})"
        )
        print(
            "Final cylinder position: "
            f"({CYLINDER_FINAL_X:.4f}, "
            f"{CYLINDER_FINAL_Y:.4f}, "
            f"{CYLINDER_FINAL_Z:.4f})"
        )
        print("Waiting for /ground_truth/odom...")

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
            return

        self.current_displacement = (
            current_x - self.initial_x
        )

        if (
            not self.inserted
            and self.current_displacement
            >= TRIGGER_FORWARD_DISPLACEMENT
        ):
            self.set_translation(
                CYLINDER_FINAL_X,
                CYLINDER_FINAL_Y,
                CYLINDER_FINAL_Z,
            )

            self.inserted = True

            print("================================================")
            print("CYLINDER INSERTED")
            print("================================================")
            print(
                "Forward displacement: "
                f"{self.current_displacement:.6f} m"
            )
            print(
                "Current ground-truth X: "
                f"{current_x:.6f} m"
            )
            print(
                "Cylinder position: "
                f"({CYLINDER_FINAL_X:.4f}, "
                f"{CYLINDER_FINAL_Y:.4f}, "
                f"{CYLINDER_FINAL_Z:.4f})"
            )

    def publish_state(self):
        displacement_message = Float64()
        displacement_message.data = float(
            self.current_displacement
        )

        inserted_message = Bool()
        inserted_message.data = bool(
            self.inserted
        )

        self.displacement_pub.publish(
            displacement_message
        )

        self.inserted_pub.publish(
            inserted_message
        )

    def on_update(self, event):
        if rclpy.ok():
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


# Cerrar una instancia anterior si el script se vuelve a ejecutar.
if "_stage3_dynamic_cylinder_instance" in globals():
    try:
        _stage3_dynamic_cylinder_instance.stop()
    except Exception:
        pass

_stage3_dynamic_cylinder_instance = Stage3DynamicCylinder()
