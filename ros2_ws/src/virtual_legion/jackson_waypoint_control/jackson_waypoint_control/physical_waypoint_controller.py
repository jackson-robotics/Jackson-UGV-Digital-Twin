#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al intervalo [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


class PhysicalWaypointController(Node):
    """
    Controlador físico estable para un cuadrado de 1 x 1 m.

    Entrada:
        /odom_physical

    Salida:
        /cmd_vel_physical

    Estrategia:
        ALIGN -> SETTLE -> DRIVE -> SETTLE -> siguiente lado

    Durante DRIVE no se aplican pequeñas correcciones angulares,
    porque la zona muerta de los motores provoca sobrecorrecciones.
    """

    def __init__(self):
        super().__init__("physical_waypoint_controller")

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel_physical",
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom_physical",
            self.odom_callback,
            20,
        )

        self.control_period = 0.05

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        # Pose actual.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Pose inicial.
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.odom_received = False
        self.initialized = False
        self.finished = False

        # Waypoints y orientaciones globales.
        self.waypoints = []
        self.segment_headings = []

        self.current_segment = 0

        # Estados:
        # ALIGN
        # SETTLE_AFTER_ALIGN
        # DRIVE
        # SETTLE_AFTER_DRIVE
        # FINAL_ALIGN
        # FINISHED
        self.state = "ALIGN"

        self.state_start_time = time.monotonic()

        # =====================================================
        # PARÁMETROS FÍSICOS
        # =====================================================

        # El robot detiene el tramo cuando está a 5 cm.
        self.position_tolerance = 0.05

        # Evita invertir constantemente el giro por pocos grados.
        self.alignment_tolerance = math.radians(8.0)

        # Solo se interrumpe un tramo si el error es realmente grande.
        self.emergency_heading_tolerance = math.radians(25.0)

        # Comandos ya comprobados con el robot físico.
        self.linear_command = 0.05
        self.angular_command = 0.30

        # Tiempo para que el robot termine de asentarse mecánicamente.
        self.settle_time = 0.70

        # Seguridad.
        self.maximum_segment_time = 35.0
        self.maximum_alignment_time = 20.0

        self.motion_start_time = time.monotonic()

        self.get_logger().info(
            "Controlador físico estable iniciado."
        )

        self.get_logger().info(
            "Esperando /odom_physical..."
        )

    def odom_callback(self, msg: Odometry):
        """Actualiza la pose desde la odometría física fusionada."""

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self.odom_received = True

        if not self.initialized:
            self.initialize_trajectory()

    def initialize_trajectory(self):
        """Construye el cuadrado respecto de la pose inicial."""

        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw

        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)

        local_waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]

        local_headings = [
            0.0,
            math.pi / 2.0,
            math.pi,
            -math.pi / 2.0,
        ]

        self.waypoints = []

        for local_x, local_y in local_waypoints:
            global_x = (
                self.start_x
                + local_x * cos_yaw
                - local_y * sin_yaw
            )

            global_y = (
                self.start_y
                + local_x * sin_yaw
                + local_y * cos_yaw
            )

            self.waypoints.append(
                (global_x, global_y)
            )

        self.segment_headings = [
            normalize_angle(
                self.start_yaw + local_heading
            )
            for local_heading in local_headings
        ]

        self.initialized = True
        self.state = "ALIGN"
        self.reset_state_timer()

        self.get_logger().info(
            f"Pose inicial: "
            f"x={self.start_x:.3f}, "
            f"y={self.start_y:.3f}, "
            f"yaw={math.degrees(self.start_yaw):.1f}°"
        )

        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(
                f"Tramo {index + 1}: "
                f"objetivo=({waypoint[0]:.3f}, "
                f"{waypoint[1]:.3f}), "
                f"rumbo={math.degrees(self.segment_headings[index]):.1f}°"
            )

    def control_loop(self):
        """Máquina de estados principal."""

        if (
            not self.odom_received
            or not self.initialized
            or self.finished
        ):
            return

        if self.state == "ALIGN":
            self.run_alignment()

        elif self.state == "SETTLE_AFTER_ALIGN":
            self.run_settle_after_alignment()

        elif self.state == "DRIVE":
            self.run_drive()

        elif self.state == "SETTLE_AFTER_DRIVE":
            self.run_settle_after_drive()

        elif self.state == "FINAL_ALIGN":
            self.run_final_alignment()

        elif self.state == "FINISHED":
            self.stop_robot()

    def run_alignment(self):
        """Alinea el robot con el rumbo fijo del tramo actual."""

        target_yaw = self.segment_headings[
            self.current_segment
        ]

        angle_error = normalize_angle(
            target_yaw - self.yaw
        )

        if abs(angle_error) <= self.alignment_tolerance:
            self.stop_robot()

            self.get_logger().info(
                f"Alineación del tramo "
                f"{self.current_segment + 1} completada: "
                f"yaw={math.degrees(self.yaw):.1f}°, "
                f"error={math.degrees(angle_error):.1f}°"
            )

            self.state = "SETTLE_AFTER_ALIGN"
            self.reset_state_timer()
            return

        if (
            self.state_elapsed()
            > self.maximum_alignment_time
        ):
            self.emergency_stop(
                "Tiempo máximo de alineación excedido."
            )
            return

        command = Twist()
        command.angular.z = math.copysign(
            self.angular_command,
            angle_error,
        )

        self.cmd_pub.publish(command)

    def run_settle_after_alignment(self):
        """Mantiene el robot detenido después del giro."""

        self.stop_robot()

        if self.state_elapsed() >= self.settle_time:
            self.state = "DRIVE"
            self.reset_state_timer()

            self.get_logger().info(
                f"Comenzando avance del tramo "
                f"{self.current_segment + 1}."
            )

    def run_drive(self):
        """Avanza recto hacia el waypoint actual."""

        target_x, target_y = self.waypoints[
            self.current_segment
        ]

        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.hypot(
            dx,
            dy,
        )

        target_yaw = self.segment_headings[
            self.current_segment
        ]

        heading_error = normalize_angle(
            target_yaw - self.yaw
        )

        if distance <= self.position_tolerance:
            self.stop_robot()

            self.get_logger().info(
                f"Tramo {self.current_segment + 1} completado: "
                f"error={distance:.3f} m, "
                f"pose=({self.x:.3f}, {self.y:.3f}), "
                f"yaw={math.degrees(self.yaw):.1f}°"
            )

            self.state = "SETTLE_AFTER_DRIVE"
            self.reset_state_timer()
            return

        if (
            abs(heading_error)
            > self.emergency_heading_tolerance
        ):
            self.stop_robot()

            self.get_logger().warn(
                f"Desviación de "
                f"{math.degrees(heading_error):.1f}°. "
                f"Realineando tramo "
                f"{self.current_segment + 1}."
            )

            self.state = "ALIGN"
            self.reset_state_timer()
            return

        if self.state_elapsed() > self.maximum_segment_time:
            self.emergency_stop(
                "Tiempo máximo del tramo excedido."
            )
            return

        command = Twist()

        # Avance completamente recto.
        command.linear.x = self.linear_command
        command.angular.z = 0.0

        self.cmd_pub.publish(command)

    def run_settle_after_drive(self):
        """Pausa después de alcanzar un waypoint."""

        self.stop_robot()

        if self.state_elapsed() < self.settle_time:
            return

        self.current_segment += 1

        if self.current_segment >= len(
            self.waypoints
        ):
            self.state = "FINAL_ALIGN"

            self.get_logger().info(
                "Posición final alcanzada. "
                "Comenzando orientación final."
            )
        else:
            self.state = "ALIGN"

            self.get_logger().info(
                f"Preparando tramo "
                f"{self.current_segment + 1}."
            )

        self.reset_state_timer()

    def run_final_alignment(self):
        """Recupera aproximadamente la orientación inicial."""

        angle_error = normalize_angle(
            self.start_yaw - self.yaw
        )

        if abs(angle_error) <= self.alignment_tolerance:
            self.stop_robot()
            self.finish_trajectory()
            return

        if (
            self.state_elapsed()
            > self.maximum_alignment_time
        ):
            self.emergency_stop(
                "Tiempo máximo de alineación final excedido."
            )
            return

        command = Twist()
        command.angular.z = math.copysign(
            self.angular_command,
            angle_error,
        )

        self.cmd_pub.publish(command)

    def finish_trajectory(self):
        """Registra los resultados finales."""

        self.finished = True
        self.state = "FINISHED"

        closure_error = math.hypot(
            self.x - self.start_x,
            self.y - self.start_y,
        )

        angular_error = math.degrees(
            normalize_angle(
                self.yaw - self.start_yaw
            )
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "TRAYECTORIA FÍSICA COMPLETADA"
        )

        self.get_logger().info(
            f"Pose final: "
            f"x={self.x:.3f}, "
            f"y={self.y:.3f}, "
            f"yaw={math.degrees(self.yaw):.1f}°"
        )

        self.get_logger().info(
            f"Error de cierre estimado: "
            f"{closure_error:.3f} m"
        )

        self.get_logger().info(
            f"Error angular estimado: "
            f"{angular_error:.1f}°"
        )

        self.get_logger().info(
            "========================================"
        )

    def emergency_stop(self, reason: str):
        """Detiene el controlador ante una condición insegura."""

        self.stop_robot()
        self.finished = True
        self.state = "FINISHED"

        self.get_logger().error(
            f"PARADA DE SEGURIDAD: {reason}"
        )

    def reset_state_timer(self):
        self.state_start_time = time.monotonic()

    def state_elapsed(self) -> float:
        return (
            time.monotonic()
            - self.state_start_time
        )

    def stop_robot(self):
        if rclpy.ok():
            self.cmd_pub.publish(
                Twist()
            )


def main(args=None):
    rclpy.init(args=args)

    node = PhysicalWaypointController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            for _ in range(10):
                node.stop_robot()
                time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
