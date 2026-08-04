#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation

    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PhysicalSquareSimple(Node):

    def __init__(self):
        super().__init__("physical_square_simple")

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel_physical",
            10,
        )

        self.create_subscription(
            Odometry,
            "/odom_physical",
            self.odom_callback,
            20,
        )

        self.create_subscription(
            Imu,
            "/imu/data_corrected",
            self.imu_callback,
            50,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        # Pose actual
        self.odom_msg = None
        self.imu_received = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Pose inicial
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # Máquina de estados
        self.initialized = False
        self.finished = False
        self.state = "WAITING"

        self.side_number = 1

        # Referencia del tramo
        self.segment_start_x = 0.0
        self.segment_start_y = 0.0
        self.segment_heading = 0.0

        # Tiempos
        self.state_start_time = time.monotonic()
        self.pause_start_time = 0.0

        # Integración IMU
        self.integrate_imu = False
        self.last_imu_stamp = None

        # Ángulo acumulado durante cada tramo recto.
        self.forward_angle_rad = 0.0

        # Ángulo acumulado durante cada giro.
        self.turn_angle_rad = 0.0

        # =====================================================
        # PARÁMETROS DE AVANCE
        # =====================================================

        self.side_length = 1.00
        self.distance_tolerance = 0.03

        self.linear_speed = 0.05

        self.heading_kp = 0.45
        self.max_angular_correction = 0.06
        self.heading_deadband = math.radians(1.0)

        # Si por cualquier razón el rumbo se aparta demasiado,
        # se detiene en vez de continuar formando un círculo.
        self.maximum_heading_error = math.radians(20.0)

        # =====================================================
        # PARÁMETROS DE GIRO
        # =====================================================

        self.target_turn_deg = 90.0

        # Resultado validado:
        # parada a 82° -> giro final aproximado de 87° a 94°.
        self.stop_turn_deg = 82.0
        self.stop_turn_rad = math.radians(
            self.stop_turn_deg
        )

        # Con el driver actual:
        # angular.z negativo -> giro antihorario.
        self.turn_command = -0.10

        # =====================================================
        # PAUSAS Y SEGURIDAD
        # =====================================================

        self.pause_before_turn = 1.0
        self.turn_settle_duration = 2.0
        self.pause_before_forward = 1.0

        self.maximum_forward_time = 35.0
        self.maximum_turn_time = 15.0

        self.get_logger().info(
            "Controlador físico combinado IMU+EKF iniciado."
        )

        self.get_logger().info(
            "Esperando /odom_physical e /imu/data_corrected..."
        )

        self.get_logger().info(
            f"Longitud de cada lado: {self.side_length:.2f} m"
        )

        self.get_logger().info(
            "Convención física del driver: "
            "angular positivo=horario; negativo=antihorario."
        )

    # =========================================================
    # CALLBACKS
    # =========================================================

    def odom_callback(self, msg: Odometry):
        self.odom_msg = msg

        self.x = float(
            msg.pose.pose.position.x
        )

        self.y = float(
            msg.pose.pose.position.y
        )

        self.yaw = yaw_from_odom(msg)

        if (
            not self.initialized
            and self.imu_received
        ):
            self.initialize_square()

    def imu_callback(self, msg: Imu):
        self.imu_received = True

        if (
            not self.initialized
            and self.odom_msg is not None
        ):
            self.initialize_square()

        stamp = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1.0e-9
        )

        if self.last_imu_stamp is None:
            self.last_imu_stamp = stamp
            return

        dt = stamp - self.last_imu_stamp
        self.last_imu_stamp = stamp

        if dt <= 0.0 or dt > 0.20:
            return

        gyro_z = float(
            msg.angular_velocity.z
        )

        if self.state == "FORWARD":
            self.forward_angle_rad += gyro_z * dt

        elif self.integrate_imu:
            self.turn_angle_rad += gyro_z * dt

    # =========================================================
    # INICIALIZACIÓN
    # =========================================================

    def initialize_square(self):
        if self.initialized:
            return

        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_yaw = self.yaw

        self.segment_start_x = self.x
        self.segment_start_y = self.y
        self.segment_heading = self.yaw

        self.side_number = 1

        self.forward_angle_rad = 0.0
        self.last_imu_stamp = None

        self.state = "FORWARD"

        self.initialized = True
        self.reset_state_timer()

        self.get_logger().info(
            f"Pose inicial: "
            f"x={self.initial_x:.3f}, "
            f"y={self.initial_y:.3f}, "
            f"yaw={math.degrees(self.initial_yaw):.2f}°"
        )

        self.get_logger().info(
            "Comenzando tramo 1."
        )

    # =========================================================
    # CONTROL PRINCIPAL
    # =========================================================

    def control_loop(self):
        if (
            not self.initialized
            or self.finished
        ):
            return

        if self.state == "FORWARD":
            self.run_forward()

        elif self.state == "PAUSE_BEFORE_TURN":
            self.run_pause_before_turn()

        elif self.state == "TURN":
            self.run_turn()

        elif self.state == "TURN_SETTLE":
            self.run_turn_settle()

        elif self.state == "PAUSE_BEFORE_FORWARD":
            self.run_pause_before_forward()

        elif self.state == "FINISHED":
            self.stop_robot()

    # =========================================================
    # AVANCE RECTO
    # =========================================================

    def run_forward(self):
        dx = self.x - self.segment_start_x
        dy = self.y - self.segment_start_y

        # Distancia recorrida desde el inicio del tramo.
        # No se proyecta usando el yaw del EKF porque su orientación
        # global todavía no es suficientemente fiable.
        forward_distance = math.hypot(dx, dy)

        # Desviación perpendicular respecto del rumbo.
        lateral_error = (
            -dx * math.sin(self.segment_heading)
            + dy * math.cos(self.segment_heading)
        )

        # La IMU corregida mide positivo cuando el robot gira
        # físicamente en sentido antihorario.
        #
        # En el driver actual:
        #   comando positivo -> giro horario
        #   comando negativo -> giro antihorario
        #
        # Por eso, si forward_angle_rad es positivo, enviamos
        # una corrección positiva para devolverlo hacia la derecha.
        heading_error = self.forward_angle_rad

        if forward_distance >= (
            self.side_length
            - self.distance_tolerance
        ):
            self.stop_robot()

            self.get_logger().info(
                f"Tramo {self.side_number} completado: "
                f"avance={forward_distance:.3f} m, "
                f"error lateral estimado={lateral_error:.3f} m, "
                f"cambio IMU={math.degrees(self.forward_angle_rad):.2f}°"
            )

            self.state = "PAUSE_BEFORE_TURN"
            self.pause_start_time = time.monotonic()
            return

        if self.state_elapsed() > self.maximum_forward_time:
            self.emergency_stop(
                f"Tiempo máximo excedido en tramo "
                f"{self.side_number}."
            )
            return

        if abs(heading_error) > self.maximum_heading_error:
            self.emergency_stop(
                f"Desviación angular excesiva en tramo "
                f"{self.side_number}: "
                f"{math.degrees(heading_error):.1f}°."
            )
            return

        if abs(heading_error) < self.heading_deadband:
            angular_command = 0.0
        else:
            angular_command = (
                self.heading_kp
                * heading_error
            )

            angular_command = max(
                -self.max_angular_correction,
                min(
                    angular_command,
                    self.max_angular_correction,
                ),
            )

        command = Twist()
        command.linear.x = self.linear_speed
        command.angular.z = angular_command

        self.cmd_pub.publish(command)

    # =========================================================
    # PAUSA PREVIA AL GIRO
    # =========================================================

    def run_pause_before_turn(self):
        self.stop_robot()

        elapsed = (
            time.monotonic()
            - self.pause_start_time
        )

        if elapsed < self.pause_before_turn:
            return

        self.turn_angle_rad = 0.0
        self.last_imu_stamp = None
        self.integrate_imu = True

        self.state = "TURN"
        self.reset_state_timer()

        self.get_logger().info(
            f"Comenzando giro {self.side_number} "
            "medido directamente por IMU."
        )

    # =========================================================
    # GIRO ANTIHORARIO
    # =========================================================

    def run_turn(self):
        turned_deg = math.degrees(
            self.turn_angle_rad
        )

        if self.turn_angle_rad >= self.stop_turn_rad:
            self.stop_robot()

            self.get_logger().info(
                f"Ordenando parada del giro "
                f"{self.side_number} a "
                f"{turned_deg:.2f}° integrados."
            )

            self.state = "TURN_SETTLE"
            self.pause_start_time = time.monotonic()
            return

        if self.state_elapsed() > self.maximum_turn_time:
            self.integrate_imu = False

            self.emergency_stop(
                f"Tiempo máximo excedido en giro "
                f"{self.side_number}."
            )
            return

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = self.turn_command

        self.cmd_pub.publish(command)

    # =========================================================
    # ASENTAMIENTO DEL GIRO
    # =========================================================

    def run_turn_settle(self):
        self.stop_robot()

        elapsed = (
            time.monotonic()
            - self.pause_start_time
        )

        if elapsed < self.turn_settle_duration:
            return

        self.integrate_imu = False

        final_turn_deg = math.degrees(
            self.turn_angle_rad
        )

        turn_error = (
            final_turn_deg
            - self.target_turn_deg
        )

        self.get_logger().info(
            f"Giro {self.side_number} asentado: "
            f"{final_turn_deg:.2f}°, "
            f"error={turn_error:+.2f}°"
        )

        if self.side_number >= 4:
            self.finish_square()
            return

        self.side_number += 1

        self.state = "PAUSE_BEFORE_FORWARD"
        self.pause_start_time = time.monotonic()

    # =========================================================
    # NUEVO TRAMO
    # =========================================================

    def run_pause_before_forward(self):
        self.stop_robot()

        elapsed = (
            time.monotonic()
            - self.pause_start_time
        )

        if elapsed < self.pause_before_forward:
            return

        # La nueva orientación real se convierte en la referencia
        # del siguiente tramo recto.
        self.segment_heading = self.yaw

        self.segment_start_x = self.x
        self.segment_start_y = self.y

        self.forward_angle_rad = 0.0
        self.last_imu_stamp = None

        self.state = "FORWARD"
        self.reset_state_timer()

        self.get_logger().info(
            f"Comenzando tramo {self.side_number}: "
            f"rumbo de referencia="
            f"{math.degrees(self.segment_heading):.2f}°"
        )

    # =========================================================
    # FINALIZACIÓN
    # =========================================================

    def finish_square(self):
        self.stop_robot()

        self.integrate_imu = False
        self.finished = True
        self.state = "FINISHED"

        closure_error = math.hypot(
            self.x - self.initial_x,
            self.y - self.initial_y,
        )

        final_yaw_error = math.degrees(
            normalize_angle(
                self.yaw - self.initial_yaw
            )
        )

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "CUADRADO FÍSICO COMPLETADO"
        )

        self.get_logger().info(
            f"Pose final: "
            f"x={self.x:.3f}, "
            f"y={self.y:.3f}, "
            f"yaw={math.degrees(self.yaw):.2f}°"
        )

        self.get_logger().info(
            f"Error de cierre estimado: "
            f"{closure_error:.3f} m"
        )

        self.get_logger().info(
            f"Error angular estimado por EKF: "
            f"{final_yaw_error:+.2f}°"
        )

        self.get_logger().info(
            "========================================"
        )

    def emergency_stop(self, reason: str):
        self.stop_robot()

        self.integrate_imu = False
        self.finished = True
        self.state = "FINISHED"

        self.get_logger().error(
            f"PARADA DE SEGURIDAD: {reason}"
        )

    # =========================================================
    # UTILIDADES
    # =========================================================

    def stop_robot(self):
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    def reset_state_timer(self):
        self.state_start_time = time.monotonic()

    def state_elapsed(self) -> float:
        return (
            time.monotonic()
            - self.state_start_time
        )

    def destroy_node(self):
        try:
            for _ in range(15):
                self.stop_robot()
                time.sleep(0.05)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PhysicalSquareSimple()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
