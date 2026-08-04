#!/usr/bin/env python3

import time
from typing import Dict, List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PhysicalSquareController(Node):
    """
    Control temporal para que el Jackson físico recorra
    un cuadrado aproximado de 20 x 20 cm.

    Publica exclusivamente en:

        /cmd_vel_physical

    Calibración experimental:

        linear.x = 0.05 durante 2.97 s ≈ 20 cm
        angular.z = 0.30 durante 1.85 s ≈ 90°
    """

    def __init__(self):
        super().__init__("physical_square_controller")

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel_physical",
            10,
        )

        # Publicación a 20 Hz.
        self.control_period = 0.05

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        # ====================================================
        # PARÁMETROS CALIBRADOS
        # ====================================================

        self.forward_duration = 3.27
        self.turn_duration = 2.12

        self.linear_command = 0.05
        self.angular_command = 0.30

        # Pausas para que el robot se detenga antes
        # de cambiar entre avance y giro.
        self.pause_duration = 1.00
        self.final_pause_duration = 1.50

        # ====================================================
        # SECUENCIA DEL CUADRADO
        # ====================================================

        self.sequence: List[Dict[str, float | str]] = []

        for side_number in range(1, 5):

            self.sequence.append(
                {
                    "name": f"AVANCE_{side_number}",
                    "duration": self.forward_duration,
                    "linear_x": self.linear_command,
                    "angular_z": 0.0,
                }
            )

            self.sequence.append(
                {
                    "name": f"PAUSA_DESPUES_AVANCE_{side_number}",
                    "duration": self.pause_duration,
                    "linear_x": 0.0,
                    "angular_z": 0.0,
                }
            )

            self.sequence.append(
                {
                    "name": f"GIRO_{side_number}",
                    "duration": self.turn_duration,
                    "linear_x": 0.0,
                    "angular_z": self.angular_command,
                }
            )

            pause_after_turn = (
                self.final_pause_duration
                if side_number == 4
                else self.pause_duration
            )

            self.sequence.append(
                {
                    "name": f"PAUSA_DESPUES_GIRO_{side_number}",
                    "duration": pause_after_turn,
                    "linear_x": 0.0,
                    "angular_z": 0.0,
                }
            )

        self.current_step = 0
        self.step_start_time = time.monotonic()
        self.finished = False

        self.get_logger().info(
            "Controlador del cuadrado físico iniciado."
        )

        self.get_logger().info(
            "Publicando en /cmd_vel_physical"
        )

        self.get_logger().info(
            f"Tiempo de avance: "
            f"{self.forward_duration:.2f} s"
        )

        self.get_logger().info(
            f"Tiempo de giro: "
            f"{self.turn_duration:.2f} s"
        )

        self.get_logger().info(
            "Trayectoria esperada: cuadrado de 20 x 20 cm."
        )

        self.report_current_step()

    def control_loop(self):
        """Ejecuta la máquina de estados temporal."""

        if self.finished:
            self.publish_stop()
            return

        step = self.sequence[self.current_step]

        elapsed = (
            time.monotonic()
            - self.step_start_time
        )

        duration = float(step["duration"])

        if elapsed >= duration:
            self.publish_stop()

            self.get_logger().info(
                f"Paso completado: {step['name']} "
                f"(tiempo real={elapsed:.2f} s)"
            )

            self.current_step += 1

            if self.current_step >= len(self.sequence):
                self.finish_trajectory()
                return

            self.step_start_time = time.monotonic()
            self.report_current_step()
            return

        command = Twist()

        command.linear.x = float(
            step["linear_x"]
        )

        command.angular.z = float(
            step["angular_z"]
        )

        self.publisher.publish(command)

    def report_current_step(self):
        """Informa el paso que comenzará."""

        step = self.sequence[self.current_step]

        self.get_logger().info(
            f"Comenzando {step['name']}: "
            f"duración={float(step['duration']):.2f} s, "
            f"linear.x={float(step['linear_x']):.2f}, "
            f"angular.z={float(step['angular_z']):.2f}"
        )

    def finish_trajectory(self):
        """Finaliza el cuadrado y mantiene el robot detenido."""

        self.finished = True

        for _ in range(5):
            self.publish_stop()

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "CUADRADO FÍSICO COMPLETADO"
        )

        self.get_logger().info(
            "Robot detenido."
        )

        self.get_logger().info(
            "Mida el error entre la pose inicial y final."
        )

        self.get_logger().info(
            "========================================"
        )

    def publish_stop(self):
        """Publica una orden completa de parada."""

        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = PhysicalSquareController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            for _ in range(10):
                node.publish_stop()
                time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
