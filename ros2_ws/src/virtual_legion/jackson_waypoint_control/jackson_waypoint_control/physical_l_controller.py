#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class PhysicalLController(Node):
    """
    Ejecuta una trayectoria temporal en L con el Jackson físico:

        avance aproximado de 20 cm
        pausa
        giro antihorario aproximado de 90 grados
        pausa
        avance aproximado de 20 cm
        parada final

    Publica únicamente en /cmd_vel_physical.
    """

    def __init__(self):
        super().__init__('physical_l_controller')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_physical',
            10,
        )

        # Lazo de publicación a 20 Hz.
        self.timer_period = 0.05

        self.timer = self.create_timer(
            self.timer_period,
            self.control_loop,
        )

        # ====================================================
        # CALIBRACIÓN ACTUALIZADA
        # ====================================================
        #
        # Prueba anterior:
        #   4.00 s  -> aproximadamente 30 cm
        #   2.65 s  -> aproximadamente 145 grados
        #
        # Nuevos tiempos calculados:
        #   avance de 20 cm:
        #       4.00 × 20/30 = 2.67 s
        #
        #   giro de 90 grados:
        #       2.65 × 90/145 = 1.64 s
        #
        FORWARD_DURATION = 2.97
        TURN_DURATION = 1.85

        LINEAR_COMMAND = 0.05
        ANGULAR_COMMAND = 0.30

        PAUSE_DURATION = 1.00

        self.sequence = [
            {
                'name': 'AVANCE_1',
                'duration': FORWARD_DURATION,
                'linear_x': LINEAR_COMMAND,
                'angular_z': 0.0,
            },
            {
                'name': 'PAUSA_1',
                'duration': PAUSE_DURATION,
                'linear_x': 0.0,
                'angular_z': 0.0,
            },
            {
                'name': 'GIRO_90',
                'duration': TURN_DURATION,
                'linear_x': 0.0,
                'angular_z': ANGULAR_COMMAND,
            },
            {
                'name': 'PAUSA_2',
                'duration': PAUSE_DURATION,
                'linear_x': 0.0,
                'angular_z': 0.0,
            },
            {
                'name': 'AVANCE_2',
                'duration': FORWARD_DURATION,
                'linear_x': LINEAR_COMMAND,
                'angular_z': 0.0,
            },
            {
                'name': 'PAUSA_FINAL',
                'duration': 1.50,
                'linear_x': 0.0,
                'angular_z': 0.0,
            },
        ]

        self.current_step = 0
        self.step_start_time = time.monotonic()
        self.finished = False

        self.get_logger().info(
            'Controlador físico en L iniciado.'
        )

        self.get_logger().info(
            'Publicando en /cmd_vel_physical'
        )

        self.get_logger().info(
            f'Tiempo de avance: {FORWARD_DURATION:.2f} s'
        )

        self.get_logger().info(
            f'Tiempo de giro: {TURN_DURATION:.2f} s'
        )

        self.report_step()

    def control_loop(self):
        """Ejecuta el paso temporal activo."""

        if self.finished:
            self.publish_stop()
            return

        step = self.sequence[self.current_step]

        elapsed = (
            time.monotonic()
            - self.step_start_time
        )

        if elapsed >= step['duration']:
            # Enviar parada antes de cambiar de estado.
            self.publish_stop()

            self.get_logger().info(
                f"Paso completado: {step['name']} "
                f"(tiempo real={elapsed:.2f} s)"
            )

            self.current_step += 1

            if self.current_step >= len(self.sequence):
                self.finished = True

                self.publish_stop()

                self.get_logger().info(
                    'Trayectoria física en L completada.'
                )

                self.get_logger().info(
                    'Robot detenido.'
                )

                return

            self.step_start_time = time.monotonic()
            self.report_step()
            return

        command = Twist()

        command.linear.x = float(
            step['linear_x']
        )

        command.angular.z = float(
            step['angular_z']
        )

        self.publisher.publish(command)

    def report_step(self):
        """Informa el comienzo del paso activo."""

        step = self.sequence[self.current_step]

        self.get_logger().info(
            f"Comenzando {step['name']}: "
            f"duración={step['duration']:.2f} s, "
            f"linear.x={step['linear_x']:.2f}, "
            f"angular.z={step['angular_z']:.2f}"
        )

    def publish_stop(self):
        """Publica una orden completa de parada."""

        self.publisher.publish(
            Twist()
        )


def main(args=None):
    rclpy.init(args=args)

    node = PhysicalLController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            # Publicar varias veces para asegurar que el
            # driver físico recibe la parada.
            for _ in range(5):
                node.publish_stop()
                time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
