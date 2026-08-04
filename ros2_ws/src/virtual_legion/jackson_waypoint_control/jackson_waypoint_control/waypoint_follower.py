#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al intervalo [-pi, pi]."""

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class JacksonSquareExplicit(Node):

    def __init__(self):
        super().__init__('jackson_square_explicit')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # Pose actual.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Pose inicial de todo el experimento.
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # Inicios de los cuatro tramos.
        self.start_1_x = 0.0
        self.start_1_y = 0.0

        self.start_2_x = 0.0
        self.start_2_y = 0.0

        self.start_3_x = 0.0
        self.start_3_y = 0.0

        self.start_4_x = 0.0
        self.start_4_y = 0.0

        # Inicios de los tres giros.
        self.turn_1_start_yaw = 0.0
        self.turn_2_start_yaw = 0.0
        self.turn_3_start_yaw = 0.0

        self.odom_received = False
        self.initialized = False

        # Máquina de estados explícita.
        self.state = 'FORWARD_1'

        # Distancias ya comprobadas.
        self.first_segment_distance = 1.00

        # Compensación de 2 cm para los siguientes lados.
        self.other_segment_distance = 1.02

        # Tolerancia lineal de 2 cm.
        self.distance_tolerance = 0.02

        # Giro antihorario.
        self.target_turn = math.radians(90.0)

        # Tolerancia angular validada.
        self.angle_tolerance = math.radians(2.0)

        # Velocidades lineales validadas.
        self.linear_speed_fast = 0.10
        self.linear_speed_slow = 0.05

        # Velocidades angulares validadas.
        self.angular_speed_fast = 0.45
        self.angular_speed_medium = 0.30
        self.angular_speed_minimum = 0.30

        self.get_logger().info(
            'NUEVO SCRIPT: cuadrado explícito de cuatro tramos.'
        )

        self.get_logger().info(
            'Esperando mensajes en /odom...'
        )

    def odom_callback(self, msg: Odometry):
        """Actualiza la pose desde /odom."""

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
            cosy_cosp
        )

        self.odom_received = True

        if not self.initialized:
            self.initial_x = self.x
            self.initial_y = self.y
            self.initial_yaw = self.yaw

            self.start_1_x = self.x
            self.start_1_y = self.y

            self.initialized = True

            self.get_logger().info(
                f'Pose inicial: '
                f'x={self.initial_x:.3f}, '
                f'y={self.initial_y:.3f}, '
                f'yaw={math.degrees(self.initial_yaw):.1f} grados'
            )

            self.get_logger().info(
                'Comenzando tramo 1.'
            )

    def control_loop(self):
        """Ejecuta el estado actual."""

        if not self.odom_received or not self.initialized:
            return

        if self.state == 'FORWARD_1':
            self.run_forward_1()

        elif self.state == 'TURN_1':
            self.run_turn_1()

        elif self.state == 'FORWARD_2':
            self.run_forward_2()

        elif self.state == 'TURN_2':
            self.run_turn_2()

        elif self.state == 'FORWARD_3':
            self.run_forward_3()

        elif self.state == 'TURN_3':
            self.run_turn_3()

        elif self.state == 'FORWARD_4':
            self.run_forward_4()

        elif self.state == 'FINISHED':
            self.stop_robot()

    # ==========================================================
    # TRAMO 1
    # ==========================================================

    def run_forward_1(self):
        traveled = math.hypot(
            self.x - self.start_1_x,
            self.y - self.start_1_y
        )

        remaining = self.first_segment_distance - traveled

        if traveled >= (
            self.first_segment_distance
            - self.distance_tolerance
        ):
            self.stop_robot()

            self.get_logger().info(
                f'Tramo 1 completado: {traveled:.3f} m'
            )

            self.turn_1_start_yaw = self.yaw
            self.state = 'TURN_1'

            self.get_logger().info(
                f'Comenzando giro 1 desde '
                f'{math.degrees(self.turn_1_start_yaw):.1f} grados'
            )
            return

        self.publish_forward(remaining)

    # ==========================================================
    # GIRO 1
    # ==========================================================

    def run_turn_1(self):
        if self.execute_turn(
            self.turn_1_start_yaw,
            turn_number=1
        ):
            self.start_2_x = self.x
            self.start_2_y = self.y
            self.state = 'FORWARD_2'

            self.get_logger().info(
                f'Comenzando tramo 2 desde '
                f'x={self.start_2_x:.3f}, '
                f'y={self.start_2_y:.3f}'
            )

    # ==========================================================
    # TRAMO 2
    # ==========================================================

    def run_forward_2(self):
        traveled = math.hypot(
            self.x - self.start_2_x,
            self.y - self.start_2_y
        )

        remaining = self.other_segment_distance - traveled

        if traveled >= (
            self.other_segment_distance
            - self.distance_tolerance
        ):
            self.stop_robot()

            self.get_logger().info(
                f'Tramo 2 completado: {traveled:.3f} m'
            )

            self.turn_2_start_yaw = self.yaw
            self.state = 'TURN_2'

            self.get_logger().info(
                f'Comenzando giro 2 desde '
                f'{math.degrees(self.turn_2_start_yaw):.1f} grados'
            )
            return

        self.publish_forward(remaining)

    # ==========================================================
    # GIRO 2
    # ==========================================================

    def run_turn_2(self):
        if self.execute_turn(
            self.turn_2_start_yaw,
            turn_number=2
        ):
            self.start_3_x = self.x
            self.start_3_y = self.y
            self.state = 'FORWARD_3'

            self.get_logger().info(
                f'Comenzando tramo 3 desde '
                f'x={self.start_3_x:.3f}, '
                f'y={self.start_3_y:.3f}'
            )

    # ==========================================================
    # TRAMO 3
    # ==========================================================

    def run_forward_3(self):
        traveled = math.hypot(
            self.x - self.start_3_x,
            self.y - self.start_3_y
        )

        remaining = self.other_segment_distance - traveled

        if traveled >= (
            self.other_segment_distance
            - self.distance_tolerance
        ):
            self.stop_robot()

            self.get_logger().info(
                f'Tramo 3 completado: {traveled:.3f} m'
            )

            self.turn_3_start_yaw = self.yaw
            self.state = 'TURN_3'

            self.get_logger().info(
                f'Comenzando giro 3 desde '
                f'{math.degrees(self.turn_3_start_yaw):.1f} grados'
            )
            return

        self.publish_forward(remaining)

    # ==========================================================
    # GIRO 3
    # ==========================================================

    def run_turn_3(self):
        if self.execute_turn(
            self.turn_3_start_yaw,
            turn_number=3
        ):
            self.start_4_x = self.x
            self.start_4_y = self.y
            self.state = 'FORWARD_4'

            self.get_logger().info(
                f'Comenzando tramo 4 desde '
                f'x={self.start_4_x:.3f}, '
                f'y={self.start_4_y:.3f}'
            )

    # ==========================================================
    # TRAMO 4
    # ==========================================================

    def run_forward_4(self):
        traveled = math.hypot(
            self.x - self.start_4_x,
            self.y - self.start_4_y
        )

        remaining = self.other_segment_distance - traveled

        if traveled >= (
            self.other_segment_distance
            - self.distance_tolerance
        ):
            self.stop_robot()

            self.get_logger().info(
                f'Tramo 4 completado: {traveled:.3f} m'
            )

            self.state = 'FINISHED'
            self.report_final_results()
            return

        self.publish_forward(remaining)

    # ==========================================================
    # FUNCIONES COMUNES
    # ==========================================================

    def publish_forward(self, remaining_distance: float):
        """Publica avance recto con dos velocidades."""

        cmd = Twist()

        if remaining_distance > 0.20:
            cmd.linear.x = self.linear_speed_fast
        else:
            cmd.linear.x = self.linear_speed_slow

        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def execute_turn(
        self,
        turn_start_yaw: float,
        turn_number: int
    ) -> bool:
        """
        Ejecuta un giro antihorario de 90 grados.

        Devuelve True cuando el giro terminó.
        """

        target_yaw = normalize_angle(
            turn_start_yaw + self.target_turn
        )

        angle_error = normalize_angle(
            target_yaw - self.yaw
        )

        error_degrees = abs(
            math.degrees(angle_error)
        )

        if abs(angle_error) <= self.angle_tolerance:
            self.stop_robot()

            turned_angle = normalize_angle(
                self.yaw - turn_start_yaw
            )

            self.get_logger().info(
                f'Giro {turn_number} completado: '
                f'{math.degrees(turned_angle):.1f} grados'
            )

            return True

        cmd = Twist()
        cmd.linear.x = 0.0

        if error_degrees > 20.0:
            angular_speed = self.angular_speed_fast

        elif error_degrees > 7.0:
            angular_speed = self.angular_speed_medium

        else:
            angular_speed = self.angular_speed_minimum

        cmd.angular.z = math.copysign(
            angular_speed,
            angle_error
        )

        self.cmd_pub.publish(cmd)

        return False

    def report_final_results(self):
        """Calcula el error de cierre al terminar el cuarto tramo."""

        closure_error = math.hypot(
            self.x - self.initial_x,
            self.y - self.initial_y
        )

        expected_final_yaw = normalize_angle(
            self.initial_yaw
            + math.radians(270.0)
        )

        angular_error = normalize_angle(
            self.yaw - expected_final_yaw
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'CUADRADO COMPLETADO'
        )

        self.get_logger().info(
            f'Pose inicial: '
            f'x={self.initial_x:.3f}, '
            f'y={self.initial_y:.3f}, '
            f'yaw={math.degrees(self.initial_yaw):.1f} grados'
        )

        self.get_logger().info(
            f'Pose final: '
            f'x={self.x:.3f}, '
            f'y={self.y:.3f}, '
            f'yaw={math.degrees(self.yaw):.1f} grados'
        )

        self.get_logger().info(
            f'Error de cierre lineal: '
            f'{closure_error:.3f} m'
        )

        self.get_logger().info(
            f'Error angular respecto a 270 grados: '
            f'{math.degrees(angular_error):.1f} grados'
        )

        self.get_logger().info(
            'Robot detenido.'
        )

        self.get_logger().info(
            '========================================'
        )

    def stop_robot(self):
        """Publica velocidades cero."""

        if rclpy.ok():
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = JacksonSquareExplicit()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
