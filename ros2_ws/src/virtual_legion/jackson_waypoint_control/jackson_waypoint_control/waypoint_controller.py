#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al intervalo [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Limita un valor a un intervalo."""
    return max(minimum, min(maximum, value))


class JacksonWaypointController(Node):

    def __init__(self):
        super().__init__('jackson_waypoint_controller')

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

        # Control a 20 Hz.
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # Pose actual.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Pose inicial.
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.odom_received = False
        self.initialized = False
        self.finished = False

        # Estados:
        # ALIGN -> DRIVE -> ... -> FINAL_ALIGN -> FINISHED
        self.state = 'ALIGN'

        # Cuadrado de 1 x 1 m, relativo a la pose inicial.
        self.local_waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]

        self.odom_waypoints = []
        self.current_waypoint = 0

        # Tolerancias.
        self.position_tolerance = 0.03
        self.align_tolerance = math.radians(3.0)
        self.final_angle_tolerance = math.radians(2.0)
        self.drive_angle_limit = math.radians(15.0)

        # Velocidades lineales.
        self.max_linear_speed = 0.10
        self.min_linear_speed = 0.04

        # Velocidades angulares.
        self.max_angular_speed = 0.45

        # En las pruebas vimos que valores menores pueden
        # quedar dentro de la zona muerta del robot.
        self.min_turn_speed = 0.30

        # Ganancias.
        self.kp_angular_align = 1.5
        self.kp_angular_drive = 1.2
        self.kp_linear = 0.35

        # Medición del recorrido total.
        self.total_distance = 0.0
        self.previous_x = None
        self.previous_y = None

        self.get_logger().info(
            'Controlador por waypoints con alineación final iniciado.'
        )

        self.get_logger().info(
            'Esperando mensajes en /odom...'
        )

    def odom_callback(self, msg: Odometry):
        """Actualiza pose y distancia recorrida."""

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

        if self.previous_x is not None:
            self.total_distance += math.hypot(
                self.x - self.previous_x,
                self.y - self.previous_y
            )

        self.previous_x = self.x
        self.previous_y = self.y

        self.odom_received = True

        if not self.initialized:
            self.initialize_waypoints()

    def initialize_waypoints(self):
        """Transforma los waypoints locales al marco odom."""

        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_yaw = self.yaw

        cos_yaw = math.cos(self.initial_yaw)
        sin_yaw = math.sin(self.initial_yaw)

        for local_x, local_y in self.local_waypoints:

            odom_x = (
                self.initial_x
                + local_x * cos_yaw
                - local_y * sin_yaw
            )

            odom_y = (
                self.initial_y
                + local_x * sin_yaw
                + local_y * cos_yaw
            )

            self.odom_waypoints.append(
                (odom_x, odom_y)
            )

        self.initialized = True

        self.get_logger().info(
            f'Pose inicial: '
            f'x={self.initial_x:.3f}, '
            f'y={self.initial_y:.3f}, '
            f'yaw={math.degrees(self.initial_yaw):.1f}°'
        )

        for index, waypoint in enumerate(
            self.odom_waypoints,
            start=1
        ):
            self.get_logger().info(
                f'Waypoint {index}: '
                f'x={waypoint[0]:.3f}, '
                f'y={waypoint[1]:.3f}'
            )

        self.report_current_waypoint()

    def control_loop(self):
        """Ejecuta el estado actual."""

        if not self.odom_received or not self.initialized:
            return

        if self.finished:
            self.stop_robot()
            return

        if self.state == 'FINAL_ALIGN':
            self.final_align()
            return

        target_x, target_y = self.odom_waypoints[
            self.current_waypoint
        ]

        dx = target_x - self.x
        dy = target_y - self.y

        distance_error = math.hypot(
            dx,
            dy
        )

        desired_yaw = math.atan2(
            dy,
            dx
        )

        angle_error = normalize_angle(
            desired_yaw - self.yaw
        )

        if distance_error <= self.position_tolerance:
            self.reach_waypoint(distance_error)
            return

        if self.state == 'ALIGN':
            self.align_to_waypoint(angle_error)

        elif self.state == 'DRIVE':
            self.drive_to_waypoint(
                distance_error,
                angle_error
            )

    def align_to_waypoint(self, angle_error: float):
        """Gira hasta orientarse hacia el waypoint."""

        if abs(angle_error) <= self.align_tolerance:
            self.stop_robot()
            self.state = 'DRIVE'

            self.get_logger().info(
                f'Alineación completada para waypoint '
                f'{self.current_waypoint + 1}.'
            )
            return

        cmd = Twist()

        angular_command = (
            self.kp_angular_align
            * angle_error
        )

        angular_magnitude = clamp(
            abs(angular_command),
            self.min_turn_speed,
            self.max_angular_speed
        )

        cmd.linear.x = 0.0
        cmd.angular.z = math.copysign(
            angular_magnitude,
            angle_error
        )

        self.cmd_pub.publish(cmd)

    def drive_to_waypoint(
        self,
        distance_error: float,
        angle_error: float
    ):
        """Avanza corrigiendo continuamente el rumbo."""

        if abs(angle_error) > self.drive_angle_limit:
            self.stop_robot()
            self.state = 'ALIGN'
            return

        cmd = Twist()

        linear_command = (
            self.kp_linear
            * distance_error
        )

        cmd.linear.x = clamp(
            linear_command,
            self.min_linear_speed,
            self.max_linear_speed
        )

        if distance_error < 0.20:
            cmd.linear.x = self.min_linear_speed

        angular_command = (
            self.kp_angular_drive
            * angle_error
        )

        cmd.angular.z = clamp(
            angular_command,
            -0.25,
            0.25
        )

        self.cmd_pub.publish(cmd)

    def reach_waypoint(self, final_error: float):
        """Gestiona la llegada al waypoint."""

        self.stop_robot()

        self.get_logger().info(
            f'Waypoint {self.current_waypoint + 1} alcanzado. '
            f'Error restante={final_error:.3f} m; '
            f'pose=({self.x:.3f}, {self.y:.3f}); '
            f'yaw={math.degrees(self.yaw):.1f}°'
        )

        self.current_waypoint += 1

        # Después del último waypoint se recupera
        # la orientación inicial.
        if self.current_waypoint >= len(
            self.odom_waypoints
        ):
            self.state = 'FINAL_ALIGN'

            self.get_logger().info(
                'Posición cerrada. Comenzando alineación final.'
            )

            self.get_logger().info(
                f'Yaw actual: {math.degrees(self.yaw):.1f}°'
            )

            self.get_logger().info(
                f'Yaw objetivo: '
                f'{math.degrees(self.initial_yaw):.1f}°'
            )

            return

        self.state = 'ALIGN'
        self.report_current_waypoint()

    def final_align(self):
        """Recupera la orientación inicial."""

        angle_error = normalize_angle(
            self.initial_yaw - self.yaw
        )

        if abs(angle_error) <= self.final_angle_tolerance:
            self.stop_robot()

            self.get_logger().info(
                'Alineación final completada.'
            )

            self.finish_trajectory()
            return

        cmd = Twist()

        angular_command = (
            self.kp_angular_align
            * angle_error
        )

        angular_magnitude = clamp(
            abs(angular_command),
            self.min_turn_speed,
            self.max_angular_speed
        )

        cmd.linear.x = 0.0
        cmd.angular.z = math.copysign(
            angular_magnitude,
            angle_error
        )

        self.cmd_pub.publish(cmd)

    def report_current_waypoint(self):
        """Informa el waypoint activo."""

        target_x, target_y = self.odom_waypoints[
            self.current_waypoint
        ]

        self.get_logger().info(
            f'Comenzando waypoint '
            f'{self.current_waypoint + 1} de '
            f'{len(self.odom_waypoints)}: '
            f'x={target_x:.3f}, '
            f'y={target_y:.3f}'
        )

    def finish_trajectory(self):
        """Calcula los errores finales."""

        self.stop_robot()
        self.finished = True
        self.state = 'FINISHED'

        closure_error = math.hypot(
            self.x - self.initial_x,
            self.y - self.initial_y
        )

        angular_error = normalize_angle(
            self.yaw - self.initial_yaw
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'TRAYECTORIA CERRADA COMPLETADA'
        )

        self.get_logger().info(
            f'Pose inicial: '
            f'x={self.initial_x:.3f}, '
            f'y={self.initial_y:.3f}, '
            f'yaw={math.degrees(self.initial_yaw):.1f}°'
        )

        self.get_logger().info(
            f'Pose final: '
            f'x={self.x:.3f}, '
            f'y={self.y:.3f}, '
            f'yaw={math.degrees(self.yaw):.1f}°'
        )

        self.get_logger().info(
            f'Error de cierre lineal: '
            f'{closure_error:.3f} m'
        )

        self.get_logger().info(
            f'Error de cierre angular: '
            f'{math.degrees(angular_error):.1f}°'
        )

        self.get_logger().info(
            f'Distancia total medida por /odom: '
            f'{self.total_distance:.3f} m'
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

    node = JacksonWaypointController()

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
