#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (
        q.w * q.z + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y + q.z * q.z
    )

    return math.atan2(siny_cosp, cosy_cosp)


class Square1m(Node):

    def __init__(self):
        super().__init__("square_1m")

        # Parámetros del experimento
        self.declare_parameter("side_length", 1.0)
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.20)

        self.declare_parameter("linear_stop_margin", 0.04)
        self.declare_parameter("angular_stop_margin", 0.14)

        self.declare_parameter("heading_kp", 1.2)
        self.declare_parameter(
            "max_heading_correction",
            0.12,
        )

        self.declare_parameter("settle_time", 1.2)
        self.declare_parameter("start_delay", 2.0)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("maximum_test_time", 120.0)

        self.side_length = float(
            self.get_parameter("side_length").value
        )
        self.linear_speed = float(
            self.get_parameter("linear_speed").value
        )
        self.angular_speed = float(
            self.get_parameter("angular_speed").value
        )

        self.linear_stop_margin = float(
            self.get_parameter(
                "linear_stop_margin"
            ).value
        )
        self.angular_stop_margin = float(
            self.get_parameter(
                "angular_stop_margin"
            ).value
        )

        self.heading_kp = float(
            self.get_parameter("heading_kp").value
        )
        self.max_heading_correction = float(
            self.get_parameter(
                "max_heading_correction"
            ).value
        )

        self.settle_time = float(
            self.get_parameter("settle_time").value
        )
        self.start_delay = float(
            self.get_parameter("start_delay").value
        )
        self.odom_timeout = float(
            self.get_parameter("odom_timeout").value
        )
        self.maximum_test_time = float(
            self.get_parameter(
                "maximum_test_time"
            ).value
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            20,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        # Pose actual
        self.x = None
        self.y = None
        self.yaw_wrapped = None
        self.yaw_unwrapped = None
        self.previous_yaw_wrapped = None

        self.last_odom_time = None
        self.first_odom_time = None

        # Estado del experimento
        self.state = "WAITING"
        self.state_start_time = time.monotonic()
        self.experiment_start_time = None

        self.side_index = 0

        self.initial_x = None
        self.initial_y = None
        self.initial_heading = None

        self.segment_start_x = None
        self.segment_start_y = None
        self.target_heading = None

        self.timeout_reported = False
        self.finished = False

        self.get_logger().info(
            "Controlador cuadrado 1 x 1 m iniciado"
        )
        self.get_logger().info(
            "Esperando /odometry/filtered..."
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        current_yaw = quaternion_to_yaw(
            msg.pose.pose.orientation
        )

        if self.previous_yaw_wrapped is None:
            self.yaw_wrapped = current_yaw
            self.previous_yaw_wrapped = current_yaw
            self.yaw_unwrapped = current_yaw
        else:
            delta_yaw = normalize_angle(
                current_yaw -
                self.previous_yaw_wrapped
            )

            self.yaw_unwrapped += delta_yaw
            self.yaw_wrapped = current_yaw
            self.previous_yaw_wrapped = current_yaw

        now = time.monotonic()
        self.last_odom_time = now

        if self.first_odom_time is None:
            self.first_odom_time = now

    def publish_command(
        self,
        linear_x=0.0,
        angular_z=0.0,
    ):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_publisher.publish(msg)

    def stop_robot(self):
        self.publish_command(0.0, 0.0)

    def begin_experiment(self, now):
        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_heading = self.yaw_unwrapped

        self.segment_start_x = self.x
        self.segment_start_y = self.y
        self.target_heading = self.initial_heading

        self.experiment_start_time = now
        self.state_start_time = now
        self.state = "STRAIGHT"

        self.get_logger().info(
            "Inicio del cuadrado: lado 1 de 4"
        )

    def control_straight(self, now):
        delta_x = self.x - self.segment_start_x
        delta_y = self.y - self.segment_start_y

        progress = (
            delta_x * math.cos(self.target_heading)
            + delta_y * math.sin(self.target_heading)
        )

        stop_distance = (
            self.side_length -
            self.linear_stop_margin
        )

        if progress >= stop_distance:
            self.stop_robot()

            self.state = "PAUSE_AFTER_STRAIGHT"
            self.state_start_time = now

            self.get_logger().info(
                f"Lado {self.side_index + 1}: "
                f"orden de parada en {progress:.3f} m"
            )
            return

        heading_error = normalize_angle(
            self.target_heading -
            self.yaw_unwrapped
        )

        angular_correction = clamp(
            self.heading_kp * heading_error,
            -self.max_heading_correction,
            self.max_heading_correction,
        )

        self.publish_command(
            self.linear_speed,
            angular_correction,
        )

    def begin_turn(self, now):
        self.target_heading = (
            self.initial_heading
            + (self.side_index + 1)
            * math.pi / 2.0
        )

        self.state = "TURN"
        self.state_start_time = now

        remaining = (
            self.target_heading -
            self.yaw_unwrapped
        )

        self.get_logger().info(
            f"Giro {self.side_index + 1}: "
            f"objetivo restante="
            f"{math.degrees(remaining):.1f} grados"
        )

    def control_turn(self, now):
        remaining_angle = (
            self.target_heading -
            self.yaw_unwrapped
        )

        if remaining_angle <= self.angular_stop_margin:
            self.stop_robot()

            self.state = "PAUSE_AFTER_TURN"
            self.state_start_time = now

            self.get_logger().info(
                f"Giro {self.side_index + 1}: "
                f"orden de parada; restante EKF="
                f"{math.degrees(remaining_angle):.1f} grados"
            )
            return

        self.publish_command(
            0.0,
            self.angular_speed,
        )

    def finish_turn(self, now):
        achieved_heading = (
            self.yaw_unwrapped -
            self.initial_heading
        )

        self.get_logger().info(
            f"Giro {self.side_index + 1} terminado: "
            f"yaw acumulado="
            f"{math.degrees(achieved_heading):.1f} grados"
        )

        self.side_index += 1

        if self.side_index >= 4:
            self.finish_experiment()
            return

        self.segment_start_x = self.x
        self.segment_start_y = self.y

        self.target_heading = (
            self.initial_heading
            + self.side_index * math.pi / 2.0
        )

        self.state = "STRAIGHT"
        self.state_start_time = now

        self.get_logger().info(
            f"Inicio del lado "
            f"{self.side_index + 1} de 4"
        )

    def finish_experiment(self):
        self.stop_robot()

        final_position_error = math.hypot(
            self.x - self.initial_x,
            self.y - self.initial_y,
        )

        final_yaw_error = normalize_angle(
            self.yaw_unwrapped -
            self.initial_heading
        )

        self.finished = True
        self.state = "FINISHED"

        self.get_logger().info(
            "CUADRADO TERMINADO"
        )
        self.get_logger().info(
            f"Error de cierre EKF: "
            f"{final_position_error:.3f} m"
        )
        self.get_logger().info(
            f"Error final de yaw EKF: "
            f"{math.degrees(final_yaw_error):.2f} grados"
        )
        self.get_logger().info(
            "El nodo permanecerá detenido. "
            "Use Ctrl+C para finalizar."
        )

    def control_loop(self):
        now = time.monotonic()

        if self.finished:
            self.stop_robot()
            return

        if self.x is None or self.last_odom_time is None:
            self.stop_robot()
            return

        if (
            now - self.last_odom_time
            > self.odom_timeout
        ):
            self.stop_robot()

            if not self.timeout_reported:
                self.get_logger().error(
                    "Timeout de odometría. "
                    "Robot detenido."
                )
                self.timeout_reported = True
            return

        self.timeout_reported = False

        if (
            self.experiment_start_time is not None
            and now - self.experiment_start_time
            > self.maximum_test_time
        ):
            self.stop_robot()
            self.finished = True
            self.state = "FINISHED"

            self.get_logger().error(
                "Tiempo máximo excedido. "
                "Robot detenido."
            )
            return

        if self.state == "WAITING":
            self.stop_robot()

            if (
                now - self.first_odom_time
                >= self.start_delay
            ):
                self.begin_experiment(now)

        elif self.state == "STRAIGHT":
            self.control_straight(now)

        elif self.state == "PAUSE_AFTER_STRAIGHT":
            self.stop_robot()

            if (
                now - self.state_start_time
                >= self.settle_time
            ):
                self.begin_turn(now)

        elif self.state == "TURN":
            self.control_turn(now)

        elif self.state == "PAUSE_AFTER_TURN":
            self.stop_robot()

            if (
                now - self.state_start_time
                >= self.settle_time
            ):
                self.finish_turn(now)

    def destroy_node(self):
        try:
            for _ in range(5):
                self.stop_robot()
                time.sleep(0.05)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Square1m()

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
