#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class StraightDistance(Node):

    def __init__(self):
        super().__init__("straight_distance")

        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("maximum_time", 20.0)

        self.target_distance = float(
            self.get_parameter("target_distance").value
        )
        self.linear_speed = float(
            self.get_parameter("linear_speed").value
        )
        self.maximum_time = float(
            self.get_parameter("maximum_time").value
        )

        self.publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.subscription = self.create_subscription(
            Odometry,
            "/wheel/odom",
            self.odom_callback,
            10,
        )

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        self.start_time = None
        self.stopping = False
        self.stop_cycles = 0

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.get_logger().info(
            f"Prueba recta: objetivo={self.target_distance:.3f} m, "
            f"velocidad={self.linear_speed:.3f} m/s"
        )

        self.get_logger().info(
            "Esperando /wheel/odom..."
        )

    def odom_callback(self, message: Odometry):
        self.current_x = message.pose.pose.position.x
        self.current_y = message.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_time = time.monotonic()

            self.get_logger().info(
                "Odometría recibida. Iniciando movimiento."
            )

    def travelled_distance(self) -> float:
        if self.start_x is None:
            return 0.0

        return math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y,
        )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def control_loop(self):
        if self.start_x is None:
            return

        if self.stopping:
            self.publish_stop()
            self.stop_cycles += 1

            if self.stop_cycles >= 10:
                rclpy.shutdown()

            return

        distance = self.travelled_distance()
        elapsed = time.monotonic() - self.start_time

        if distance >= self.target_distance:
            self.get_logger().info(
                f"Objetivo alcanzado: {distance:.4f} m"
            )
            self.stopping = True
            self.publish_stop()
            return

        if elapsed >= self.maximum_time:
            self.get_logger().error(
                f"Timeout de seguridad. Distancia={distance:.4f} m"
            )
            self.stopping = True
            self.publish_stop()
            return

        command = Twist()
        command.linear.x = self.linear_speed
        command.angular.z = 0.0

        self.publisher.publish(command)

    def destroy_node(self):
        try:
            for _ in range(5):
                self.publish_stop()
                time.sleep(0.05)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StraightDistance()

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
