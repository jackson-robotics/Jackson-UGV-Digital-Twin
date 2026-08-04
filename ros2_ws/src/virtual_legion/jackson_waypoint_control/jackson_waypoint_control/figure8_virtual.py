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
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def approach(current, target, maximum_change):
    if target > current:
        return min(target, current + maximum_change)
    return max(target, current - maximum_change)


class VirtualFigure8(Node):

    def __init__(self):
        super().__init__("virtual_figure8_r05")

        # Geometry and pilot speed.
        self.declare_parameter("radius", 0.5)
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("nominal_angular_speed", 0.16)

        # Circle tracking from EKF pose.
        self.declare_parameter("heading_kp", 1.2)
        self.declare_parameter("radial_heading_gain", 1.4)
        self.declare_parameter("max_radial_heading", 0.30)
        self.declare_parameter("max_angular_correction", 0.12)
        self.declare_parameter("maximum_angular_speed", 0.40)

        # Smooth command transitions.
        self.declare_parameter("linear_acceleration", 0.16)
        self.declare_parameter("angular_acceleration", 0.30)

        # Experiment timing and completion.
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("maximum_test_time", 120.0)
        self.declare_parameter("first_loop_switch_margin", 0.04)
        self.declare_parameter("final_yaw_margin", 0.03)

        self.radius = float(self.get_parameter("radius").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.heading_kp = float(self.get_parameter("heading_kp").value)
        self.radial_heading_gain = float(
            self.get_parameter("radial_heading_gain").value
        )
        self.max_radial_heading = float(
            self.get_parameter("max_radial_heading").value
        )
        self.max_angular_correction = float(
            self.get_parameter("max_angular_correction").value
        )
        self.maximum_angular_speed = float(
            self.get_parameter("maximum_angular_speed").value
        )
        self.linear_acceleration = float(
            self.get_parameter("linear_acceleration").value
        )
        self.angular_acceleration = float(
            self.get_parameter("angular_acceleration").value
        )
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout").value)
        self.maximum_test_time = float(
            self.get_parameter("maximum_test_time").value
        )
        self.first_loop_switch_margin = float(
            self.get_parameter("first_loop_switch_margin").value
        )
        self.final_yaw_margin = float(
            self.get_parameter("final_yaw_margin").value
        )

        if self.radius <= 0.0:
            raise ValueError("radius must be positive")
        if self.linear_speed <= 0.0:
            raise ValueError("linear_speed must be positive")

        self.nominal_angular_speed = float(self.get_parameter("nominal_angular_speed").value)

        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            20,
        )
        self.timer = self.create_timer(0.05, self.control_loop)

        self.x = None
        self.y = None
        self.yaw_wrapped = None
        self.yaw_unwrapped = None
        self.previous_yaw_wrapped = None
        self.first_odom_time = None
        self.last_odom_time = None

        self.state = "WAITING"
        self.experiment_start_time = None
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None
        self.circle_ccw_center = None
        self.circle_cw_center = None

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_control_time = self.get_clock().now().nanoseconds * 1.0e-9
        self.timeout_reported = False
        self.finished = False

        self.get_logger().info(
            "Virtual Figure-8 controller initialized: "
            f"R={self.radius:.3f} m, v={self.linear_speed:.3f} m/s, "
            f"nominal |w|={self.nominal_angular_speed:.3f} rad/s"
        )
        self.get_logger().info("Waiting for /odometry/filtered...")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.previous_yaw_wrapped is None:
            self.yaw_wrapped = yaw
            self.previous_yaw_wrapped = yaw
            self.yaw_unwrapped = yaw
        else:
            delta = normalize_angle(yaw - self.previous_yaw_wrapped)
            self.yaw_unwrapped += delta
            self.yaw_wrapped = yaw
            self.previous_yaw_wrapped = yaw

        now = self.get_clock().now().nanoseconds * 1.0e-9
        self.last_odom_time = now
        if self.first_odom_time is None:
            self.first_odom_time = now

    def publish_command(self, linear_x, angular_z):
        message = Twist()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        self.cmd_publisher.publish(message)

    def stop_robot(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_command(0.0, 0.0)

    def begin_experiment(self, now):
        self.initial_x = self.x
        self.initial_y = self.y
        self.initial_yaw = self.yaw_unwrapped

        left_x = -math.sin(self.initial_yaw)
        left_y = math.cos(self.initial_yaw)

        self.circle_ccw_center = (
            self.initial_x + self.radius * left_x,
            self.initial_y + self.radius * left_y,
        )
        self.circle_cw_center = (
            self.initial_x - self.radius * left_x,
            self.initial_y - self.radius * left_y,
        )

        self.experiment_start_time = now
        self.state = "LOOP_CCW"
        self.timeout_reported = False

        self.get_logger().info(
            "FIGURE8 START: first loop CCW; "
            f"initial pose=({self.initial_x:.3f}, {self.initial_y:.3f}), "
            f"yaw={math.degrees(self.initial_yaw):.2f} deg"
        )

    def circle_target(self, center, direction):
        dx = self.x - center[0]
        dy = self.y - center[1]
        distance = math.hypot(dx, dy)
        polar_angle = math.atan2(dy, dx)
        radial_error = distance - self.radius

        radial_heading = clamp(
            self.radial_heading_gain * radial_error,
            -self.max_radial_heading,
            self.max_radial_heading,
        )
        desired_heading = (
            polar_angle
            + direction * math.pi / 2.0
            + direction * radial_heading
        )
        heading_error = normalize_angle(desired_heading - self.yaw_wrapped)
        angular_correction = clamp(
            self.heading_kp * heading_error,
            -self.max_angular_correction,
            self.max_angular_correction,
        )
        desired_angular = clamp(
            direction * self.nominal_angular_speed + angular_correction,
            -self.maximum_angular_speed,
            self.maximum_angular_speed,
        )

        return self.linear_speed, desired_angular

    def smooth_publish(self, desired_linear, desired_angular, dt):
        self.current_linear = approach(
            self.current_linear,
            desired_linear,
            self.linear_acceleration * dt,
        )
        self.current_angular = approach(
            self.current_angular,
            desired_angular,
            self.angular_acceleration * dt,
        )
        self.publish_command(self.current_linear, self.current_angular)

    def finish_experiment(self, now, reason="completed"):
        if self.finished:
            return

        self.stop_robot()
        self.finished = True
        self.state = "FINISHED"

        elapsed = (
            now - self.experiment_start_time
            if self.experiment_start_time is not None
            else 0.0
        )
        dx = self.x - self.initial_x
        dy = self.y - self.initial_y
        closure = math.hypot(dx, dy)
        net_yaw = self.yaw_unwrapped - self.initial_yaw

        self.get_logger().info(f"FIGURE8 FINISHED: reason={reason}")
        self.get_logger().info(
            f"Final EKF pose delta: dx={dx:.4f} m, dy={dy:.4f} m, "
            f"closure={closure:.4f} m"
        )
        self.get_logger().info(
            f"Final net EKF yaw: {math.degrees(net_yaw):.2f} deg"
        )
        self.get_logger().info(f"Motion duration: {elapsed:.3f} s")

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1.0e-9
        dt = clamp(now - self.last_control_time, 0.0, 0.20)
        self.last_control_time = now

        if self.finished:
            self.publish_command(0.0, 0.0)
            return

        if self.last_odom_time is None:
            self.publish_command(0.0, 0.0)
            return

        if now - self.last_odom_time > self.odom_timeout:
            self.stop_robot()
            if not self.timeout_reported:
                self.get_logger().error(
                    "Odometry timeout: robot stopped for safety"
                )
                self.timeout_reported = True
            return

        self.timeout_reported = False

        if self.state == "WAITING":
            self.publish_command(0.0, 0.0)
            if now - self.first_odom_time >= self.start_delay:
                self.begin_experiment(now)
            return

        if now - self.experiment_start_time > self.maximum_test_time:
            self.finish_experiment(now, reason="maximum_test_time")
            return

        accumulated_yaw = self.yaw_unwrapped - self.initial_yaw

        if self.state == "LOOP_CCW":
            desired_linear, desired_angular = self.circle_target(
                self.circle_ccw_center,
                direction=1.0,
            )
            self.smooth_publish(desired_linear, desired_angular, dt)

            switch_target = 2.0 * math.pi - self.first_loop_switch_margin
            if accumulated_yaw >= switch_target:
                self.state = "LOOP_CW"
                self.get_logger().info(
                    "FIGURE8 CROSSOVER: switching CCW -> CW; "
                    f"accumulated yaw={math.degrees(accumulated_yaw):.2f} deg"
                )
            return

        if self.state == "LOOP_CW":
            desired_linear, desired_angular = self.circle_target(
                self.circle_cw_center,
                direction=-1.0,
            )
            self.smooth_publish(desired_linear, desired_angular, dt)

            if accumulated_yaw <= self.final_yaw_margin:
                self.finish_experiment(now)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualFigure8()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted by user; stopping robot")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
