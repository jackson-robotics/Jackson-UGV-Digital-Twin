#!/usr/bin/env python3

import math

import rclpy

from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


class JointStateOdometry(Node):

    def __init__(self):
        super().__init__(
            "joint_state_odometry"
        )

        # Jackson virtual geometry
        self.declare_parameter(
            "wheel_radius",
            0.033,
        )

        self.declare_parameter(
            "wheel_separation",
            0.198,
        )

        # Joint names published by Isaac Sim
        self.declare_parameter(
            "left_joint",
            "left_wheel_joint",
        )

        self.declare_parameter(
            "right_joint",
            "right_wheel_joint",
        )

        # Both wheels showed positive velocity
        # during forward motion.
        self.declare_parameter(
            "left_sign",
            1.0,
        )

        self.declare_parameter(
            "right_sign",
            1.0,
        )

        # Output frames
        self.declare_parameter(
            "odom_frame",
            "odom",
        )

        self.declare_parameter(
            "base_frame",
            "base_link",
        )

        self.wheel_radius = float(
            self.get_parameter(
                "wheel_radius"
            ).value
        )

        self.wheel_separation = float(
            self.get_parameter(
                "wheel_separation"
            ).value
        )

        self.left_joint = str(
            self.get_parameter(
                "left_joint"
            ).value
        )

        self.right_joint = str(
            self.get_parameter(
                "right_joint"
            ).value
        )

        self.left_sign = float(
            self.get_parameter(
                "left_sign"
            ).value
        )

        self.right_sign = float(
            self.get_parameter(
                "right_sign"
            ).value
        )

        self.odom_frame = str(
            self.get_parameter(
                "odom_frame"
            ).value
        )

        self.base_frame = str(
            self.get_parameter(
                "base_frame"
            ).value
        )

        self.odometry_publisher = (
            self.create_publisher(
                Odometry,
                "/wheel/odom",
                20,
            )
        )

        self.joint_state_subscription = (
            self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                20,
            )
        )

        # Previous encoder state
        self.previous_left_position = None
        self.previous_right_position = None
        self.previous_time = None

        # Integrated planar pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.missing_joint_reported = False

        self.get_logger().info(
            "Virtual wheel odometry initialized: "
            f"radius={self.wheel_radius:.3f} m, "
            f"separation="
            f"{self.wheel_separation:.3f} m, "
            f"left_joint={self.left_joint}, "
            f"right_joint={self.right_joint}"
        )

    @staticmethod
    def stamp_to_seconds(stamp):
        return (
            float(stamp.sec)
            + float(stamp.nanosec)
            * 1.0e-9
        )

    def initialize_sample(
        self,
        left_position,
        right_position,
        timestamp,
    ):
        self.previous_left_position = (
            left_position
        )

        self.previous_right_position = (
            right_position
        )

        self.previous_time = timestamp

    def reset_odometry(
        self,
        left_position,
        right_position,
        timestamp,
    ):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.initialize_sample(
            left_position,
            right_position,
            timestamp,
        )

        self.get_logger().info(
            "Simulation time restarted; "
            "wheel odometry reset."
        )

    @staticmethod
    def create_pose_covariance():
        covariance = [0.0] * 36

        # x and y
        covariance[0] = 0.0025
        covariance[7] = 0.0025

        # z, roll and pitch are not observed
        covariance[14] = 999.0
        covariance[21] = 999.0
        covariance[28] = 999.0

        # yaw
        covariance[35] = 0.01

        return covariance

    @staticmethod
    def create_twist_covariance():
        covariance = [0.0] * 36

        # Linear x velocity
        covariance[0] = 0.001

        # Linear y velocity
        covariance[7] = 0.01

        # Linear z, roll and pitch
        covariance[14] = 999.0
        covariance[21] = 999.0
        covariance[28] = 999.0

        # Angular z velocity
        covariance[35] = 0.01

        return covariance

    def joint_state_callback(self, msg):
        try:
            left_index = msg.name.index(
                self.left_joint
            )

            right_index = msg.name.index(
                self.right_joint
            )

        except ValueError:
            if not self.missing_joint_reported:
                self.get_logger().error(
                    "Required wheel joints were "
                    "not found in /joint_states. "
                    f"Received joints: {msg.name}"
                )

                self.missing_joint_reported = True

            return

        if (
            left_index >= len(msg.position)
            or right_index >= len(msg.position)
        ):
            return

        left_position = (
            self.left_sign
            * float(
                msg.position[left_index]
            )
        )

        right_position = (
            self.right_sign
            * float(
                msg.position[right_index]
            )
        )

        timestamp = self.stamp_to_seconds(
            msg.header.stamp
        )

        # First valid measurement
        if self.previous_time is None:
            self.initialize_sample(
                left_position,
                right_position,
                timestamp,
            )

            return

        dt = timestamp - self.previous_time

        # Isaac simulation time may return to
        # zero after Stop -> Play.
        if dt <= 0.0:
            self.reset_odometry(
                left_position,
                right_position,
                timestamp,
            )

            return

        # Normalize the incremental joint
        # rotation. This removes artificial
        # +/- 2*pi discontinuities produced
        # when Isaac wraps a joint position.
        delta_left_angle = normalize_angle(
            left_position
            - self.previous_left_position
        )

        delta_right_angle = normalize_angle(
            right_position
            - self.previous_right_position
        )

        # Wheel travel
        delta_left_distance = (
            self.wheel_radius
            * delta_left_angle
        )

        delta_right_distance = (
            self.wheel_radius
            * delta_right_angle
        )

        # Differential-drive increment
        delta_distance = 0.5 * (
            delta_right_distance
            + delta_left_distance
        )

        delta_yaw = (
            delta_right_distance
            - delta_left_distance
        ) / self.wheel_separation

        # Midpoint integration
        middle_yaw = (
            self.yaw
            + 0.5 * delta_yaw
        )

        self.x += (
            delta_distance
            * math.cos(middle_yaw)
        )

        self.y += (
            delta_distance
            * math.sin(middle_yaw)
        )

        self.yaw = normalize_angle(
            self.yaw + delta_yaw
        )

        linear_velocity = (
            delta_distance / dt
        )

        angular_velocity = (
            delta_yaw / dt
        )

        odometry = Odometry()

        odometry.header.stamp = (
            msg.header.stamp
        )

        odometry.header.frame_id = (
            self.odom_frame
        )

        odometry.child_frame_id = (
            self.base_frame
        )

        odometry.pose.pose.position.x = (
            self.x
        )

        odometry.pose.pose.position.y = (
            self.y
        )

        odometry.pose.pose.position.z = 0.0

        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0

        odometry.pose.pose.orientation.z = (
            math.sin(
                0.5 * self.yaw
            )
        )

        odometry.pose.pose.orientation.w = (
            math.cos(
                0.5 * self.yaw
            )
        )

        odometry.twist.twist.linear.x = (
            linear_velocity
        )

        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.linear.z = 0.0

        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0

        odometry.twist.twist.angular.z = (
            angular_velocity
        )

        odometry.pose.covariance = (
            self.create_pose_covariance()
        )

        odometry.twist.covariance = (
            self.create_twist_covariance()
        )

        self.odometry_publisher.publish(
            odometry
        )

        self.initialize_sample(
            left_position,
            right_position,
            timestamp,
        )


def main(args=None):
    rclpy.init(args=args)

    node = JointStateOdometry()

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
