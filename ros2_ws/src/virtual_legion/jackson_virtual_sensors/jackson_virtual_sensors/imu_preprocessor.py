#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuPreprocessor(Node):

    def __init__(self):
        super().__init__(
            "virtual_imu_preprocessor"
        )

        self.declare_parameter(
            "input_topic",
            "/imu/data",
        )

        self.declare_parameter(
            "output_topic",
            "/imu/data_filtered",
        )

        self.declare_parameter(
            "output_frame",
            "base_link",
        )

        # Calibration obtained from V12:
        # observed orientation / integrated yaw rate
        self.declare_parameter(
            "angular_velocity_z_scale",
            0.6953,
        )

        self.declare_parameter(
            "angular_velocity_variance",
            0.0001,
        )

        self.declare_parameter(
            "linear_acceleration_variance",
            0.04,
        )

        self.input_topic = str(
            self.get_parameter(
                "input_topic"
            ).value
        )

        self.output_topic = str(
            self.get_parameter(
                "output_topic"
            ).value
        )

        self.output_frame = str(
            self.get_parameter(
                "output_frame"
            ).value
        )

        self.angular_velocity_z_scale = float(
            self.get_parameter(
                "angular_velocity_z_scale"
            ).value
        )

        self.angular_velocity_variance = float(
            self.get_parameter(
                "angular_velocity_variance"
            ).value
        )

        self.linear_acceleration_variance = float(
            self.get_parameter(
                "linear_acceleration_variance"
            ).value
        )

        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            20,
        )

        self.subscription = (
            self.create_subscription(
                Imu,
                self.input_topic,
                self.imu_callback,
                20,
            )
        )

        self.get_logger().info(
            "Virtual IMU preprocessor initialized: "
            f"{self.input_topic} -> "
            f"{self.output_topic}, "
            f"frame={self.output_frame}, "
            f"angular_velocity_z_scale="
            f"{self.angular_velocity_z_scale:.6f}"
        )

    def imu_callback(self, msg):
        output = Imu()

        output.header.stamp = (
            msg.header.stamp
        )

        output.header.frame_id = (
            self.output_frame
        )

        # Orientation is retained for analysis,
        # but is not supplied to the EKF.
        output.orientation = (
            msg.orientation
        )

        output.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        output.angular_velocity.x = (
            msg.angular_velocity.x
        )

        output.angular_velocity.y = (
            msg.angular_velocity.y
        )

        output.angular_velocity.z = (
            self.angular_velocity_z_scale
            * msg.angular_velocity.z
        )

        angular_variance = (
            self.angular_velocity_variance
        )

        output.angular_velocity_covariance = [
            angular_variance, 0.0, 0.0,
            0.0, angular_variance, 0.0,
            0.0, 0.0, angular_variance,
        ]

        output.linear_acceleration = (
            msg.linear_acceleration
        )

        acceleration_variance = (
            self.linear_acceleration_variance
        )

        output.linear_acceleration_covariance = [
            acceleration_variance, 0.0, 0.0,
            0.0, acceleration_variance, 0.0,
            0.0, 0.0, acceleration_variance,
        ]

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = ImuPreprocessor()

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
