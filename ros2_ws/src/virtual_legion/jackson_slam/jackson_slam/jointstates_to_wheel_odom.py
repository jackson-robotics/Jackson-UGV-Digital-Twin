import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def angle_delta(new, old):
    return math.atan2(math.sin(new - old), math.cos(new - old))


class JointStatesToWheelOdom(Node):

    def __init__(self):
        super().__init__('jointstates_to_wheel_odom')

        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.192)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('publish_tf', False)

        self.declare_parameter('left_sign', 1.0)
        self.declare_parameter('right_sign', 1.0)
        self.declare_parameter('yaw_sign', 1.0)

        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.left_sign = float(self.get_parameter('left_sign').value)
        self.right_sign = float(self.get_parameter('right_sign').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('joint_states_to_wheel_odom iniciado')
        self.get_logger().info(f'left_joint_name: {self.left_joint_name}')
        self.get_logger().info(f'right_joint_name: {self.right_joint_name}')
        self.get_logger().info(f'odom_topic: {self.odom_topic}')
        self.get_logger().info(f'publish_tf: {self.publish_tf}')

    def joint_callback(self, msg):
        if self.left_joint_name not in msg.name or self.right_joint_name not in msg.name:
            self.get_logger().warn(
                f'No encuentro joints. Recibidos: {msg.name}',
                throttle_duration_sec=3.0
            )
            return

        left_index = msg.name.index(self.left_joint_name)
        right_index = msg.name.index(self.right_joint_name)

        if left_index >= len(msg.position) or right_index >= len(msg.position):
            self.get_logger().warn('El mensaje /joint_states no trae posiciones válidas')
            return

        left_pos = self.left_sign * msg.position[left_index]
        right_pos = self.right_sign * msg.position[right_index]

        stamp = Time.from_msg(msg.header.stamp)

        if self.prev_left is None:
            self.prev_left = left_pos
            self.prev_right = right_pos
            self.prev_time = stamp
            return

        dt = (stamp - self.prev_time).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        d_left_angle = angle_delta(left_pos, self.prev_left)
        d_right_angle = angle_delta(right_pos, self.prev_right)

        d_left = self.wheel_radius * d_left_angle
        d_right = self.wheel_radius * d_right_angle

        ds = 0.5 * (d_right + d_left)
        dyaw = self.yaw_sign * ((d_right - d_left) / self.wheel_separation)

        self.x += ds * math.cos(self.yaw + dyaw * 0.5)
        self.y += ds * math.sin(self.yaw + dyaw * 0.5)
        self.yaw += dyaw

        vx = ds / dt
        wz = dyaw / dt

        self.publish_odom(msg.header.stamp, vx, wz)

        self.prev_left = left_pos
        self.prev_right = right_pos
        self.prev_time = stamp

    def publish_odom(self, stamp, vx, wz):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        odom.pose.covariance = [1e6] * 36
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10

        odom.twist.covariance = [1e6] * 36
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToWheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
