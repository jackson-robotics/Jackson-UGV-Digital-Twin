#!/usr/bin/env python3

import math
import time

import rclpy
import serial

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()

    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)

    return quaternion


class ESP32OdomImu(Node):

    def __init__(self):
        super().__init__("esp32_odom_imu")

        # ==================================================
        # Parámetros de comunicación
        # ==================================================

        self.declare_parameter(
            "port",
            "/dev/ttyACM0",
        )

        self.declare_parameter(
            "baud",
            115200,
        )

        # ==================================================
        # Geometría física calibrada
        # ==================================================

        self.declare_parameter(
            "wheel_radius",
            0.033,
        )

        self.declare_parameter(
            "track_width",
            0.198,
        )

        self.declare_parameter(
            "ticks_per_rev",
            621.0,
        )

        # ==================================================
        # Signos configurables
        # ==================================================

        self.declare_parameter(
            "left_encoder_sign",
            1.0,
        )

        self.declare_parameter(
            "right_encoder_sign",
            1.0,
        )

        self.declare_parameter(
            "imu_yaw_rate_sign",
            1.0,
        )

        # ==================================================
        # Calibración del giróscopo
        # ==================================================

        self.declare_parameter(
            "gyro_calibration_duration",
            5.0,
        )

        # ==================================================
        # Frames
        # ==================================================

        self.declare_parameter(
            "odom_frame",
            "odom",
        )

        self.declare_parameter(
            "base_frame",
            "base_footprint",
        )

        self.declare_parameter(
            "imu_frame",
            "imu_link",
        )

        # ==================================================
        # Leer parámetros
        # ==================================================

        port = str(
            self.get_parameter("port").value
        )

        baud = int(
            self.get_parameter("baud").value
        )

        self.wheel_radius = float(
            self.get_parameter(
                "wheel_radius"
            ).value
        )

        self.track_width = float(
            self.get_parameter(
                "track_width"
            ).value
        )

        self.ticks_per_rev = float(
            self.get_parameter(
                "ticks_per_rev"
            ).value
        )

        self.left_sign = float(
            self.get_parameter(
                "left_encoder_sign"
            ).value
        )

        self.right_sign = float(
            self.get_parameter(
                "right_encoder_sign"
            ).value
        )

        self.imu_wz_sign = float(
            self.get_parameter(
                "imu_yaw_rate_sign"
            ).value
        )

        self.gyro_calibration_duration = float(
            self.get_parameter(
                "gyro_calibration_duration"
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

        self.imu_frame = str(
            self.get_parameter(
                "imu_frame"
            ).value
        )

        # ==================================================
        # Validaciones
        # ==================================================

        if self.wheel_radius <= 0.0:
            raise ValueError(
                "wheel_radius debe ser mayor que cero"
            )

        if self.track_width <= 0.0:
            raise ValueError(
                "track_width debe ser mayor que cero"
            )

        if self.ticks_per_rev <= 0.0:
            raise ValueError(
                "ticks_per_rev debe ser mayor que cero"
            )

        if self.gyro_calibration_duration < 0.0:
            raise ValueError(
                "gyro_calibration_duration no puede ser negativo"
            )

        self.meters_per_tick = (
            2.0
            * math.pi
            * self.wheel_radius
            / self.ticks_per_rev
        )

        # ==================================================
        # Publishers
        # ==================================================

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/wheel/odom",
            10,
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            "/imu/data",
            50,
        )

        # ==================================================
        # Estado de odometría
        # ==================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_t_ms = None
        self.last_left_ticks = None
        self.last_right_ticks = None

        # ==================================================
        # Estado de calibración del giróscopo
        # ==================================================

        self.gyro_calibration_start = None
        self.gyro_z_samples = []
        self.gyro_z_bias = 0.0
        self.gyro_calibrated = False

        # ==================================================
        # Puerto serial
        # ==================================================

        self.serial_port = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.02,
        )

        time.sleep(0.5)
        self.serial_port.reset_input_buffer()

        # La calibración comienza después de limpiar el buffer.
        self.gyro_calibration_start = (
            time.monotonic()
        )

        # La ESP32 transmite aproximadamente a 20 Hz.
        self.timer = self.create_timer(
            0.02,
            self.read_serial,
        )

        # ==================================================
        # Registros iniciales
        # ==================================================

        self.get_logger().info(
            f"ESP32: {port} @ {baud} baud"
        )

        self.get_logger().info(
            "Geometría: "
            f"radio={self.wheel_radius:.3f} m, "
            f"trocha={self.track_width:.3f} m, "
            f"ticks/rev={self.ticks_per_rev:.1f}"
        )

        self.get_logger().info(
            "Distancia por tick: "
            f"{self.meters_per_tick:.8f} m"
        )

        self.get_logger().info(
            "Calibrando gyro Z durante "
            f"{self.gyro_calibration_duration:.1f} s. "
            "Mantenga Jackson completamente inmóvil."
        )

    # ==================================================
    # Lectura serial
    # ==================================================

    def read_serial(self):
        try:
            if self.serial_port.in_waiting <= 0:
                return

            line = (
                self.serial_port
                .readline()
                .decode(
                    "utf-8",
                    errors="ignore",
                )
                .strip()
            )

        except serial.SerialException as error:
            self.get_logger().error(
                f"Error de comunicación serial: {error}"
            )
            return

        if not line:
            return

        parts = line.split(",")

        # Formato esperado:
        # t_ms,left,right,ax,ay,az,gx,gy,gz
        if (
            len(parts) != 9
            or parts[0] == "t_ms"
        ):
            return

        try:
            t_ms = int(parts[0])

            left_ticks = (
                int(parts[1])
                * self.left_sign
            )

            right_ticks = (
                int(parts[2])
                * self.right_sign
            )

            ax = float(parts[3])
            ay = float(parts[4])
            az = float(parts[5])

            gx = float(parts[6])
            gy = float(parts[7])

            raw_gz = (
                float(parts[8])
                * self.imu_wz_sign
            )

        except ValueError:
            return

        # Corregir el sesgo de gyro Z.
        corrected_gz = self.correct_gyro_z(
            raw_gz
        )

        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.publish_imu(
            stamp=stamp,
            ax=ax,
            ay=ay,
            az=az,
            gx=gx,
            gy=gy,
            gz=corrected_gz,
        )

        self.update_odometry(
            stamp=stamp,
            t_ms=t_ms,
            left_ticks=left_ticks,
            right_ticks=right_ticks,
        )

    # ==================================================
    # Calibración del giróscopo
    # ==================================================

    def correct_gyro_z(
        self,
        raw_gz: float,
    ) -> float:

        if self.gyro_calibrated:
            return (
                raw_gz
                - self.gyro_z_bias
            )

        self.gyro_z_samples.append(
            raw_gz
        )

        elapsed = (
            time.monotonic()
            - self.gyro_calibration_start
        )

        if (
            elapsed
            >= self.gyro_calibration_duration
            and len(self.gyro_z_samples) >= 20
        ):
            self.gyro_z_bias = (
                sum(self.gyro_z_samples)
                / len(self.gyro_z_samples)
            )

            self.gyro_calibrated = True

            self.get_logger().info(
                "Calibración gyro Z completada: "
                f"bias={self.gyro_z_bias:.7f} rad/s, "
                f"muestras={len(self.gyro_z_samples)}"
            )

        # Mientras se calibra, el EKF recibe giro cero.
        return 0.0

    # ==================================================
    # Publicación de la IMU
    # ==================================================

    def publish_imu(
        self,
        stamp,
        ax: float,
        ay: float,
        az: float,
        gx: float,
        gy: float,
        gz: float,
    ):
        message = Imu()

        message.header.stamp = stamp
        message.header.frame_id = (
            self.imu_frame
        )

        message.linear_acceleration.x = ax
        message.linear_acceleration.y = ay
        message.linear_acceleration.z = az

        message.angular_velocity.x = gx
        message.angular_velocity.y = gy
        message.angular_velocity.z = gz

        # La MPU6050 no proporciona orientación absoluta.
        message.orientation_covariance[0] = -1.0

        message.angular_velocity_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.0004,
        ]

        message.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04,
        ]

        self.imu_publisher.publish(
            message
        )

    # ==================================================
    # Odometría diferencial
    # ==================================================

    def update_odometry(
        self,
        stamp,
        t_ms: int,
        left_ticks: float,
        right_ticks: float,
    ):
        if self.last_t_ms is None:
            self.set_encoder_reference(
                t_ms,
                left_ticks,
                right_ticks,
            )
            return

        # Detectar reinicio o retroceso del reloj ESP32.
        if t_ms <= self.last_t_ms:
            self.get_logger().warning(
                "Reinicio o retroceso del tiempo de la ESP32"
            )

            self.set_encoder_reference(
                t_ms,
                left_ticks,
                right_ticks,
            )
            return

        dt = (
            t_ms - self.last_t_ms
        ) / 1000.0

        delta_left_ticks = (
            left_ticks
            - self.last_left_ticks
        )

        delta_right_ticks = (
            right_ticks
            - self.last_right_ticks
        )

        self.set_encoder_reference(
            t_ms,
            left_ticks,
            right_ticks,
        )

        # Rechazar intervalos inválidos.
        if (
            dt <= 0.0
            or dt > 1.0
        ):
            return

        distance_left = (
            delta_left_ticks
            * self.meters_per_tick
        )

        distance_right = (
            delta_right_ticks
            * self.meters_per_tick
        )

        distance_center = 0.5 * (
            distance_right
            + distance_left
        )

        delta_yaw = (
            distance_right
            - distance_left
        ) / self.track_width

        middle_yaw = (
            self.yaw
            + delta_yaw / 2.0
        )

        self.x += (
            distance_center
            * math.cos(middle_yaw)
        )

        self.y += (
            distance_center
            * math.sin(middle_yaw)
        )

        self.yaw = (
            self.yaw
            + delta_yaw
            + math.pi
        ) % (2.0 * math.pi) - math.pi

        linear_velocity = (
            distance_center / dt
        )

        angular_velocity = (
            delta_yaw / dt
        )

        message = Odometry()

        message.header.stamp = stamp
        message.header.frame_id = (
            self.odom_frame
        )

        message.child_frame_id = (
            self.base_frame
        )

        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0

        message.pose.pose.orientation = (
            yaw_to_quaternion(
                self.yaw
            )
        )

        message.twist.twist.linear.x = (
            linear_velocity
        )

        # Restricción no holonómica:
        # Jackson no se desplaza lateralmente.
        message.twist.twist.linear.y = 0.0

        message.twist.twist.angular.z = (
            angular_velocity
        )

        # Covarianza de pose.
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.04

        # Covarianza de velocidades.
        message.twist.covariance[0] = 0.01

        # Confianza alta, pero no perfecta, en vy=0.
        message.twist.covariance[7] = 0.001

        message.twist.covariance[35] = 0.04

        self.odom_publisher.publish(
            message
        )

    # ==================================================
    # Referencia de encoders
    # ==================================================

    def set_encoder_reference(
        self,
        t_ms: int,
        left_ticks: float,
        right_ticks: float,
    ):
        self.last_t_ms = t_ms
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

    # ==================================================
    # Cierre seguro
    # ==================================================

    def destroy_node(self):
        try:
            if self.serial_port.is_open:
                self.serial_port.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ESP32OdomImu()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
