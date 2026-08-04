#!/usr/bin/env python3

import time
from typing import Tuple

import board
import busio
from adafruit_pca9685 import PCA9685

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class JacksonMotorDriver(Node):

    def __init__(self):
        super().__init__("jackson_motor_driver")

        # ==================================================
        # Parámetros ROS
        # ==================================================

        # Entrada de velocidad
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.60)
        self.declare_parameter("control_frequency", 20.0)

        # Geometría actualizada de Jackson
        self.declare_parameter("track_width", 0.198)
        self.declare_parameter("max_wheel_speed", 0.25)

        # Configuración PWM
        self.declare_parameter("pwm_frequency", 1000)
        self.declare_parameter("max_pwm", 0.80)
        self.declare_parameter("minimum_start_pwm", 0.32)
        self.declare_parameter("minimum_run_pwm", 0.25)
        self.declare_parameter("ramp_time", 0.45)

        # Compensación independiente por rueda física
        self.declare_parameter("left_pwm_scale", 1.0)
        self.declare_parameter("right_pwm_scale", 1.0)

        # Sentidos físicos confirmados
        #
        # Motor B = rueda física izquierda
        # Motor A = rueda física derecha
        #
        # El motor B necesita inversión.
        self.declare_parameter("invert_left", True)
        self.declare_parameter("invert_right", False)

        # Canales PCA9685 confirmados
        self.declare_parameter("pwma_channel", 0)
        self.declare_parameter("pwmb_channel", 6)

        self.declare_parameter("ain1_channel", 2)
        self.declare_parameter("ain2_channel", 1)

        self.declare_parameter("bin1_channel", 4)
        self.declare_parameter("bin2_channel", 5)

        self.declare_parameter("standby_channel", 3)

        # ==================================================
        # Leer parámetros
        # ==================================================

        self.cmd_topic = str(
            self.get_parameter("cmd_topic").value
        )

        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )

        self.control_frequency = float(
            self.get_parameter("control_frequency").value
        )

        self.track_width = float(
            self.get_parameter("track_width").value
        )

        self.max_wheel_speed = float(
            self.get_parameter("max_wheel_speed").value
        )

        pwm_frequency = int(
            self.get_parameter("pwm_frequency").value
        )

        self.max_pwm = float(
            self.get_parameter("max_pwm").value
        )

        self.minimum_start_pwm = float(
            self.get_parameter("minimum_start_pwm").value
        )

        self.minimum_run_pwm = float(
            self.get_parameter("minimum_run_pwm").value
        )

        self.ramp_time = float(
            self.get_parameter("ramp_time").value
        )

        self.left_pwm_scale = float(
            self.get_parameter("left_pwm_scale").value
        )

        self.right_pwm_scale = float(
            self.get_parameter("right_pwm_scale").value
        )

        self.invert_left = bool(
            self.get_parameter("invert_left").value
        )

        self.invert_right = bool(
            self.get_parameter("invert_right").value
        )

        # ==================================================
        # Validaciones
        # ==================================================

        if self.control_frequency <= 0.0:
            raise ValueError(
                "control_frequency debe ser mayor que cero"
            )

        if self.track_width <= 0.0:
            raise ValueError(
                "track_width debe ser mayor que cero"
            )

        if self.max_wheel_speed <= 0.0:
            raise ValueError(
                "max_wheel_speed debe ser mayor que cero"
            )

        if not 0.0 < self.max_pwm <= 1.0:
            raise ValueError(
                "max_pwm debe estar en el intervalo (0, 1]"
            )

        if self.left_pwm_scale < 0.0:
            raise ValueError(
                "left_pwm_scale no puede ser negativo"
            )

        if self.right_pwm_scale < 0.0:
            raise ValueError(
                "right_pwm_scale no puede ser negativo"
            )

        # ==================================================
        # Inicializar PCA9685
        # ==================================================

        i2c = busio.I2C(
            board.SCL,
            board.SDA,
        )

        self.pca = PCA9685(i2c)
        self.pca.frequency = pwm_frequency

        # Motor A: rueda física derecha
        self.pwma = self.pca.channels[
            int(self.get_parameter("pwma_channel").value)
        ]

        self.ain1 = self.pca.channels[
            int(self.get_parameter("ain1_channel").value)
        ]

        self.ain2 = self.pca.channels[
            int(self.get_parameter("ain2_channel").value)
        ]

        # Motor B: rueda física izquierda
        self.pwmb = self.pca.channels[
            int(self.get_parameter("pwmb_channel").value)
        ]

        self.bin1 = self.pca.channels[
            int(self.get_parameter("bin1_channel").value)
        ]

        self.bin2 = self.pca.channels[
            int(self.get_parameter("bin2_channel").value)
        ]

        self.standby = self.pca.channels[
            int(
                self.get_parameter(
                    "standby_channel"
                ).value
            )
        ]

        # ==================================================
        # Estado del controlador
        # ==================================================

        self.desired_linear = 0.0
        self.desired_angular = 0.0

        self.last_command_time = None

        # Comandos normalizados de las ruedas físicas
        self.current_left = 0.0
        self.current_right = 0.0

        self.control_period = (
            1.0 / self.control_frequency
        )

        # ==================================================
        # ROS
        # ==================================================

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.command_callback,
            10,
        )

        self.timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        # El driver comienza detenido
        self.stop_hardware()

        self.get_logger().info(
            "Jackson motor driver iniciado"
        )

        self.get_logger().info(
            f"Escuchando: {self.cmd_topic}"
        )

        self.get_logger().info(
            f"Trocha={self.track_width:.3f} m, "
            f"velocidad máxima por rueda="
            f"{self.max_wheel_speed:.3f} m/s"
        )

        self.get_logger().info(
            "Asignación física confirmada: "
            "Motor B=izquierda, Motor A=derecha"
        )

        self.get_logger().info(
            f"invert_left={self.invert_left}, "
            f"invert_right={self.invert_right}"
        )

        self.get_logger().info(
            f"left_pwm_scale={self.left_pwm_scale:.3f}, "
            f"right_pwm_scale={self.right_pwm_scale:.3f}"
        )

    # ==================================================
    # Utilidades de hardware
    # ==================================================

    def digital(self, channel, state: bool):
        channel.duty_cycle = (
            0xFFFF if state else 0x0000
        )

    def set_pwm(self, channel, pwm: float):
        pwm = clamp(
            pwm,
            0.0,
            self.max_pwm,
        )

        channel.duty_cycle = int(
            pwm * 0xFFFF
        )

    # ==================================================
    # Recepción de cmd_vel
    # ==================================================

    def command_callback(self, message: Twist):
        self.desired_linear = float(
            message.linear.x
        )

        self.desired_angular = float(
            message.angular.z
        )

        self.last_command_time = (
            time.monotonic()
        )

    # ==================================================
    # Cinemática diferencial
    # ==================================================

    def twist_to_wheels(
        self,
        linear: float,
        angular: float,
    ) -> Tuple[float, float]:

        # Convención ROS:
        # angular.z positivo = giro antihorario
        #
        # Para girar a la izquierda:
        # rueda derecha más rápida
        # rueda izquierda más lenta

        left_speed = (
            linear
            - angular * self.track_width / 2.0
        )

        right_speed = (
            linear
            + angular * self.track_width / 2.0
        )

        left_command = clamp(
            left_speed / self.max_wheel_speed,
            -1.0,
            1.0,
        )

        right_command = clamp(
            right_speed / self.max_wheel_speed,
            -1.0,
            1.0,
        )

        return left_command, right_command

    # ==================================================
    # Rampa de aceleración
    # ==================================================

    def slew(
        self,
        current: float,
        target: float,
    ) -> float:

        if self.ramp_time <= 0.0:
            return target

        maximum_change = (
            self.control_period / self.ramp_time
        )

        if target > current:
            return min(
                current + maximum_change,
                target,
            )

        return max(
            current - maximum_change,
            target,
        )

    # ==================================================
    # Dirección de motores
    # ==================================================

    def set_directions(
        self,
        left_forward: bool,
        right_forward: bool,
    ):
        # Motor B = rueda física izquierda
        if self.invert_left:
            left_forward = not left_forward

        # Motor A = rueda física derecha
        if self.invert_right:
            right_forward = not right_forward

        # Motor A: rueda física derecha
        self.digital(
            self.ain1,
            right_forward,
        )
        self.digital(
            self.ain2,
            not right_forward,
        )

        # Motor B: rueda física izquierda
        self.digital(
            self.bin1,
            left_forward,
        )
        self.digital(
            self.bin2,
            not left_forward,
        )

    # ==================================================
    # Zona muerta de los motores
    # ==================================================

    def apply_deadband(
        self,
        pwm: float,
        was_moving: bool,
    ) -> float:

        if pwm <= 0.0:
            return 0.0

        if was_moving:
            minimum_pwm = (
                self.minimum_run_pwm
            )
        else:
            minimum_pwm = (
                self.minimum_start_pwm
            )

        return max(
            pwm,
            minimum_pwm,
        )

    # ==================================================
    # Lazo principal
    # ==================================================

    def control_loop(self):
        now = time.monotonic()

        command_expired = (
            self.last_command_time is None
            or (
                now - self.last_command_time
                > self.command_timeout
            )
        )

        if command_expired:
            target_left = 0.0
            target_right = 0.0

        else:
            target_left, target_right = (
                self.twist_to_wheels(
                    self.desired_linear,
                    self.desired_angular,
                )
            )

        left_was_moving = (
            abs(self.current_left) > 1e-3
        )

        right_was_moving = (
            abs(self.current_right) > 1e-3
        )

        self.current_left = self.slew(
            self.current_left,
            target_left,
        )

        self.current_right = self.slew(
            self.current_right,
            target_right,
        )

        # Parada completa
        if (
            abs(self.current_left) < 1e-3
            and abs(self.current_right) < 1e-3
        ):
            self.stop_hardware()
            return

        # Activar TB6612FNG
        self.digital(
            self.standby,
            True,
        )

        # Aplicar sentidos físicos
        self.set_directions(
            left_forward=(
                self.current_left >= 0.0
            ),
            right_forward=(
                self.current_right >= 0.0
            ),
        )

        # PWM de rueda física izquierda
        left_pwm = (
            abs(self.current_left)
            * self.max_pwm
            * self.left_pwm_scale
        )

        # PWM de rueda física derecha
        right_pwm = (
            abs(self.current_right)
            * self.max_pwm
            * self.right_pwm_scale
        )

        left_pwm = self.apply_deadband(
            left_pwm,
            left_was_moving,
        )

        right_pwm = self.apply_deadband(
            right_pwm,
            right_was_moving,
        )

        # Motor B = rueda física izquierda
        self.set_pwm(
            self.pwmb,
            left_pwm,
        )

        # Motor A = rueda física derecha
        self.set_pwm(
            self.pwma,
            right_pwm,
        )

    # ==================================================
    # Parada segura
    # ==================================================

    def stop_hardware(self):
        self.set_pwm(
            self.pwma,
            0.0,
        )

        self.set_pwm(
            self.pwmb,
            0.0,
        )

        self.digital(
            self.standby,
            False,
        )

        self.digital(
            self.ain1,
            False,
        )

        self.digital(
            self.ain2,
            False,
        )

        self.digital(
            self.bin1,
            False,
        )

        self.digital(
            self.bin2,
            False,
        )

        self.current_left = 0.0
        self.current_right = 0.0

    def destroy_node(self):
        try:
            self.stop_hardware()
        except Exception:
            pass

        try:
            self.pca.deinit()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = JacksonMotorDriver()
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
