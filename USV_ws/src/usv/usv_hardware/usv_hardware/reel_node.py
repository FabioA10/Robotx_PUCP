#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool

from usv_hardware.drivers.mg6010_can import MG6010, MG6010Error

class ReelControlNode(Node):
    """
    ROS 2 Node for controlling the USV winch/reel powered by an MG6010 CAN motor.
    Encapsulates all hardware control parameters and monitoring logic inside the ROS 2 Node.
    Structured after hardware_tests/carrete/control_carrete.py.
    """

    def __init__(self):
        super().__init__('reel_control')

        # Internal defaults matching control_carrete.py
        DEFAULT_MOTOR_ID = 1
        DEFAULT_CAN_CHANNEL = 'can0'
        DEFAULT_REDUCTION = 6
        DEFAULT_MAX_SPEED_DPS = 1200.0
        DEFAULT_TOLERANCE_DEG = 1.5
        DEFAULT_PUBLISH_RATE = 10.0

        # Declare ROS 2 parameters with internal defaults
        self.declare_parameter('motor_id', DEFAULT_MOTOR_ID)
        self.declare_parameter('can_channel', DEFAULT_CAN_CHANNEL)
        self.declare_parameter('reduction', DEFAULT_REDUCTION)
        self.declare_parameter('max_speed_dps', DEFAULT_MAX_SPEED_DPS)
        self.declare_parameter('tolerance_deg', DEFAULT_TOLERANCE_DEG)
        self.declare_parameter('publish_rate', DEFAULT_PUBLISH_RATE)

        # Retrieve parameter values
        self.motor_id = self.get_parameter('motor_id').get_parameter_value().integer_value
        self.can_channel = self.get_parameter('can_channel').get_parameter_value().string_value
        self.reduction = self.get_parameter('reduction').get_parameter_value().integer_value
        self.max_speed_dps = self.get_parameter('max_speed_dps').get_parameter_value().double_value
        self.tolerance_deg = self.get_parameter('tolerance_deg').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # State variables
        self.motor = None
        self.is_connected = False
        self.target_angle = None
        self.is_moving = False
        self.last_pos = 0.0
        self.static_ticks = 0
        self.consecutive_errors = 0

        # Subscriptions (Single canonical command topic)
        self.sub_cmd_angle = self.create_subscription(Float64,
                                                      '/usv/reel/cmd_angle',
                                                      self.cmd_angle_callback,
                                                      10)
        self.sub_stop = self.create_subscription(Bool,
                                                 '/usv/reel/stop',
                                                 self.stop_callback,
                                                 10)

        # Publishers
        self.pub_current_angle = self.create_publisher(Float64,
                                                       '/usv/reel/current_angle',
                                                       10)
        self.pub_status = self.create_publisher(String,
                                                '/usv/reel/status',
                                                10)
        self.pub_is_moving = self.create_publisher(Bool,
                                                   '/usv/reel/is_moving',
                                                   10)

        # Connect to MG6010 motor
        self.init_motor()

        # Main timer loop
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info(
            f"=== Reel Control Node Started ===\n"
            f"CAN Interface: '{self.can_channel}' | Motor ID: {self.motor_id} | Reduction: {self.reduction}:1\n"
            f"Max Speed: {self.max_speed_dps} dps | Tolerance: {self.tolerance_deg}°\n"
            f"Listening on: '/usv/reel/cmd_angle'\n"
            f"Publishing to: '/usv/reel/current_angle', '/usv/reel/status', '/usv/reel/is_moving'"
        )

    def init_motor(self):
        """Initialize CAN connection and enable MG6010 motor safely."""
        self.get_logger().info(f"[+] Inicializando conexión con el motor MG6010 (ID: {self.motor_id})...")
        
        # Safely shut down existing bus if reconnecting
        if self.motor is not None:
            try:
                self.motor.disconnect()
            except Exception:
                pass
            self.motor = None

        try:
            self.motor = MG6010(
                motor_id=self.motor_id,
                channel=self.can_channel,
                reduction=self.reduction
            )
            self.motor.connect()
            self.motor.enable_motor()
            self.motor.clear_error()

            pos_actual = self.motor.read_position()
            self.last_pos = pos_actual
            self.is_connected = True
            self.consecutive_errors = 0
            self.get_logger().info(f"[+] Conexión exitosa. Posición inicial: {pos_actual:.2f}°")

            # If node was actively moving prior to reconnect, re-issue target command
            if self.is_moving and self.target_angle is not None:
                self.get_logger().info(f"[+] Re-enviando comando de movimiento a {self.target_angle:.2f}° tras reconexión CAN.")
                self.motor.move_to_angle(self.target_angle, max_speed_dps=self.max_speed_dps)
                self.static_ticks = 0

        except MG6010Error as e:
            self.get_logger().error(f"[-] Error al conectar con el motor MG6010 en {self.can_channel}: {e}")
            self.is_connected = False
        except Exception as e:
            self.get_logger().error(f"[-] Excepción inesperada en inicialización del motor: {e}")
            self.is_connected = False

    def cmd_angle_callback(self, msg: Float64):
        self.set_target_angle(msg.data)

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("[!] Comando de Parada de Emergencia recibido para el carrete.")
            self.stop_motor()

    def set_target_angle(self, target_deg: float):
        if not self.is_connected or self.motor is None:
            self.get_logger().warn("[-] No se puede mover el carrete: motor no conectado.")
            return

        try:
            pos_actual = self.motor.read_position()
            self.get_logger().info(f"[+] Posición actual: {pos_actual:.2f}°. Objetivo solicitado: {target_deg:.2f}°")

            if abs(pos_actual - target_deg) < 1.0:
                self.get_logger().info("[+] El motor ya se encuentra en la posición objetivo.")
                self.is_moving = False
                self.target_angle = target_deg
                return

            self.target_angle = target_deg
            self.static_ticks = 0
            self.last_pos = pos_actual

            self.get_logger().info(f"[+] Enviando comando: Mover a {target_deg:.2f}° a {self.max_speed_dps} dps.")
            self.motor.move_to_angle(target_deg, max_speed_dps=self.max_speed_dps)
            self.is_moving = True

        except MG6010Error as e:
            self.get_logger().error(f"[-] Error enviando comando de movimiento: {e}")
            self.is_moving = False

    def stop_motor(self):
        """Safely stop motor and halt movement tracking."""
        self.is_moving = False
        self.target_angle = None
        if self.is_connected and self.motor:
            try:
                self.motor.stop()
                self.get_logger().info("[+] Motor detenido (Stop enviado).")
            except Exception as e:
                self.get_logger().error(f"[-] Error al detener el motor: {e}")

    def control_loop(self):
        msg_is_moving = Bool(data=self.is_moving)
        self.pub_is_moving.publish(msg_is_moving)

        if not self.is_connected or self.motor is None:
            status_str = f"ERROR: CAN disconnect on {self.can_channel}"
            self.pub_status.publish(String(data=status_str))
            self.init_motor()
            return

        try:
            pos_actual = self.motor.read_position()
            status = self.motor.read_status()
            speed_dps = abs(status.speed_dps)

            # Successful read -> reset error counter
            self.consecutive_errors = 0

            # Publish current angle
            self.pub_current_angle.publish(Float64(data=pos_actual))

            if self.is_moving and self.target_angle is not None:
                # 1. Target reached check
                if abs(pos_actual - self.target_angle) <= self.tolerance_deg and speed_dps == 0:
                    self.is_moving = False
                    self.get_logger().info("==================================================")
                    self.get_logger().info(f"[¡ÉXITO!] Movimiento concluido.")
                    self.get_logger().info(f"[>] POSICIÓN FINAL REAL: {pos_actual:.2f}°")
                    self.get_logger().info("==================================================")
                    self.target_angle = None

                # 2. Static position stall check (only when speed is 0)
                elif abs(pos_actual - self.last_pos) < 0.2 and speed_dps == 0:
                    self.static_ticks += 1
                    if self.static_ticks > 10:
                        self.get_logger().warn(f"[!] Movimiento estancado sin cambios en {pos_actual:.2f}°. Finalizando comando.")
                        self.is_moving = False
                        self.target_angle = None
                else:
                    self.static_ticks = 0

                self.last_pos = pos_actual

            # Publish status message
            state_str = "MOVING" if self.is_moving else "IDLE"
            target_str = f"{self.target_angle:.2f}°" if self.target_angle is not None else "None"
            status_str = f"State: {state_str} | Pos: {pos_actual:.2f}° | Target: {target_str} | Speed: {speed_dps} dps | Temp: {status.temperature_c}°C"
            self.pub_status.publish(String(data=status_str))

        except MG6010Error as e:
            self.consecutive_errors += 1
            if self.consecutive_errors < 3:
                self.get_logger().warn(f"[-] Timeout temporal en lectura CAN ({self.consecutive_errors}/3): {e}")
            else:
                self.get_logger().error(f"[-] Pérdida de conexión CAN tras 3 intentos: {e}")
                self.is_connected = False
        except Exception as e:
            self.get_logger().error(f"[-] Excepción en ciclo de control: {e}")

    def cleanup(self):
        """Cleanup motor connection preserving multivuelta RAM memory."""
        if self.is_connected and self.motor:
            try:
                self.motor.stop()
                self.motor.disconnect()
                self.get_logger().info("[+] Interfaz CAN liberada. (Motor en Stop, memoria preservada)")
            except Exception as e:
                self.get_logger().error(f"[-] Error cerrando conexión con el motor: {e}")
            finally:
                self.is_connected = False
                self.motor = None


def main(args=None):
    rclpy.init(args=args)
    node = ReelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
