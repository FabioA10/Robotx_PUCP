import json

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String

try:
    from gpiozero import Buzzer, LED

    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    LED = None
    Buzzer = None

# GPIO 17 → Luz ROJA     → DESARMADO / SIN CONEXIÓN
# GPIO 27 → Luz AMARILLA → MODO MANUAL (armado)
# GPIO 22 → Luz VERDE    → MODO AUTÓNOMO (armado)
# GPIO 23 → BUZZER
# GPIO 21 → KEEP-ALIVE RELÉ (HIGH=bote vivo, LOW=failsafe corta energía)

MODOS_AUTONOMOS = {
    'AUTO', 'GUIDED', 'RTL', 'SMART_RTL', 'CIRCLE',
    'AVOID_ADSB', 'GUIDED_NOGPS', 'AUTO:MISSION'
}
MODOS_MANUALES = {
    'MANUAL', 'ACRO', 'LEARNING', 'STEERING',
    'HOLD', 'LOITER', 'STABILIZE', 'SPORT'
}


class MockDevice:
    """Fallback simulado cuando gpiozero / GPIO no está disponible."""

    def __init__(self, name, pin):
        self.name = name
        self.pin = pin
        self.is_active = False

    def on(self):
        self.is_active = True
        print(f"  [MOCK] {self.name} → ON")

    def off(self):
        self.is_active = False
        print(f"  [MOCK] {self.name} → OFF")

    def blink(self, on_time=1.0, off_time=1.0, n=None, background=True):
        self.is_active = True
        print(f"  [MOCK] {self.name} → BLINK ({on_time}s/{off_time}s)")

    def beep(self, on_time=1.0, off_time=1.0, n=None, background=True):
        self.is_active = True
        print(f"  [MOCK] {self.name} → BEEP ({on_time}s/{off_time}s, n={n})")

    def close(self):
        self.is_active = False


class BeaconControlNode(Node):
    USV_IDLE = 0
    USV_MANUAL = 1
    USV_AUTONOMO = 2
    USV_ALERTA = 3

    def __init__(self):
        super().__init__('beacon_control_node')

        self.declare_parameter('pin_red', 17)
        self.declare_parameter('pin_yellow', 27)
        self.declare_parameter('pin_green', 22)
        self.declare_parameter('pin_buzzer', 23)
        self.declare_parameter('pin_keepalive', 21)
        self.declare_parameter('pin_light1', -1)
        self.declare_parameter('pin_light2', -1)
        self.declare_parameter('pin_light3', -1)

        self.pin_red = self.get_parameter('pin_red').get_parameter_value().integer_value
        self.pin_yellow = self.get_parameter('pin_yellow').get_parameter_value().integer_value
        self.pin_green = self.get_parameter('pin_green').get_parameter_value().integer_value
        self.pin_buzzer = self.get_parameter('pin_buzzer').get_parameter_value().integer_value
        self.pin_keepalive = self.get_parameter('pin_keepalive').get_parameter_value().integer_value

        p1 = self.get_parameter('pin_light1').get_parameter_value().integer_value
        p2 = self.get_parameter('pin_light2').get_parameter_value().integer_value
        p3 = self.get_parameter('pin_light3').get_parameter_value().integer_value
        if p1 > 0:
            self.pin_red = p1
        if p2 > 0:
            self.pin_yellow = p2
        if p3 > 0:
            self.pin_green = p3

        self.hw_red = self._init_device(LED, self.pin_red, "Luz ROJA    (GPIO 17)")
        self.hw_yellow = self._init_device(LED, self.pin_yellow, "Luz AMARILLA (GPIO 27)")
        self.hw_green = self._init_device(LED, self.pin_green, "Luz VERDE   (GPIO 22)")
        self.hw_buzzer = self._init_device(Buzzer, self.pin_buzzer, "Buzzer      (GPIO 23)")
        self.hw_keepalive = self._init_device(LED, self.pin_keepalive, "Keep-Alive  (GPIO 21)")

        self.hw_light1 = self.hw_red
        self.hw_light2 = self.hw_yellow
        self.hw_light3 = self.hw_green

        self.usv_mode = self.USV_IDLE
        self.mavros_mode = "DESCONOCIDO"
        self.mavros_armed = False
        self.mavros_conectado = False
        self.mavros_sys_status = 0

        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(State, '/mavros/state', self._mavros_state_callback, qos_mavros)
        self.create_subscription(Int8, '/robotx/beacon/usv_mode', self._usv_mode_override_callback, 10)
        self.create_subscription(Bool, '/robotx/beacon/is_autonomous', self._is_autonomous_callback, 10)

        self.create_subscription(Bool, '/robotx/beacon/light1', self._light1_callback, 10)
        self.create_subscription(Bool, '/robotx/beacon/light2', self._light2_callback, 10)
        self.create_subscription(Bool, '/robotx/beacon/light3', self._light3_callback, 10)
        self.create_subscription(Bool, '/robotx/beacon/buzzer', self._buzzer_callback, 10)
        self.create_subscription(Int8, '/robotx/beacon/light1_mode', self._light1_mode_callback, 10)
        self.create_subscription(Int8, '/robotx/beacon/light2_mode', self._light2_mode_callback, 10)
        self.create_subscription(Int8, '/robotx/beacon/light3_mode', self._light3_mode_callback, 10)
        self.create_subscription(Int8, '/robotx/beacon/buzzer_mode', self._buzzer_mode_callback, 10)
        self.create_subscription(RCIn, '/mavros/rc/in', self._rc_callback, qos_mavros)

        self.pub_status = self.create_publisher(String, '/robotx/beacon/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self._arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self._last_mavros_time = self.get_clock().now()
        self._last_rc_time = self.get_clock().now()
        self._herelink_ok = False
        self._herelink_perdido = False
        self._failsafe_signature_start_time = None

        self.create_timer(1.0, self._publish_status)
        self.create_timer(0.1, self._failsafe_vel_loop)
        self.create_timer(1.0, self._check_connection_timeout)
        self.create_timer(1.0, self._check_rc_timeout)

        self._set_keepalive(False)
        self._apply_usv_mode(self.USV_IDLE)

        self.get_logger().info('=' * 58)
        self.get_logger().info('  BEACON CONTROL NODE — RobotX USV')
        self.get_logger().info('  GPIO17=ROJA | GPIO27=AMARILLA | GPIO22=VERDE | GPIO23=BUZZER | GPIO21=KEEPALIVE')
        self.get_logger().info('  ROJO     → Desarmado / Sin conexión / Failsafe')
        self.get_logger().info('  AMARILLO → Armado + MANUAL')
        self.get_logger().info('  VERDE    → Armado + AUTÓNOMO')
        self.get_logger().info(f"  Hardware: {'gpiozero REAL' if GPIO_AVAILABLE else 'MockDevice'}")
        self.get_logger().info('  Leyendo modo desde: /mavros/state')
        self.get_logger().info(f'  Modos MANUALES : {sorted(MODOS_MANUALES)}')
        self.get_logger().info(f'  Modos AUTÓNOMOS: {sorted(MODOS_AUTONOMOS)}')
        self.get_logger().info('=' * 58)

    def _set_keepalive(self, activo: bool):
        try:
            if activo:
                self.hw_keepalive.on()
                self.get_logger().info('[KEEPALIVE] GPIO21 → HIGH (relé activo)')
            else:
                self.hw_keepalive.off()
                self.get_logger().warn('[KEEPALIVE] GPIO21 → LOW  (relé caído — FAILSAFE)')
        except Exception as e:
            self.get_logger().error(f'[KEEPALIVE] Error GPIO21: {e}')

    def _failsafe_vel_loop(self):
        if self.usv_mode == self.USV_ALERTA:
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)

    def _rc_callback(self, msg: RCIn):
        self._last_rc_time = self.get_clock().now()

        if not self._herelink_ok:
            self._herelink_ok = True
            self.get_logger().info('[RC] Herelink detectado (mensajes RC llegando)')

        if len(msg.channels) >= 8:
            c1, c2, c3, c4 = msg.channels[0:4]
            c7 = msg.channels[6]
            c8 = msg.channels[7]

            is_failsafe_signature = (
                1500 <= c1 <= 1530 and
                1500 <= c2 <= 1530 and
                1500 <= c3 <= 1530 and
                1500 <= c4 <= 1530 and
                c7 < 1100 and
                c8 > 1900
            )

            if is_failsafe_signature or msg.rssi == 0:
                if self._failsafe_signature_start_time is None:
                    self._failsafe_signature_start_time = self.get_clock().now()
                else:
                    dt = (self.get_clock().now() - self._failsafe_signature_start_time).nanoseconds / 1e9
                    if dt > 3.0 and not self._herelink_perdido:
                        self._herelink_perdido = True
                        self.get_logger().warn(f'[RC] Mando Herelink APAGADO detectado ({dt:.1f}s) → FAILSAFE')
                        self._trigger_rc_failsafe()
            else:
                self._failsafe_signature_start_time = None
                if self._herelink_perdido:
                    self._herelink_perdido = False
                    self.get_logger().info('[RC] Mando Herelink ENCENDIDO (Movimiento detectado)')

    def _check_rc_timeout(self):
        if not self._herelink_ok:
            return
        ahora = self.get_clock().now()
        dt = (ahora - self._last_rc_time).nanoseconds / 1e9
        if dt > 2.0 and not self._herelink_perdido:
            self._herelink_perdido = True
            self.get_logger().warn(f'[RC] Timeout {dt:.1f}s sin RC — Herelink apagado → FAILSAFE')
            self._trigger_rc_failsafe()

    def _trigger_rc_failsafe(self):
        self._set_keepalive(False)
        self._send_disarm()
        self._apply_usv_mode(self.USV_ALERTA)

    def _check_connection_timeout(self):
        ahora = self.get_clock().now()
        dt = (ahora - self._last_mavros_time).nanoseconds / 1e9
        if dt > 3.0:
            if self.mavros_conectado or self.usv_mode != self.USV_ALERTA:
                self.get_logger().warn(f'[BEACON] Timeout {dt:.1f}s sin /mavros/state → FAILSAFE')
                self.mavros_conectado = False
                self._set_keepalive(False)
                self._send_disarm()
                self._apply_usv_mode(self.USV_ALERTA)

    def _mavros_state_callback(self, msg: State):
        self._last_mavros_time = self.get_clock().now()

        flight_mode = msg.mode.upper().strip()
        armed = msg.armed
        connected = msg.connected
        sys_status = msg.system_status

        if (
            flight_mode == self.mavros_mode and
            armed == self.mavros_armed and
            connected == self.mavros_conectado and
            sys_status == self.mavros_sys_status
        ):
            return

        self.mavros_mode = flight_mode
        self.mavros_armed = armed
        self.mavros_conectado = connected
        self.mavros_sys_status = sys_status

        estado_str = {3: 'STANDBY', 4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY'}.get(sys_status, str(sys_status))
        self.get_logger().info(
            f'[MAVROS] Mode={flight_mode} | Armed={armed} | Connected={connected} | SysStatus={estado_str}'
        )

        if sys_status >= 5 and connected:
            self.get_logger().warn(f'[BEACON] SysStatus={estado_str} → RC FAILSAFE (Herelink apagado) → ROJO')
            self._trigger_rc_failsafe()
            return

        if not connected:
            self._set_keepalive(False)
            self._send_disarm()
            self._apply_usv_mode(self.USV_ALERTA)
        elif flight_mode in MODOS_AUTONOMOS:
            self._set_keepalive(True)
            self._apply_usv_mode(self.USV_AUTONOMO)
        elif flight_mode in MODOS_MANUALES:
            self._set_keepalive(True)
            self._apply_usv_mode(self.USV_MANUAL)
        else:
            self.get_logger().warn(f"[BEACON] Modo '{flight_mode}' desconocido → MANUAL")
            self._set_keepalive(True)
            self._apply_usv_mode(self.USV_MANUAL)

    def _send_disarm(self):
        if not self._arming_client.service_is_ready():
            self.get_logger().warn('[DISARM] Servicio /mavros/cmd/arming no disponible')
            return
        req = CommandBool.Request()
        req.value = False
        future = self._arming_client.call_async(req)
        future.add_done_callback(self._disarm_response_cb)
        self.get_logger().warn('[DISARM] Solicitud de desarmado enviada')

    def _disarm_response_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().warn('[DISARM] ✅ Vehículo desarmado correctamente')
            else:
                self.get_logger().error(f'[DISARM] ❌ Fallo al desarmar (result={result.result})')
        except Exception as e:
            self.get_logger().error(f'[DISARM] Error en respuesta: {e}')

    def _light1_callback(self, msg: Bool):
        self._set_light('light1', 1 if msg.data else 0)

    def _light2_callback(self, msg: Bool):
        self._set_light('light2', 1 if msg.data else 0)

    def _light3_callback(self, msg: Bool):
        self._set_light('light3', 1 if msg.data else 0)

    def _buzzer_callback(self, msg: Bool):
        self._set_buzzer_mode(1 if msg.data else 0)

    def _light1_mode_callback(self, msg: Int8):
        self._set_light('light1', int(msg.data))

    def _light2_mode_callback(self, msg: Int8):
        self._set_light('light2', int(msg.data))

    def _light3_mode_callback(self, msg: Int8):
        self._set_light('light3', int(msg.data))

    def _buzzer_mode_callback(self, msg: Int8):
        self._set_buzzer_mode(int(msg.data))

    def _usv_mode_override_callback(self, msg: Int8):
        mode = int(msg.data)
        if mode in (self.USV_IDLE, self.USV_MANUAL, self.USV_AUTONOMO, self.USV_ALERTA):
            self.get_logger().info(f'[BEACON] Override: {mode}')
            self._apply_usv_mode(mode)
        else:
            self.get_logger().warn(f'[BEACON] Modo override inválido: {mode}')

    def _is_autonomous_callback(self, msg: Bool):
        self._apply_usv_mode(self.USV_AUTONOMO if msg.data else self.USV_MANUAL)

    def _apply_usv_mode(self, mode: int):
        if mode == self.usv_mode:
            return

        self.usv_mode = mode
        names = {0: 'IDLE', 1: 'MANUAL', 2: 'AUTONOMO', 3: 'ALERTA'}
        self.get_logger().info(f'[BEACON] ══► Modo: {names.get(mode, "?")}')

        self._all_lights_off()
        self._stop_buzzer()

        if mode == self.USV_ALERTA:
            self._set_keepalive(False)
            try:
                self.hw_red.on()
            except Exception as e:
                self.get_logger().error(f'Error roja: {e}')
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.1, off_time=0.1, n=1, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.1, off_time=0.1, n=1, background=True)
            except Exception as e:
                self.get_logger().error(f'Error buzzer: {e}')
            self.get_logger().info('[BEACON] ROJA ON + 1 beep')

        elif mode == self.USV_MANUAL:
            try:
                self.hw_yellow.on()
            except Exception as e:
                self.get_logger().error(f'Error amarilla: {e}')
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.5, off_time=1.5, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.5, off_time=1.5, background=True)
            except Exception as e:
                self.get_logger().error(f'Error buzzer: {e}')
            self.get_logger().info('[BEACON] AMARILLA ON + pitido lento')

        elif mode == self.USV_AUTONOMO:
            try:
                self.hw_green.on()
            except Exception as e:
                self.get_logger().error(f'Error verde: {e}')
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.1, off_time=0.1, n=2, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.1, off_time=0.1, n=2, background=True)
            except Exception as e:
                self.get_logger().error(f'Error buzzer: {e}')
            self.get_logger().info('[BEACON] VERDE ON + 2 beeps')

    def _set_light(self, name: str, mode: int):
        hw = getattr(self, f'hw_{name}', None)
        if hw is None:
            return
        try:
            if mode == 0:
                hw.off()
            elif mode == 1:
                hw.on()
            elif mode == 2:
                hw.blink(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3:
                hw.blink(on_time=0.2, off_time=0.2, background=True)
        except Exception as e:
            self.get_logger().error(f'[BEACON] Error {name}: {e}')

    def _set_buzzer_mode(self, mode: int):
        try:
            if mode == 0:
                self._stop_buzzer()
            elif mode == 1:
                if hasattr(self.hw_buzzer, 'on'):
                    self.hw_buzzer.on()
            elif mode == 2:
                self.hw_buzzer.beep(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3:
                self.hw_buzzer.beep(on_time=0.2, off_time=0.2, background=True)
        except Exception as e:
            self.get_logger().error(f'[BEACON] Error buzzer: {e}')

    def _all_lights_off(self):
        for hw in [self.hw_red, self.hw_yellow, self.hw_green]:
            try:
                hw.off()
            except Exception:
                pass

    def _stop_buzzer(self):
        try:
            self.hw_buzzer.off()
        except Exception:
            pass

    def _init_device(self, device_cls, pin, name):
        if GPIO_AVAILABLE and device_cls is not None:
            try:
                return device_cls(pin)
            except Exception as e:
                self.get_logger().error(f'[BEACON] GPIO {pin} ({name}) falló: {e}. Usando Mock.')
                return MockDevice(name, pin)
        return MockDevice(name, pin)

    def _publish_status(self):
        mode_names = {0: 'IDLE', 1: 'MANUAL', 2: 'AUTONOMO', 3: 'ALERTA'}
        status = {
            'usv_mode': mode_names.get(self.usv_mode, '?'),
            'mavros_mode': self.mavros_mode,
            'mavros_armed': self.mavros_armed,
            'mavros_conectado': self.mavros_conectado,
            'hardware': 'gpiozero' if GPIO_AVAILABLE else 'mock',
            'keepalive': self.hw_keepalive.is_active if hasattr(self.hw_keepalive, 'is_active') else '?',
            'pins': {
                'red (GPIO17)': self.pin_red,
                'yellow (GPIO27)': self.pin_yellow,
                'green (GPIO22)': self.pin_green,
                'buzzer (GPIO23)': self.pin_buzzer,
                'keepalive (GPIO21)': self.pin_keepalive,
            },
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    def cleanup(self):
        self.get_logger().info('[BEACON] Apagando hardware de forma segura...')
        self._set_keepalive(False)
        self._all_lights_off()
        self._stop_buzzer()
        for dev in [self.hw_red, self.hw_yellow, self.hw_green, self.hw_buzzer, self.hw_keepalive]:
            if hasattr(dev, 'close'):
                try:
                    dev.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = BeaconControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
