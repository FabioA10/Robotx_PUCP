import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String
from mavros_msgs.msg import State
import json

try:
    from gpiozero import LED, Buzzer
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    LED = None
    Buzzer = None


# =============================================================================
# GPIO PIN → COLOR MAPPING (BCM numbering, Raspberry Pi 5)
# =============================================================================
#   GPIO 17  →  Luz ROJA     (reservada / emergencia)
#   GPIO 27  →  Luz AMARILLA → activa en MODO MANUAL
#   GPIO 22  →  Luz VERDE    → activa en MODO AUTÓNOMO
#   GPIO 23  →  BUZZER
# =============================================================================

# =============================================================================
# MODOS MAVLINK → MODO USV
# Modos MANUALES (mando RF tiene control total):
#   MANUAL, ACRO, LEARNING, STEERING, HOLD, LOITER (en ArduRover manual sense)
# Modos AUTÓNOMOS (el bote navega solo):
#   AUTO, GUIDED, RTL, SMART_RTL, CIRCLE, AVOID_ADSB, GUIDED_NOGPS
# =============================================================================

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
    """
    ROS 2 Beacon Control Node — RobotX USV (Raspberry Pi 5).

    Se auto-detecta leyendo /mavros/state (mode field).
    El mando de RF cambia el flight mode en ArduPilot →
    MAVROS publica en /mavros/state → este nodo actualiza la baliza.

    GPIO (BCM):
        GPIO 17 → Luz ROJA     (reservada)
        GPIO 27 → Luz AMARILLA → MODO MANUAL
        GPIO 22 → Luz VERDE    → MODO AUTÓNOMO
        GPIO 23 → BUZZER

    Comportamiento:
        MANUAL   → Luz AMARILLA fija + buzzer pitido lento (0.5s ON / 1.5s OFF)
        AUTÓNOMO → Luz VERDE fija + buzzer doble pitido al activar
        IDLE/OFF → Todo apagado

    También acepta override manual:
        /robotx/beacon/usv_mode  (Int8: 0=IDLE, 1=MANUAL, 2=AUTONOMO)
        /robotx/beacon/is_autonomous (Bool: True=autónomo, False=manual)
    """

    USV_IDLE     = 0
    USV_MANUAL   = 1
    USV_AUTONOMO = 2

    def __init__(self):
        super().__init__('beacon_control_node')

        # ── Parámetros GPIO ──────────────────────────────────────────────────
        self.declare_parameter('pin_red',    17)
        self.declare_parameter('pin_yellow', 27)
        self.declare_parameter('pin_green',  22)
        self.declare_parameter('pin_buzzer', 23)
        # aliases heredados
        self.declare_parameter('pin_light1', -1)
        self.declare_parameter('pin_light2', -1)
        self.declare_parameter('pin_light3', -1)

        self.pin_red    = self.get_parameter('pin_red').get_parameter_value().integer_value
        self.pin_yellow = self.get_parameter('pin_yellow').get_parameter_value().integer_value
        self.pin_green  = self.get_parameter('pin_green').get_parameter_value().integer_value
        self.pin_buzzer = self.get_parameter('pin_buzzer').get_parameter_value().integer_value

        p1 = self.get_parameter('pin_light1').get_parameter_value().integer_value
        p2 = self.get_parameter('pin_light2').get_parameter_value().integer_value
        p3 = self.get_parameter('pin_light3').get_parameter_value().integer_value
        if p1 > 0: self.pin_red    = p1
        if p2 > 0: self.pin_yellow = p2
        if p3 > 0: self.pin_green  = p3

        # ── Hardware ─────────────────────────────────────────────────────────
        self.hw_red    = self._init_device(LED,    self.pin_red,    "Luz ROJA    (GPIO 17)")
        self.hw_yellow = self._init_device(LED,    self.pin_yellow, "Luz AMARILLA (GPIO 27)")
        self.hw_green  = self._init_device(LED,    self.pin_green,  "Luz VERDE   (GPIO 22)")
        self.hw_buzzer = self._init_device(Buzzer, self.pin_buzzer, "Buzzer      (GPIO 23)")

        # Aliases heredados
        self.hw_light1 = self.hw_red
        self.hw_light2 = self.hw_yellow
        self.hw_light3 = self.hw_green

        # ── Estado interno ───────────────────────────────────────────────────
        self.usv_mode        = self.USV_IDLE
        self.mavros_mode     = "DESCONOCIDO"
        self.mavros_armed    = False
        self.mavros_conectado = False

        # ── Subscripción principal: estado MAVLink (mando RF) ────────────────
        # QoS compatible con MAVROS State (best effort, volatile)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos_mavros = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(
            State, '/mavros/state', self._mavros_state_callback, qos_mavros)

        # ── Override manual (tópicos opcionales) ─────────────────────────────
        self.create_subscription(
            Int8, '/robotx/beacon/usv_mode', self._usv_mode_override_callback, 10)
        self.create_subscription(
            Bool, '/robotx/beacon/is_autonomous', self._is_autonomous_callback, 10)

        # Heredados individuales (compatibilidad)
        self.create_subscription(Bool, '/robotx/beacon/light1',      lambda m: self._set_light('light1', 1 if m.data else 0), 10)
        self.create_subscription(Bool, '/robotx/beacon/light2',      lambda m: self._set_light('light2', 1 if m.data else 0), 10)
        self.create_subscription(Bool, '/robotx/beacon/light3',      lambda m: self._set_light('light3', 1 if m.data else 0), 10)
        self.create_subscription(Bool, '/robotx/beacon/buzzer',      lambda m: self._set_buzzer_mode(1 if m.data else 0), 10)
        self.create_subscription(Int8, '/robotx/beacon/light1_mode', lambda m: self._set_light('light1', m.data), 10)
        self.create_subscription(Int8, '/robotx/beacon/light2_mode', lambda m: self._set_light('light2', m.data), 10)
        self.create_subscription(Int8, '/robotx/beacon/light3_mode', lambda m: self._set_light('light3', m.data), 10)
        self.create_subscription(Int8, '/robotx/beacon/buzzer_mode', lambda m: self._set_buzzer_mode(m.data), 10)

        # ── Publisher de estado ──────────────────────────────────────────────
        self.pub_status = self.create_publisher(String, '/robotx/beacon/status', 10)
        self.create_timer(1.0, self._publish_status)

        # ── Modo inicial ─────────────────────────────────────────────────────
        self._apply_usv_mode(self.USV_IDLE)

        self.get_logger().info("=" * 58)
        self.get_logger().info("  BEACON CONTROL NODE — RobotX USV")
        self.get_logger().info("=" * 58)
        self.get_logger().info(f"  GPIO 17 → Luz ROJA     (reservada)")
        self.get_logger().info(f"  GPIO 27 → Luz AMARILLA → MODO MANUAL (mando RF)")
        self.get_logger().info(f"  GPIO 22 → Luz VERDE    → MODO AUTÓNOMO")
        self.get_logger().info(f"  GPIO 23 → BUZZER")
        self.get_logger().info(f"  Hardware: {'gpiozero REAL' if GPIO_AVAILABLE else 'MockDevice (sin GPIO real)'}")
        self.get_logger().info("=" * 58)
        self.get_logger().info("  Leyendo modo desde: /mavros/state")
        self.get_logger().info(f"  Modos MANUALES : {sorted(MODOS_MANUALES)}")
        self.get_logger().info(f"  Modos AUTÓNOMOS: {sorted(MODOS_AUTONOMOS)}")
        self.get_logger().info("=" * 58)

    # ─────────────────────────────────────────────────────────────────────────
    # Callback principal: /mavros/state (mando de RF → ArduPilot → MAVROS)
    # ─────────────────────────────────────────────────────────────────────────

    def _mavros_state_callback(self, msg: State):
        """
        Se llama automáticamente cada vez que el flight controller
        cambia de modo (manual ↔ autónomo con el mando de RF).
        """
        flight_mode = msg.mode.upper().strip()
        armed        = msg.armed
        connected    = msg.connected

        # Detectar cambio real de modo
        if (flight_mode == self.mavros_mode and
                armed == self.mavros_armed and
                connected == self.mavros_conectado):
            return

        self.mavros_mode      = flight_mode
        self.mavros_armed     = armed
        self.mavros_conectado = connected

        self.get_logger().info(
            f"[MAVROS] Mode={flight_mode} | Armed={armed} | Connected={connected}")

        # Mapear flight mode → modo USV
        if not connected:
            nuevo_modo = self.USV_IDLE
        elif flight_mode in MODOS_AUTONOMOS:
            nuevo_modo = self.USV_AUTONOMO
        elif flight_mode in MODOS_MANUALES:
            nuevo_modo = self.USV_MANUAL
        else:
            # Modo desconocido → tratar como manual por seguridad
            self.get_logger().warn(
                f"[BEACON] Modo MAVLink '{flight_mode}' no clasificado → tratando como MANUAL")
            nuevo_modo = self.USV_MANUAL

        self._apply_usv_mode(nuevo_modo)

    # ─────────────────────────────────────────────────────────────────────────
    # Overrides manuales (por tópico, opcionales)
    # ─────────────────────────────────────────────────────────────────────────

    def _usv_mode_override_callback(self, msg: Int8):
        mode = int(msg.data)
        if mode not in (self.USV_IDLE, self.USV_MANUAL, self.USV_AUTONOMO):
            self.get_logger().warn(f"[BEACON] Modo override inválido: {mode}")
            return
        self.get_logger().info(f"[BEACON] Override manual de modo: {mode}")
        self._apply_usv_mode(mode)

    def _is_autonomous_callback(self, msg: Bool):
        self._apply_usv_mode(self.USV_AUTONOMO if msg.data else self.USV_MANUAL)

    # ─────────────────────────────────────────────────────────────────────────
    # Aplicar modo → GPIO
    # ─────────────────────────────────────────────────────────────────────────

    def _apply_usv_mode(self, mode: int):
        if mode == self.usv_mode and mode != self.USV_IDLE:
            return  # Sin cambio real

        self.usv_mode = mode
        mode_names = {0: "IDLE", 1: "MANUAL", 2: "AUTONOMO"}
        self.get_logger().info(
            f"[BEACON] ══► Aplicando modo: {mode_names.get(mode, '?')}")

        # Apagar todo primero
        self._all_lights_off()
        self._stop_buzzer()

        if mode == self.USV_IDLE:
            pass  # Todo apagado

        elif mode == self.USV_MANUAL:
            # ── MANUAL: Amarilla fija + pitido lento ─────────────────────────
            try:
                self.hw_yellow.on()
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error luz amarilla: {e}")
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.5, off_time=1.5, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.5, off_time=1.5, background=True)
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error buzzer manual: {e}")
            self.get_logger().info("[BEACON] 🟡 Luz AMARILLA ON + buzzer lento")

        elif mode == self.USV_AUTONOMO:
            # ── AUTÓNOMO: Verde fija + doble beep de confirmación ────────────
            try:
                self.hw_green.on()
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error luz verde: {e}")
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.1, off_time=0.1, n=2, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.1, off_time=0.1, n=2, background=True)
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error buzzer autónomo: {e}")
            self.get_logger().info("[BEACON] 🟢 Luz VERDE ON + doble beep")

    # ─────────────────────────────────────────────────────────────────────────
    # Control individual heredado
    # ─────────────────────────────────────────────────────────────────────────

    def _set_light(self, name: str, mode: int):
        hw = getattr(self, f'hw_{name}', None)
        if hw is None:
            return
        try:
            if mode == 0:   hw.off()
            elif mode == 1: hw.on()
            elif mode == 2: hw.blink(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3: hw.blink(on_time=0.2, off_time=0.2, background=True)
        except Exception as e:
            self.get_logger().error(f"[BEACON] Error {name}: {e}")

    def _set_buzzer_mode(self, mode: int):
        try:
            if mode == 0:   self._stop_buzzer()
            elif mode == 1: self.hw_buzzer.on() if hasattr(self.hw_buzzer, 'on') else None
            elif mode == 2: self.hw_buzzer.beep(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3: self.hw_buzzer.beep(on_time=0.2, off_time=0.2, background=True)
        except Exception as e:
            self.get_logger().error(f"[BEACON] Error buzzer: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _all_lights_off(self):
        for hw in [self.hw_red, self.hw_yellow, self.hw_green]:
            try: hw.off()
            except Exception: pass

    def _stop_buzzer(self):
        try: self.hw_buzzer.off()
        except Exception: pass

    def _init_device(self, device_cls, pin, name):
        if GPIO_AVAILABLE and device_cls is not None:
            try:
                return device_cls(pin)
            except Exception as e:
                self.get_logger().error(
                    f"[BEACON] GPIO {pin} ({name}) falló: {e}. Usando Mock.")
                return MockDevice(name, pin)
        return MockDevice(name, pin)

    def _publish_status(self):
        mode_names = {0: "IDLE", 1: "MANUAL", 2: "AUTONOMO"}
        status = {
            'usv_mode':       mode_names.get(self.usv_mode, "?"),
            'mavros_mode':    self.mavros_mode,
            'mavros_armed':   self.mavros_armed,
            'mavros_conectado': self.mavros_conectado,
            'hardware':       'gpiozero' if GPIO_AVAILABLE else 'mock',
            'pins': {
                'red (GPIO17)':    self.pin_red,
                'yellow (GPIO27)': self.pin_yellow,
                'green (GPIO22)':  self.pin_green,
                'buzzer (GPIO23)': self.pin_buzzer,
            },
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def cleanup(self):
        self.get_logger().info("[BEACON] Apagando hardware de forma segura...")
        self._all_lights_off()
        self._stop_buzzer()
        for dev in [self.hw_red, self.hw_yellow, self.hw_green, self.hw_buzzer]:
            if hasattr(dev, 'close'):
                try: dev.close()
                except Exception: pass


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
