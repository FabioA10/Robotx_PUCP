import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8, String
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
#   GPIO 17  →  Luz ROJA    (pin_red   / light1)
#   GPIO 27  →  Luz AMARILLA (pin_yellow / light2)
#   GPIO 22  →  Luz VERDE   (pin_green  / light3)
#   GPIO 23  →  BUZZER
# =============================================================================

# =============================================================================
# MODOS DE OPERACIÓN USV
# =============================================================================
#   USV_MODE_IDLE     = 0  → Todas las luces APAGADAS, buzzer silencioso
#   USV_MODE_MANUAL   = 1  → Luz AMARILLA fija + buzzer pitido lento (1 Hz)
#   USV_MODE_AUTONOMO = 2  → Luz VERDE fija + buzzer doble pitido al cambiar
# =============================================================================


class MockDevice:
    """Fallback simulado cuando gpiozero / GPIO no está disponible."""

    def __init__(self, name, pin):
        self.name = name
        self.pin = pin
        self.is_active = False

    def on(self):
        self.is_active = True

    def off(self):
        self.is_active = False

    def blink(self, on_time=1.0, off_time=1.0, n=None, background=True):
        self.is_active = True

    def beep(self, on_time=1.0, off_time=1.0, n=None, background=True):
        self.is_active = True

    def close(self):
        self.is_active = False


class BeaconControlNode(Node):
    """
    ROS 2 Beacon Control Node — RobotX USV (Raspberry Pi 5).

    Pines GPIO (BCM):
        GPIO 17 → Luz ROJA    (pin_red)
        GPIO 27 → Luz AMARILLA (pin_yellow)
        GPIO 22 → Luz VERDE   (pin_green)
        GPIO 23 → BUZZER

    Modos de operación principales (/robotx/beacon/usv_mode, Int8):
        0 = IDLE     → Todas las luces apagadas, buzzer silencioso
        1 = MANUAL   → Luz AMARILLA fija + buzzer pitido lento (cada 1 s)
        2 = AUTONOMO → Luz VERDE fija + buzzer doble pitido al activar

    También mantiene compatibilidad con los tópicos individuales heredados:
        /robotx/beacon/light{1,2,3}       (Bool  → ON/OFF)
        /robotx/beacon/light{1,2,3}_mode  (Int8  → 0=OFF,1=ON,2=SLOW_BLINK,3=FAST_BLINK)
        /robotx/beacon/buzzer             (Bool  → ON/OFF)
        /robotx/beacon/buzzer_mode        (Int8  → 0=OFF,1=ON,2=SLOW_BEEP,3=FAST_BEEP)
    """

    # Constantes de modo USV
    USV_IDLE     = 0
    USV_MANUAL   = 1
    USV_AUTONOMO = 2

    def __init__(self):
        super().__init__('beacon_control_node')

        # ── Parámetros GPIO ──────────────────────────────────────────────────
        self.declare_parameter('pin_red',    17)   # Luz ROJA
        self.declare_parameter('pin_yellow', 27)   # Luz AMARILLA
        self.declare_parameter('pin_green',  22)   # Luz VERDE
        self.declare_parameter('pin_buzzer', 23)   # Buzzer

        # Aliases heredados (light1=rojo, light2=amarillo, light3=verde)
        self.declare_parameter('pin_light1', -1)
        self.declare_parameter('pin_light2', -1)
        self.declare_parameter('pin_light3', -1)

        self.pin_red    = self.get_parameter('pin_red').get_parameter_value().integer_value
        self.pin_yellow = self.get_parameter('pin_yellow').get_parameter_value().integer_value
        self.pin_green  = self.get_parameter('pin_green').get_parameter_value().integer_value
        self.pin_buzzer = self.get_parameter('pin_buzzer').get_parameter_value().integer_value

        # Sobreescribir con alias heredados si vienen configurados
        p1 = self.get_parameter('pin_light1').get_parameter_value().integer_value
        p2 = self.get_parameter('pin_light2').get_parameter_value().integer_value
        p3 = self.get_parameter('pin_light3').get_parameter_value().integer_value
        if p1 > 0: self.pin_red    = p1
        if p2 > 0: self.pin_yellow = p2
        if p3 > 0: self.pin_green  = p3

        # ── Inicializar hardware ─────────────────────────────────────────────
        self.hw_red    = self._init_device(LED,    self.pin_red,    "Luz ROJA (GPIO 17)")
        self.hw_yellow = self._init_device(LED,    self.pin_yellow, "Luz AMARILLA (GPIO 27)")
        self.hw_green  = self._init_device(LED,    self.pin_green,  "Luz VERDE (GPIO 22)")
        self.hw_buzzer = self._init_device(Buzzer, self.pin_buzzer, "Buzzer (GPIO 23)")

        # Aliases para compatibilidad con código heredado
        self.hw_light1 = self.hw_red
        self.hw_light2 = self.hw_yellow
        self.hw_light3 = self.hw_green

        # ── Estado interno ───────────────────────────────────────────────────
        self.usv_mode = self.USV_IDLE   # Modo de operación actual
        self.modes = {                  # Para control individual heredado
            'light1': 0, 'light2': 0, 'light3': 0, 'buzzer': 0
        }

        # ── Subscripciones principales ───────────────────────────────────────
        # Modo USV: 0=IDLE, 1=MANUAL, 2=AUTONOMO
        self.create_subscription(
            Int8, '/robotx/beacon/usv_mode', self._usv_mode_callback, 10)

        # Compatibilidad: tópico Bool "is_autonomous" (True=autónomo, False=manual)
        self.create_subscription(
            Bool, '/robotx/beacon/is_autonomous', self._is_autonomous_callback, 10)

        # ── Subscripciones heredadas (control individual) ────────────────────
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

        # ── Activar modo inicial ─────────────────────────────────────────────
        self._apply_usv_mode(self.USV_IDLE)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  BEACON CONTROL NODE — RobotX USV")
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"  GPIO 17 → Luz ROJA    (pin_red)")
        self.get_logger().info(f"  GPIO 27 → Luz AMARILLA (pin_yellow)")
        self.get_logger().info(f"  GPIO 22 → Luz VERDE   (pin_green)")
        self.get_logger().info(f"  GPIO 23 → BUZZER")
        self.get_logger().info(f"  Hardware: {'gpiozero (REAL)' if GPIO_AVAILABLE else 'MockDevice (SIMULADO)'}")
        self.get_logger().info("=" * 55)
        self.get_logger().info("  Publicar modo en: /robotx/beacon/usv_mode")
        self.get_logger().info("    Int8: 0=IDLE | 1=MANUAL | 2=AUTONOMO")

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks de modo USV
    # ─────────────────────────────────────────────────────────────────────────

    def _usv_mode_callback(self, msg: Int8):
        """Recibe 0=IDLE, 1=MANUAL, 2=AUTONOMO y aplica el patrón de baliza."""
        mode = int(msg.data)
        if mode not in (self.USV_IDLE, self.USV_MANUAL, self.USV_AUTONOMO):
            self.get_logger().warn(f"[BEACON] Modo USV desconocido: {mode}. Se ignora.")
            return
        if mode == self.usv_mode:
            return
        self._apply_usv_mode(mode)

    def _is_autonomous_callback(self, msg: Bool):
        """Compatibilidad: Bool True→AUTONOMO, False→MANUAL."""
        self._apply_usv_mode(self.USV_AUTONOMO if msg.data else self.USV_MANUAL)

    def _apply_usv_mode(self, mode: int):
        """
        Aplica el patrón de luces y buzzer según el modo de operación.

        IDLE     → Todo apagado
        MANUAL   → Luz AMARILLA fija + buzzer pitido lento (1 s ON, 1 s OFF)
        AUTONOMO → Luz VERDE fija + buzzer doble pitido corto al activar
        """
        self.usv_mode = mode
        mode_names = {0: "IDLE", 1: "MANUAL", 2: "AUTONOMO"}
        self.get_logger().info(f"[BEACON] ══► Modo USV: {mode_names.get(mode, '?')}")

        # Apagar todo primero
        self._all_lights_off()
        self._stop_buzzer()

        if mode == self.USV_IDLE:
            # Todo apagado — ya hecho arriba
            pass

        elif mode == self.USV_MANUAL:
            # ── MODO MANUAL ──────────────────────────────────────────────────
            # Luz AMARILLA fija (estable, no parpadea — visible y sin confundir)
            try:
                self.hw_yellow.on()
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error luz amarilla: {e}")

            # Buzzer: pitido lento periódico (1 s encendido, 1 s apagado)
            # Señal acústica de "precaución: control manual"
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.5, off_time=1.5, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.5, off_time=1.5, background=True)
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error buzzer manual: {e}")

            self.get_logger().info("[BEACON] 🟡 Luz AMARILLA ON + Buzzer pitido lento")

        elif mode == self.USV_AUTONOMO:
            # ── MODO AUTÓNOMO ─────────────────────────────────────────────────
            # Luz VERDE fija (el USV navega solo — green = go)
            try:
                self.hw_green.on()
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error luz verde: {e}")

            # Buzzer: doble pitido rápido al activar (2 beeps cortos de confirmación)
            # Señal acústica de "modo autónomo activado"
            try:
                if hasattr(self.hw_buzzer, 'beep'):
                    self.hw_buzzer.beep(on_time=0.1, off_time=0.1, n=2, background=True)
                else:
                    self.hw_buzzer.blink(on_time=0.1, off_time=0.1, n=2, background=True)
            except Exception as e:
                self.get_logger().error(f"[BEACON] Error buzzer autónomo: {e}")

            self.get_logger().info("[BEACON] 🟢 Luz VERDE ON + Buzzer doble pitido")

    # ─────────────────────────────────────────────────────────────────────────
    # Control individual heredado
    # ─────────────────────────────────────────────────────────────────────────

    def _set_light(self, name: str, mode: int):
        """Control directo de una luz individual (para uso heredado / debug)."""
        if self.modes[name] == mode:
            return
        self.modes[name] = mode
        hw = getattr(self, f'hw_{name}')
        mode_names = {0: "OFF", 1: "ON", 2: "SLOW_BLINK", 3: "FAST_BLINK"}
        self.get_logger().info(f"[BEACON] {name} → {mode_names.get(mode, mode)}")
        try:
            if mode == 0:   hw.off()
            elif mode == 1: hw.on()
            elif mode == 2: hw.blink(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3: hw.blink(on_time=0.2, off_time=0.2, background=True)
            else:           hw.off()
        except Exception as e:
            self.get_logger().error(f"[BEACON] Error controlando {name}: {e}")

    def _set_buzzer_mode(self, mode: int):
        """Control directo del buzzer (para uso heredado / debug)."""
        if self.modes['buzzer'] == mode:
            return
        self.modes['buzzer'] = mode
        try:
            if mode == 0:
                self._stop_buzzer()
            elif mode == 1:
                self.hw_buzzer.on() if hasattr(self.hw_buzzer, 'on') else None
            elif mode == 2:
                self.hw_buzzer.beep(on_time=1.0, off_time=1.0, background=True)
            elif mode == 3:
                self.hw_buzzer.beep(on_time=0.2, off_time=0.2, background=True)
        except Exception as e:
            self.get_logger().error(f"[BEACON] Error controlando buzzer: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers internos
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
                    f"[BEACON] Error al inicializar GPIO {pin} ({name}): {e}. Usando Mock.")
                return MockDevice(name, pin)
        return MockDevice(name, pin)

    def _publish_status(self):
        mode_names = {0: "IDLE", 1: "MANUAL", 2: "AUTONOMO"}
        status_data = {
            'usv_mode':      mode_names.get(self.usv_mode, "DESCONOCIDO"),
            'hardware_mode': 'gpiozero' if GPIO_AVAILABLE else 'mock',
            'pins': {
                'red (GPIO 17)':    self.pin_red,
                'yellow (GPIO 27)': self.pin_yellow,
                'green (GPIO 22)':  self.pin_green,
                'buzzer (GPIO 23)': self.pin_buzzer,
            },
            'light_modes': self.modes,
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.pub_status.publish(msg)

    def cleanup(self):
        self.get_logger().info("[BEACON] Apagando hardware GPIO de forma segura...")
        self._apply_usv_mode(self.USV_IDLE)
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
