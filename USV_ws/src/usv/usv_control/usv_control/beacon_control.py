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


class MockDevice:
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
    ROS 2 Beacon Control Node for RobotX USV (Raspberry Pi 5).
    Controls 3 beacon lights and 1 buzzer.
    
    Features:
    - BCM GPIO pin configuration via ROS 2 parameters
    - Hardware isolation with Mock fallback when gpiozero or Pi GPIO is unavailable
    - Supports simple Bool control (ON/OFF) and Int8 mode control (0=OFF, 1=ON, 2=SLOW_BLINK, 3=FAST_BLINK)
    - Status & diagnostic publisher at 1 Hz
    - Safe hardware cleanup on node shutdown
    """

    def __init__(self):
        super().__init__('beacon_control_node')

        # Declare ROS Parameters
        self.declare_parameter('pin_light1', 17)
        self.declare_parameter('pin_light2', 27)
        self.declare_parameter('pin_light3', 22)
        self.declare_parameter('pin_buzzer', 23)

        # Retrieve parameter values
        self.pin_light1 = self.get_parameter('pin_light1').get_parameter_value().integer_value
        self.pin_light2 = self.get_parameter('pin_light2').get_parameter_value().integer_value
        self.pin_light3 = self.get_parameter('pin_light3').get_parameter_value().integer_value
        self.pin_buzzer = self.get_parameter('pin_buzzer').get_parameter_value().integer_value

        # Initialize Hardware Devices (with fallback to MockDevice)
        self.hw_light1 = self._init_device(LED, self.pin_light1, "Light1")
        self.hw_light2 = self._init_device(LED, self.pin_light2, "Light2")
        self.hw_light3 = self._init_device(LED, self.pin_light3, "Light3")
        self.hw_buzzer = self._init_device(Buzzer, self.pin_buzzer, "Buzzer")

        # Track Mode State (0=OFF, 1=ON, 2=SLOW_BLINK, 3=FAST_BLINK)
        self.modes = {
            'light1': 0,
            'light2': 0,
            'light3': 0,
            'buzzer': 0
        }

        # Subscriptions - Boolean (Simple ON/OFF)
        self.sub_light1 = self.create_subscription(Bool,
                                                   '/robotx/beacon/light1',
                                                   self.light1_callback,
                                                   10)
        self.sub_light2 = self.create_subscription(Bool,
                                                   '/robotx/beacon/light2',
                                                   self.light2_callback,
                                                   10)
        self.sub_light3 = self.create_subscription(Bool,
                                                   '/robotx/beacon/light3',
                                                   self.light3_callback,
                                                   10)
        self.sub_buzzer = self.create_subscription(Bool,
                                                   '/robotx/beacon/buzzer',
                                                   self.buzzer_callback,
                                                   10)

        # Subscriptions - Mode Int8 (0=OFF, 1=ON, 2=SLOW_BLINK, 3=FAST_BLINK)
        self.sub_light1_mode = self.create_subscription(Int8,
                                                        '/robotx/beacon/light1_mode',
                                                        self.light1_mode_callback,
                                                        10)
        self.sub_light2_mode = self.create_subscription(Int8,
                                                        '/robotx/beacon/light2_mode',
                                                        self.light2_mode_callback,
                                                        10)
        self.sub_light3_mode = self.create_subscription(Int8,
                                                        '/robotx/beacon/light3_mode',
                                                        self.light3_mode_callback,
                                                        10)
        self.sub_buzzer_mode = self.create_subscription(Int8,
                                                        '/robotx/beacon/buzzer_mode',
                                                        self.buzzer_mode_callback,
                                                        10)

        # Status Publisher & Timer
        self.pub_status = self.create_publisher(String, '/robotx/beacon/status', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info("=== BEACON CONTROL NODE INITIALIZED ===")
        self.get_logger().info(f"GPIO Pins -> Light1:{self.pin_light1}, Light2:{self.pin_light2}, Light3:{self.pin_light3}, Buzzer:{self.pin_buzzer}")
        self.get_logger().info(f"Hardware Mode: {'Real GPIO (gpiozero)' if GPIO_AVAILABLE else 'Mock Simulation'}")

    # Boolean Callbacks
    def light1_callback(self, msg: Bool):
        self._set_mode('light1', 1 if msg.data else 0)

    def light2_callback(self, msg: Bool):
        self._set_mode('light2', 1 if msg.data else 0)

    def light3_callback(self, msg: Bool):
        self._set_mode('light3', 1 if msg.data else 0)

    def buzzer_callback(self, msg: Bool):
        self._set_mode('buzzer', 1 if msg.data else 0)

    # Int8 Mode Callbacks
    def light1_mode_callback(self, msg: Int8):
        self._set_mode('light1', msg.data)

    def light2_mode_callback(self, msg: Int8):
        self._set_mode('light2', msg.data)

    def light3_mode_callback(self, msg: Int8):
        self._set_mode('light3', msg.data)

    def buzzer_mode_callback(self, msg: Int8):
        self._set_mode('buzzer', msg.data)

    def _init_device(self, device_cls, pin, name):
        if GPIO_AVAILABLE and device_cls is not None:
            try:
                return device_cls(pin)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize GPIO pin {pin} for {name}: {e}. Falling back to Mock.")
                return MockDevice(name, pin)
        else:
            return MockDevice(name, pin)

    def _set_mode(self, device_name, mode):
        if self.modes[device_name] == mode:
            return

        self.modes[device_name] = mode
        hw_device = getattr(self, f"hw_{device_name}")

        mode_names = {0: "OFF", 1: "ON", 2: "SLOW_BLINK", 3: "FAST_BLINK"}
        mode_str = mode_names.get(mode, f"UNKNOWN({mode})")
        self.get_logger().info(f"Setting {device_name} to {mode_str}")

        try:
            if mode == 0:
                hw_device.off()
            elif mode == 1:
                hw_device.on()
            elif mode == 2:
                if device_name == 'buzzer' and hasattr(hw_device, 'beep'):
                    hw_device.beep(on_time=1.0, off_time=1.0)
                else:
                    hw_device.blink(on_time=1.0, off_time=1.0)
            elif mode == 3:
                if device_name == 'buzzer' and hasattr(hw_device, 'beep'):
                    hw_device.beep(on_time=0.2, off_time=0.2)
                else:
                    hw_device.blink(on_time=0.2, off_time=0.2)
            else:
                hw_device.off()
        except Exception as e:
            self.get_logger().error(f"Error controlling hardware for {device_name}: {e}")

    def _safe_reset(self):
        for dev in ['light1', 'light2', 'light3', 'buzzer']:
            self.modes[dev] = 0
            hw_device = getattr(self, f"hw_{dev}")
            try:
                hw_device.off()
            except Exception as e:
                self.get_logger().error(f"Error turning off {dev} during safe reset: {e}")

    def _publish_status(self):
        msg = String()
        status_data = {
            'hardware_mode': 'gpiozero' if GPIO_AVAILABLE else 'mock',
            'modes': self.modes,
            'pins': {
                'light1': self.pin_light1,
                'light2': self.pin_light2,
                'light3': self.pin_light3,
                'buzzer': self.pin_buzzer
            }
        }
        msg.data = json.dumps(status_data)
        self.pub_status.publish(msg)

    def cleanup(self):
        self.get_logger().info("Cleaning up beacon GPIO hardware...")
        self._safe_reset()
        for dev in [self.hw_light1, self.hw_light2, self.hw_light3, self.hw_buzzer]:
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
