#!/usr/bin/env python3

import serial
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


SYSTEM_INFO_COMMAND = b"#0281C1\r\n"
STATUS_COMMAND = b"#10000DC0\r\n"
MAG_CAL_RESET_COMMAND = b"#20041803\r\n"
MAG_CAL_CALCULATE_COMMAND = b"#2005D9C3\r\n"
SETTINGS_SAVE_COMMAND = b"#18000A\r\n"


def crc16_ibm(data: bytes) -> int:
    """Calcula el CRC-16-IBM utilizado por SeaTrac."""
    crc = 0x0000

    for byte in data:
        value = byte

        for _ in range(8):
            if (value & 0x01) ^ (crc & 0x01):
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

            value >>= 1

    return crc


def build_seatrac_command(message_data: bytes) -> bytes:
    """Construye una trama ASCII #... terminada en CR+LF."""
    crc = crc16_ibm(message_data)
    binary_frame = message_data + struct.pack("<H", crc)

    return (
        b"#"
        + binary_frame.hex().upper().encode("ascii")
        + b"\r\n"
    )


class SeaTracSerialNode(Node):

    def __init__(self) -> None:
        super().__init__("seatrac_serial_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("salinity_ppt", 0.0)
        self.declare_parameter("auto_vos", True)
        self.declare_parameter("auto_pressure_offset", True)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.raw_publisher = self.create_publisher(
            String,
            "seatrac/system_info_raw",
            qos,
        )

        self.status_raw_publisher = self.create_publisher(
            String,
            "seatrac/status_raw",
            10,
        )

        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.rx_buffer = bytearray()
        self.system_info_received = False

        self.read_timer = self.create_timer(0.02, self.read_serial)
        self.request_timer = self.create_timer(1.0, self.request_system_info)
        self.timeout_timer = self.create_timer(4.0, self.check_response)
        self.status_timer = self.create_timer(1.0, self.request_status)

        self.mag_reset_service = self.create_service(
            Trigger,
            "seatrac/calibration/reset_magnetometer",
            self.reset_magnetometer_callback,
        )

        self.mag_calculate_service = self.create_service(
            Trigger,
            "seatrac/calibration/calculate_magnetometer",
            self.calculate_magnetometer_callback,
        )

        self.settings_save_service = self.create_service(
            Trigger,
            "seatrac/settings/save",
            self.save_settings_callback,
        )

        self.environment_apply_service = self.create_service(
            Trigger,
            "seatrac/environment/apply",
            self.apply_environment_callback,
        )

        self.get_logger().info(
            f"Puerto abierto: {self.port}, "
            f"{self.baudrate} baud, 8N2"
        )

    def request_system_info(self) -> None:
        """Solicita una vez la información del SeaTrac."""
        self.serial_port.reset_input_buffer()
        self.serial_port.write(SYSTEM_INFO_COMMAND)
        self.serial_port.flush()

        self.get_logger().info(
            f"Comando enviado: {SYSTEM_INFO_COMMAND!r}"
        )

        self.request_timer.cancel()

    def reset_magnetometer_callback(
        self,
        request,
        response,
    ):
        """Borra el búfer de calibración magnética."""
        del request

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(MAG_CAL_RESET_COMMAND)
            self.serial_port.flush()

            self.get_logger().info(
                "Solicitando reinicio del búfer magnético..."
            )

            deadline = time.monotonic() + 2.0
            rx_buffer = bytearray()

            while time.monotonic() < deadline:
                available = self.serial_port.in_waiting

                if available > 0:
                    rx_buffer.extend(
                        self.serial_port.read(available)
                    )

                while b"\r\n" in rx_buffer:
                    raw_line, _, remaining = rx_buffer.partition(
                        b"\r\n"
                    )
                    rx_buffer = bytearray(remaining)

                    text = raw_line.decode(
                        "ascii",
                        errors="replace",
                    )

                    if not text.startswith("$20"):
                        continue

                    if text.startswith("$2000"):
                        response.success = True
                        response.message = (
                            "Búfer magnético reiniciado "
                            "correctamente."
                        )
                        self.get_logger().info(
                            response.message
                        )
                    else:
                        response.success = False
                        response.message = (
                            "El SeaTrac rechazó el comando: "
                            f"{text}"
                        )
                        self.get_logger().error(
                            response.message
                        )

                    return response

                time.sleep(0.01)

            response.success = False
            response.message = (
                "No llegó una respuesta $20 del SeaTrac."
            )
            self.get_logger().error(response.message)
            return response

        except serial.SerialException as error:
            response.success = False
            response.message = f"Error serial: {error}"
            self.get_logger().error(response.message)
            return response

    def calculate_magnetometer_callback(
        self,
        request,
        response,
    ):
        """Calcula y aplica la calibración del magnetómetro."""
        del request

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(MAG_CAL_CALCULATE_COMMAND)
            self.serial_port.flush()

            self.get_logger().info(
                "Calculando calibración magnética..."
            )

            deadline = time.monotonic() + 5.0
            rx_buffer = bytearray()

            while time.monotonic() < deadline:
                available = self.serial_port.in_waiting

                if available > 0:
                    rx_buffer.extend(
                        self.serial_port.read(available)
                    )

                while b"\r\n" in rx_buffer:
                    raw_line, _, remaining = rx_buffer.partition(
                        b"\r\n"
                    )
                    rx_buffer = bytearray(remaining)

                    text = raw_line.decode(
                        "ascii",
                        errors="replace",
                    )

                    if not text.startswith("$20"):
                        continue

                    if text.startswith("$2000"):
                        response.success = True
                        response.message = (
                            "Calibración magnética calculada "
                            "y aplicada en memoria de trabajo."
                        )
                        self.get_logger().info(response.message)
                    else:
                        response.success = False
                        response.message = (
                            "El SeaTrac rechazó el cálculo: "
                            f"{text}"
                        )
                        self.get_logger().error(response.message)

                    return response

                time.sleep(0.01)

            response.success = False
            response.message = (
                "No llegó una respuesta $20 del SeaTrac."
            )
            self.get_logger().error(response.message)
            return response

        except serial.SerialException as error:
            response.success = False
            response.message = f"Error serial: {error}"
            self.get_logger().error(response.message)
            return response

    def save_settings_callback(
        self,
        request,
        response,
    ):
        """Guarda las configuraciones actuales en la EEPROM."""
        del request

        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(SETTINGS_SAVE_COMMAND)
            self.serial_port.flush()

            self.get_logger().info(
                "Guardando configuración en memoria permanente..."
            )

            deadline = time.monotonic() + 5.0
            rx_buffer = bytearray()

            while time.monotonic() < deadline:
                available = self.serial_port.in_waiting

                if available > 0:
                    rx_buffer.extend(
                        self.serial_port.read(available)
                    )

                while b"\r\n" in rx_buffer:
                    raw_line, _, remaining = rx_buffer.partition(
                        b"\r\n"
                    )
                    rx_buffer = bytearray(remaining)

                    text = raw_line.decode(
                        "ascii",
                        errors="replace",
                    )

                    if not text.startswith("$18"):
                        continue

                    # $18 + estado 00 significa CST_OK.
                    if text.startswith("$1800"):
                        response.success = True
                        response.message = (
                            "Configuración guardada permanentemente."
                        )
                        self.get_logger().info(response.message)
                    else:
                        response.success = False
                        response.message = (
                            "El SeaTrac rechazó el guardado: "
                            f"{text}"
                        )
                        self.get_logger().error(response.message)

                    return response

                time.sleep(0.01)

            response.success = False
            response.message = (
                "No llegó una respuesta $18 del SeaTrac."
            )
            self.get_logger().error(response.message)
            return response

        except serial.SerialException as error:
            response.success = False
            response.message = f"Error serial: {error}"
            self.get_logger().error(response.message)
            return response

    def apply_environment_callback(
        self,
        request,
        response,
    ):
        """Aplica salinidad y opciones automáticas en RAM."""
        del request

        salinity_ppt = float(
            self.get_parameter("salinity_ppt").value
        )
        auto_vos = bool(
            self.get_parameter("auto_vos").value
        )
        auto_pressure_offset = bool(
            self.get_parameter(
                "auto_pressure_offset"
            ).value
        )

        if not 0.0 <= salinity_ppt <= 35.0:
            response.success = False
            response.message = (
                "salinity_ppt debe estar entre 0.0 y 35.0."
            )
            return response

        def read_response(
            expected_command_id: int,
            timeout_seconds: float,
        ):
            deadline = time.monotonic() + timeout_seconds
            local_buffer = bytearray()

            while time.monotonic() < deadline:
                available = self.serial_port.in_waiting

                if available > 0:
                    local_buffer.extend(
                        self.serial_port.read(available)
                    )

                while b"\r\n" in local_buffer:
                    raw_line, _, remaining = (
                        local_buffer.partition(b"\r\n")
                    )
                    local_buffer = bytearray(remaining)

                    try:
                        line = raw_line.decode("ascii")
                    except UnicodeDecodeError:
                        continue

                    if not line.startswith("$"):
                        continue

                    try:
                        frame = bytes.fromhex(line[1:])
                    except ValueError:
                        continue

                    if len(frame) < 4:
                        continue

                    message_data = frame[:-2]
                    received_crc = struct.unpack(
                        "<H",
                        frame[-2:],
                    )[0]

                    if crc16_ibm(message_data) != received_crc:
                        continue

                    if message_data[0] == expected_command_id:
                        return message_data

                time.sleep(0.01)

            return None

        try:
            # Evitar mezclar respuestas de estado anteriores.
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.rx_buffer.clear()

            # 1. Leer el registro completo actual.
            get_command = build_seatrac_command(
                bytes([0x15])
            )

            self.serial_port.write(get_command)
            self.serial_port.flush()

            settings_response = read_response(
                expected_command_id=0x15,
                timeout_seconds=3.0,
            )

            if settings_response is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta válida $15."
                )
                return response

            # El primer byte es CID_SETTINGS_GET.
            settings = bytearray(settings_response[1:])

            # Offsets definidos por SETTINGS_T:
            # 28     ENV_FLAGS
            # 29:33  ENV_PRESSURE_OFS
            # 33:35  ENV_SALINITY
            # 35:37  ENV_VOS
            if len(settings) < 37:
                response.success = False
                response.message = (
                    "El registro SETTINGS_T es demasiado corto: "
                    f"{len(settings)} bytes."
                )
                return response

            old_salinity_raw = struct.unpack_from(
                "<H",
                settings,
                33,
            )[0]
            old_salinity_ppt = old_salinity_raw / 10.0

            # AUTO_VOS: bit 0 de ENV_FLAGS.
            if auto_vos:
                settings[28] |= 0x01
            else:
                settings[28] &= 0xFE

            # AUTO_PRESSURE_OFS: bit 1 de ENV_FLAGS.
            if auto_pressure_offset:
                settings[28] |= 0x02
            else:
                settings[28] &= 0xFD

            new_salinity_raw = int(
                round(salinity_ppt * 10.0)
            )

            struct.pack_into(
                "<H",
                settings,
                33,
                new_salinity_raw,
            )

            # 2. Reenviar todo el registro con CID_SETTINGS_SET.
            set_message = bytes([0x16]) + bytes(settings)
            set_command = build_seatrac_command(set_message)

            self.serial_port.reset_input_buffer()
            self.serial_port.write(set_command)
            self.serial_port.flush()

            set_response = read_response(
                expected_command_id=0x16,
                timeout_seconds=4.0,
            )

            if set_response is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta válida $16."
                )
                return response

            if len(set_response) < 2:
                response.success = False
                response.message = (
                    "Respuesta $16 incompleta."
                )
                return response

            status_code = set_response[1]

            if status_code != 0x00:
                response.success = False
                response.message = (
                    "SeaTrac rechazó SETTINGS_SET; "
                    f"estado=0x{status_code:02X}."
                )
                return response

            response.success = True
            response.message = (
                f"Ambiente aplicado en RAM: "
                f"salinidad {old_salinity_ppt:.1f} → "
                f"{salinity_ppt:.1f} ppt, "
                f"AUTO_VOS={auto_vos}, "
                f"AUTO_PRESSURE_OFS={auto_pressure_offset}. "
                "Todavía no se guardó en EEPROM."
            )

            self.get_logger().info(response.message)
            return response

        except serial.SerialException as error:
            response.success = False
            response.message = f"Error serial: {error}"
            self.get_logger().error(response.message)
            return response

    def request_status(self) -> None:
        """Solicita el estado del X150 una vez por segundo."""
        if not self.system_info_received:
            return

        try:
            self.serial_port.write(STATUS_COMMAND)
            self.serial_port.flush()

        except serial.SerialException as error:
            self.get_logger().error(
                f"Error enviando la consulta de estado: {error}"
            )

    def read_serial(self) -> None:
        """Lee y separa mensajes terminados en CR+LF."""
        try:
            available = self.serial_port.in_waiting

            if available > 0:
                self.rx_buffer.extend(
                    self.serial_port.read(available)
                )

            while b"\r\n" in self.rx_buffer:
                raw_line, _, remaining = self.rx_buffer.partition(b"\r\n")
                self.rx_buffer = bytearray(remaining)

                if not raw_line:
                    continue

                text = raw_line.decode("ascii", errors="replace")

                message = String()
                message.data = text

                if text.startswith("$02"):
                    self.raw_publisher.publish(message)
                    self.system_info_received = True
                    self.get_logger().info(
                        "Comunicación ROS 2 ↔ SeaTrac confirmada."
                    )

                elif text.startswith("$10"):
                    self.status_raw_publisher.publish(message)
                    self.get_logger().info(
                        "Estado SeaTrac recibido."
                    )

                else:
                    self.get_logger().info(
                        f"RX sin clasificar: {text}"
                    )

        except serial.SerialException as error:
            self.get_logger().error(f"Error leyendo el puerto: {error}")

    def check_response(self) -> None:
        if not self.system_info_received:
            self.get_logger().warning(
                "El puerto abrió, pero no llegó una respuesta $02."
            )

        self.timeout_timer.cancel()

    def destroy_node(self) -> None:
        if self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Puerto serial cerrado.")

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = SeaTracSerialNode()
        rclpy.spin(node)

    except serial.SerialException as error:
        print(f"No se pudo abrir el puerto serial: {error}")

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
