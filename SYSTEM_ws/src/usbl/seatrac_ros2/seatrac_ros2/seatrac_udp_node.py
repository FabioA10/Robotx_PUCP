#!/usr/bin/env python3

import socket
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


class SeaTracUdpNode(Node):

    def __init__(self) -> None:
        super().__init__("seatrac_udp_node")

        # --------------------------------------------------------------
        # Communication
        # --------------------------------------------------------------

        self.declare_parameter("blueos_ip", "192.168.2.3")
        self.declare_parameter("blueos_port", 15000)
        self.declare_parameter("response_timeout_s", 3.0)
        self.declare_parameter("status_rate_hz", 1.0)
        self.declare_parameter(
            "allow_persistent_save",
            False,
        )

        # --------------------------------------------------------------
        # Environment
        # --------------------------------------------------------------

        self.declare_parameter("salinity_ppt", 0.0)
        self.declare_parameter("auto_vos", True)
        self.declare_parameter("auto_pressure_offset", True)

        # Reserved for the future X150 <-> X110 positioning layer.
        self.declare_parameter(
            "acoustic_positioning_enabled",
            False,
        )

        self.blueos_ip = str(
            self.get_parameter("blueos_ip").value
        )

        self.blueos_port = int(
            self.get_parameter("blueos_port").value
        )

        self.response_timeout_s = float(
            self.get_parameter("response_timeout_s").value
        )

        self.status_rate_hz = float(
            self.get_parameter("status_rate_hz").value
        )

        if self.status_rate_hz <= 0.0:
            self.status_rate_hz = 1.0

        # --------------------------------------------------------------
        # ROS publishers
        # --------------------------------------------------------------

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

        # --------------------------------------------------------------
        # UDP transport
        # --------------------------------------------------------------

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )

        self.udp_socket.setblocking(False)

        self.udp_socket.connect(
            (
                self.blueos_ip,
                self.blueos_port,
            )
        )

        self.rx_buffer = bytearray()
        self.system_info_received = False

        # --------------------------------------------------------------
        # Timers
        # --------------------------------------------------------------

        self.read_timer = self.create_timer(
            0.02,
            self.read_udp,
        )

        self.request_timer = self.create_timer(
            1.0,
            self.request_system_info,
        )

        self.timeout_timer = self.create_timer(
            max(
                4.0,
                self.response_timeout_s + 1.0,
            ),
            self.check_response,
        )

        self.status_timer = self.create_timer(
            1.0 / self.status_rate_hz,
            self.request_status,
        )

        # --------------------------------------------------------------
        # Services
        # --------------------------------------------------------------

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
            "SeaTrac X150 mediante BlueOS UDP: "
            f"{self.blueos_ip}:{self.blueos_port}"
        )

    # ==================================================================
    # UDP transport helpers
    # ==================================================================

    def drain_udp_input(self) -> None:
        """Descarta datos UDP pendientes antes de una transacción."""
        self.rx_buffer.clear()

        while True:
            try:
                self.udp_socket.recv(4096)
            except BlockingIOError:
                break

    def send_command(self, command: bytes) -> None:
        """Envía una trama SeaTrac al Serial Bridge de BlueOS."""
        self.udp_socket.send(command)

    def read_response(
        self,
        expected_command_id: int,
        timeout_seconds: float,
    ):
        """Espera una respuesta SeaTrac válida para un Command ID."""
        deadline = time.monotonic() + timeout_seconds
        local_buffer = bytearray()

        while time.monotonic() < deadline:

            while True:
                try:
                    data = self.udp_socket.recv(4096)

                    if data:
                        local_buffer.extend(data)

                except BlockingIOError:
                    break

            while b"\r\n" in local_buffer:

                raw_line, _, remaining = local_buffer.partition(
                    b"\r\n"
                )

                local_buffer = bytearray(remaining)

                if not raw_line:
                    continue

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

                if len(frame) < 3:
                    continue

                message_data = frame[:-2]

                received_crc = struct.unpack(
                    "<H",
                    frame[-2:],
                )[0]

                if crc16_ibm(message_data) != received_crc:
                    continue

                if not message_data:
                    continue

                if message_data[0] == expected_command_id:
                    return message_data

            time.sleep(0.01)

        return None

    # ==================================================================
    # SYSTEM INFO
    # ==================================================================

    def request_system_info(self) -> None:
        """Solicita una vez la información del SeaTrac."""
        try:
            self.drain_udp_input()
            self.send_command(SYSTEM_INFO_COMMAND)

            self.get_logger().info(
                f"Comando enviado: {SYSTEM_INFO_COMMAND!r}"
            )

            self.request_timer.cancel()

        except OSError as error:
            self.get_logger().error(
                f"Error UDP solicitando SYSTEM_INFO: {error}"
            )

    # ==================================================================
    # Magnetometer calibration
    # ==================================================================

    def reset_magnetometer_callback(
        self,
        request,
        response,
    ):
        """Borra el búfer de calibración magnética."""
        del request

        try:
            self.drain_udp_input()
            self.send_command(MAG_CAL_RESET_COMMAND)

            self.get_logger().info(
                "Solicitando reinicio del búfer magnético..."
            )

            result = self.read_response(
                expected_command_id=0x20,
                timeout_seconds=2.0,
            )

            if result is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta $20 del SeaTrac."
                )

            elif len(result) >= 2 and result[1] == 0x00:
                response.success = True
                response.message = (
                    "Búfer magnético reiniciado correctamente."
                )

            else:
                response.success = False
                response.message = (
                    "El SeaTrac rechazó el comando "
                    "de reinicio magnético."
                )

        except OSError as error:
            response.success = False
            response.message = f"Error UDP: {error}"

        if response.success:
            self.get_logger().info(response.message)
        else:
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
            self.drain_udp_input()
            self.send_command(
                MAG_CAL_CALCULATE_COMMAND
            )

            self.get_logger().info(
                "Calculando calibración magnética..."
            )

            result = self.read_response(
                expected_command_id=0x20,
                timeout_seconds=5.0,
            )

            if result is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta $20 del SeaTrac."
                )

            elif len(result) >= 2 and result[1] == 0x00:
                response.success = True
                response.message = (
                    "Calibración magnética calculada "
                    "y aplicada en memoria de trabajo."
                )

            else:
                response.success = False
                response.message = (
                    "El SeaTrac rechazó el cálculo "
                    "de calibración."
                )

        except OSError as error:
            response.success = False
            response.message = f"Error UDP: {error}"

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

        return response

    # ==================================================================
    # SETTINGS SAVE
    # ==================================================================

    def save_settings_callback(
        self,
        request,
        response,
    ):
        """Guarda las configuraciones actuales en la EEPROM."""
        del request

        allow_persistent_save = bool(
            self.get_parameter(
                "allow_persistent_save"
            ).value
        )

        if not allow_persistent_save:
            response.success = False
            response.message = (
                "Guardado permanente BLOQUEADO. "
                "allow_persistent_save=false. "
                "No se escribió EEPROM."
            )

            self.get_logger().warning(
                response.message
            )

            return response

        try:
            self.drain_udp_input()
            self.send_command(SETTINGS_SAVE_COMMAND)

            self.get_logger().info(
                "Guardando configuración en memoria permanente..."
            )

            result = self.read_response(
                expected_command_id=0x18,
                timeout_seconds=5.0,
            )

            if result is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta $18 del SeaTrac."
                )

            elif len(result) >= 2 and result[1] == 0x00:
                response.success = True
                response.message = (
                    "Configuración guardada permanentemente."
                )

            else:
                response.success = False
                response.message = (
                    "El SeaTrac rechazó el guardado."
                )

        except OSError as error:
            response.success = False
            response.message = f"Error UDP: {error}"

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

        return response

    # ==================================================================
    # Environment
    # ==================================================================

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

        try:
            # ----------------------------------------------------------
            # 1. Leer SETTINGS actuales.
            # ----------------------------------------------------------

            self.drain_udp_input()

            get_command = build_seatrac_command(
                bytes([0x15])
            )

            self.send_command(get_command)

            settings_response = self.read_response(
                expected_command_id=0x15,
                timeout_seconds=self.response_timeout_s,
            )

            if settings_response is None:
                response.success = False
                response.message = (
                    "No llegó una respuesta válida $15."
                )
                return response

            # Primer byte: CID_SETTINGS_GET.
            settings = bytearray(
                settings_response[1:]
            )

            # Offsets definidos por SETTINGS_T:
            #
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

            old_salinity_ppt = (
                old_salinity_raw / 10.0
            )

            # AUTO_VOS: bit 0.
            if auto_vos:
                settings[28] |= 0x01
            else:
                settings[28] &= 0xFE

            # AUTO_PRESSURE_OFS: bit 1.
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

            # ----------------------------------------------------------
            # 2. Reenviar SETTINGS completos mediante CID_SETTINGS_SET.
            # ----------------------------------------------------------

            set_message = (
                bytes([0x16])
                + bytes(settings)
            )

            set_command = build_seatrac_command(
                set_message
            )

            self.drain_udp_input()
            self.send_command(set_command)

            set_response = self.read_response(
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
                "Ambiente aplicado en RAM: "
                f"salinidad {old_salinity_ppt:.1f} → "
                f"{salinity_ppt:.1f} ppt, "
                f"AUTO_VOS={auto_vos}, "
                "AUTO_PRESSURE_OFS="
                f"{auto_pressure_offset}. "
                "Todavía no se guardó en EEPROM."
            )

            self.get_logger().info(
                response.message
            )

            return response

        except OSError as error:
            response.success = False
            response.message = (
                f"Error UDP: {error}"
            )

            self.get_logger().error(
                response.message
            )

            return response

    # ==================================================================
    # STATUS
    # ==================================================================

    def request_status(self) -> None:
        """Solicita periódicamente el estado del X150."""
        if not self.system_info_received:
            return

        try:
            self.send_command(STATUS_COMMAND)

        except OSError as error:
            self.get_logger().error(
                "Error enviando la consulta "
                f"de estado por UDP: {error}"
            )

    # ==================================================================
    # Continuous UDP receiver
    # ==================================================================

    def read_udp(self) -> None:
        """Lee UDP y separa tramas SeaTrac terminadas en CR+LF."""
        try:
            while True:
                try:
                    data = self.udp_socket.recv(4096)

                    if data:
                        self.rx_buffer.extend(data)

                except BlockingIOError:
                    break

            while b"\r\n" in self.rx_buffer:

                raw_line, _, remaining = (
                    self.rx_buffer.partition(
                        b"\r\n"
                    )
                )

                self.rx_buffer = bytearray(
                    remaining
                )

                if not raw_line:
                    continue

                text = raw_line.decode(
                    "ascii",
                    errors="replace",
                )

                message = String()
                message.data = text

                if text.startswith("$02"):
                    self.raw_publisher.publish(
                        message
                    )

                    if not self.system_info_received:
                        self.get_logger().info(
                            "Comunicación ROS 2 ↔ "
                            "BlueOS ↔ SeaTrac confirmada."
                        )

                    self.system_info_received = True

                elif text.startswith("$10"):
                    self.status_raw_publisher.publish(
                        message
                    )

                else:
                    self.get_logger().info(
                        f"RX sin clasificar: {text}"
                    )

        except OSError as error:
            self.get_logger().error(
                f"Error recibiendo UDP: {error}"
            )

    def check_response(self) -> None:
        """Comprueba que el X150 respondió a SYSTEM_INFO."""
        if not self.system_info_received:
            self.get_logger().warning(
                "BlueOS UDP está configurado, "
                "pero no llegó una respuesta $02."
            )

        self.timeout_timer.cancel()

    def destroy_node(self) -> None:
        """Cierra el socket UDP al detener ROS."""
        try:
            self.udp_socket.close()
        except OSError:
            pass

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = SeaTracUdpNode()
        rclpy.spin(node)

    except OSError as error:
        print(
            "No se pudo inicializar la conexión "
            f"UDP SeaTrac: {error}"
        )

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
