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

from seatrac_ros2.seatrac_protocol import (
    SeaTracFrameError,
    SeaTracStreamFramer,
    crc16_ibm,
    encode_capture_record,
    parse_ascii_frame,
)


SYSTEM_INFO_COMMAND = b"#0281C1\r\n"
STATUS_COMMAND = b"#10000DC0\r\n"
MAG_CAL_RESET_COMMAND = b"#20041803\r\n"
MAG_CAL_CALCULATE_COMMAND = b"#2005D9C3\r\n"
SETTINGS_SAVE_COMMAND = b"#18000A\r\n"


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
        self.declare_parameter("max_rx_buffer_bytes", 65536)
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

        self.max_rx_buffer_bytes = max(
            16,
            int(self.get_parameter("max_rx_buffer_bytes").value),
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

        self.frame_raw_publisher = self.create_publisher(
            String,
            "seatrac/frame_raw",
            10,
        )

        capture_qos = QoSProfile(depth=100)
        capture_qos.reliability = ReliabilityPolicy.RELIABLE
        capture_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.frame_capture_publisher = self.create_publisher(
            String,
            "seatrac/frame_capture",
            capture_qos,
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

        self.stream_framer = SeaTracStreamFramer(
            self.max_rx_buffer_bytes
        )
        self.capture_sequence = 0
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

        self.environment_read_service = self.create_service(
            Trigger,
            "seatrac/environment/read",
            self.read_environment_callback,
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
        self.stream_framer.clear()

        while True:
            try:
                self.udp_socket.recv(4096)
            except BlockingIOError:
                break

    def send_command(self, command: bytes) -> None:
        """Envía una trama SeaTrac al Serial Bridge de BlueOS."""
        self.udp_socket.send(command)

    def publish_capture(
        self,
        raw_frame: bytes,
        received_unix_ns: int,
        received_monotonic_ns: int,
        record_kind: str = 'frame',
    ) -> None:
        """Publish one byte-exact frame record before semantic routing."""
        capture = encode_capture_record(
            sequence=self.capture_sequence,
            received_unix_ns=received_unix_ns,
            received_monotonic_ns=received_monotonic_ns,
            transport='blueos_udp',
            raw_bytes=raw_frame,
            record_kind=record_kind,
        )
        self.capture_sequence += 1
        self.frame_capture_publisher.publish(String(data=capture))

    def read_response(
        self,
        expected_command_id: int,
        timeout_seconds: float,
    ):
        """Espera una respuesta SeaTrac válida para un Command ID."""
        deadline = time.monotonic() + timeout_seconds
        local_framer = SeaTracStreamFramer(self.max_rx_buffer_bytes)

        while time.monotonic() < deadline:
            completed_frames = []

            while True:
                try:
                    data = self.udp_socket.recv(4096)

                    if data:
                        result = local_framer.feed(data)
                        completed_frames.extend(result.frames)
                        if result.discarded_bytes:
                            self.publish_capture(
                                result.discarded_data,
                                time.time_ns(),
                                time.monotonic_ns(),
                                record_kind='unterminated_overflow',
                            )
                            self.get_logger().error(
                                "SeaTrac RX transaccional descartó "
                                f"{result.discarded_bytes} bytes sin CRLF."
                            )

                except BlockingIOError:
                    break

            for complete_frame in completed_frames:
                self.publish_capture(
                    complete_frame,
                    time.time_ns(),
                    time.monotonic_ns(),
                )

                try:
                    parsed = parse_ascii_frame(complete_frame)
                except SeaTracFrameError:
                    continue

                line = parsed.raw_text
                self.frame_raw_publisher.publish(String(data=line))

                if parsed.command_id == expected_command_id:
                    return parsed.message_data

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

    def read_environment_callback(
        self,
        request,
        response,
    ):
        """Lee la configuración ambiental actual sin modificarla."""
        del request

        try:
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

            settings = bytes(
                settings_response[1:]
            )

            if len(settings) < 37:
                response.success = False
                response.message = (
                    "El registro SETTINGS_T es demasiado corto: "
                    f"{len(settings)} bytes."
                )
                return response

            env_flags = settings[28]

            auto_vos = bool(
                env_flags & 0x01
            )

            auto_pressure_offset = bool(
                env_flags & 0x02
            )

            salinity_raw = struct.unpack_from(
                "<H",
                settings,
                33,
            )[0]

            salinity_ppt = (
                salinity_raw / 10.0
            )

            response.success = True
            response.message = (
                "Configuración ambiental actual del X150: "
                f"salinity_ppt={salinity_ppt:.1f}, "
                f"AUTO_VOS={auto_vos}, "
                "AUTO_PRESSURE_OFS="
                f"{auto_pressure_offset}, "
                f"ENV_FLAGS=0x{env_flags:02X}. "
                "Solo lectura; no se modificó "
                "RAM ni EEPROM."
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
                        received_unix_ns = time.time_ns()
                        received_monotonic_ns = time.monotonic_ns()
                        result = self.stream_framer.feed(data)
                        if result.discarded_bytes:
                            self.publish_capture(
                                result.discarded_data,
                                received_unix_ns,
                                received_monotonic_ns,
                                record_kind='unterminated_overflow',
                            )
                            self.get_logger().error(
                                "SeaTrac RX descartó "
                                f"{result.discarded_bytes} bytes sin CRLF "
                                "al alcanzar el límite del buffer."
                            )
                        for raw_frame in result.frames:
                            self.handle_received_frame(
                                raw_frame,
                                received_unix_ns,
                                received_monotonic_ns,
                            )

                except BlockingIOError:
                    break

        except OSError as error:
            self.get_logger().error(
                f"Error recibiendo UDP: {error}"
            )

    def handle_received_frame(
        self,
        raw_frame: bytes,
        received_unix_ns: int,
        received_monotonic_ns: int,
    ) -> None:
        """Capture, validate and only then route one complete frame."""
        self.publish_capture(
            raw_frame,
            received_unix_ns,
            received_monotonic_ns,
        )

        try:
            frame = parse_ascii_frame(raw_frame)
        except SeaTracFrameError as error:
            self.get_logger().warning(
                f"Trama SeaTrac inválida preservada pero descartada: {error}"
            )
            return

        message = String(data=frame.raw_text)
        self.frame_raw_publisher.publish(message)

        if frame.command_id == 0x02:
            self.raw_publisher.publish(message)

            if not self.system_info_received:
                self.get_logger().info(
                    "Comunicación ROS 2 ↔ BlueOS ↔ SeaTrac confirmada "
                    "con CRC válido."
                )

            self.system_info_received = True

        elif frame.command_id == 0x10:
            self.status_raw_publisher.publish(message)

        else:
            self.get_logger().info(
                f"RX válido sin clasificar: CID 0x{frame.command_id:02X}"
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
