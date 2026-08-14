#!/usr/bin/env python3

import json
import struct
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def crc16_ibm(data: bytes) -> int:
    """Calcula el CRC-16-IBM usado por SeaTrac."""
    crc = 0x0000
    polynomial = 0xA001

    for byte in data:
        value = byte

        for _ in range(8):
            if (value & 0x01) ^ (crc & 0x01):
                crc >>= 1
                crc ^= polynomial
            else:
                crc >>= 1

            value >>= 1

    return crc


class BinaryReader:
    """Lector secuencial little-endian."""

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.offset = 0

    def read(self, format_code: str):
        format_string = "<" + format_code
        size = struct.calcsize(format_string)

        if self.offset + size > len(self.data):
            raise ValueError(
                f"Trama incompleta: se requieren {size} bytes en "
                f"la posición {self.offset}."
            )

        values = struct.unpack_from(
            format_string,
            self.data,
            self.offset,
        )
        self.offset += size

        return values[0] if len(values) == 1 else values

    @property
    def remaining(self) -> int:
        return len(self.data) - self.offset


def decode_status_frame(text: str) -> Dict[str, Any]:
    """Decodifica una respuesta SeaTrac CID_STATUS ($10)."""
    text = text.strip()

    if not text.startswith("$"):
        raise ValueError("La trama no empieza con '$'.")

    hexadecimal = text[1:]

    if len(hexadecimal) % 2 != 0:
        raise ValueError("La cantidad de caracteres hexadecimales es impar.")

    try:
        complete_frame = bytes.fromhex(hexadecimal)
    except ValueError as error:
        raise ValueError("La trama contiene caracteres no hexadecimales.") from error

    if len(complete_frame) < 12:
        raise ValueError("La trama es demasiado corta.")

    message_data = complete_frame[:-2]
    received_crc = struct.unpack("<H", complete_frame[-2:])[0]
    calculated_crc = crc16_ibm(message_data)

    if received_crc != calculated_crc:
        raise ValueError(
            f"CRC incorrecto: recibido 0x{received_crc:04X}, "
            f"calculado 0x{calculated_crc:04X}."
        )

    reader = BinaryReader(message_data)

    command_id = reader.read("B")

    if command_id != 0x10:
        raise ValueError(
            f"Se esperaba CID_STATUS 0x10, pero llegó 0x{command_id:02X}."
        )

    flags = reader.read("B")
    timestamp_ms = reader.read("Q")

    status: Dict[str, Any] = {
        "crc_valid": True,
        "command_id": command_id,
        "output_flags": f"0x{flags:02X}",
        "timestamp_ms": timestamp_ms,
        "uptime_seconds": timestamp_ms / 1000.0,
    }

    # Bit 0: datos ambientales
    if flags & 0x01:
        supply_mv = reader.read("H")
        temperature_deci_c = reader.read("h")
        pressure_mbar = reader.read("i")
        depth_deci_m = reader.read("i")
        sound_speed_deci_ms = reader.read("H")

        status.update({
            "supply_voltage_v": supply_mv / 1000.0,
            "temperature_c": temperature_deci_c / 10.0,
            "pressure_bar": pressure_mbar / 1000.0,
            "depth_m": depth_deci_m / 10.0,
            "sound_speed_m_s": sound_speed_deci_ms / 10.0,
        })

    # Bit 1: orientación
    if flags & 0x02:
        yaw, pitch, roll = reader.read("hhh")

        status.update({
            "yaw_deg": yaw / 10.0,
            "pitch_deg": pitch / 10.0,
            "roll_deg": roll / 10.0,
        })

    # Bit 2: calibración del magnetómetro
    if flags & 0x04:
        calibration_buffer = reader.read("B")
        calibration_valid = reader.read("B")
        calibration_age = reader.read("I")
        calibration_fit = reader.read("B")

        status.update({
            "mag_calibration_buffer_percent": calibration_buffer,
            "mag_calibration_valid": calibration_valid != 0,
            "mag_calibration_age_s": calibration_age,
            "mag_calibration_fit_percent": calibration_fit,
        })

    # Bit 3: límites de calibración del acelerómetro
    if flags & 0x08:
        (
            minimum_x,
            minimum_y,
            minimum_z,
            maximum_x,
            maximum_y,
            maximum_z,
        ) = reader.read("hhhhhh")

        status["accelerometer_limits"] = {
            "minimum_x": minimum_x,
            "minimum_y": minimum_y,
            "minimum_z": minimum_z,
            "maximum_x": maximum_x,
            "maximum_y": maximum_y,
            "maximum_z": maximum_z,
        }

    # Bit 4: lecturas AHRS sin compensar
    if flags & 0x10:
        (
            acceleration_x,
            acceleration_y,
            acceleration_z,
            magnetic_x,
            magnetic_y,
            magnetic_z,
            gyroscope_x,
            gyroscope_y,
            gyroscope_z,
        ) = reader.read("hhhhhhhhh")

        status["ahrs_raw"] = {
            "acceleration_x": acceleration_x,
            "acceleration_y": acceleration_y,
            "acceleration_z": acceleration_z,
            "magnetic_x": magnetic_x,
            "magnetic_y": magnetic_y,
            "magnetic_z": magnetic_z,
            "gyroscope_x": gyroscope_x,
            "gyroscope_y": gyroscope_y,
            "gyroscope_z": gyroscope_z,
        }

    if reader.remaining != 0:
        status["unparsed_bytes"] = reader.remaining

    return status


class SeaTracStatusNode(Node):

    def __init__(self) -> None:
        super().__init__("seatrac_status_node")

        self.status_publisher = self.create_publisher(
            String,
            "/seatrac/status",
            10,
        )

        self.raw_subscription = self.create_subscription(
            String,
            "/seatrac/status_raw",
            self.status_callback,
            10,
        )

        self.first_valid_message = True

        self.get_logger().info(
            "Esperando tramas en /seatrac/status_raw..."
        )

    def status_callback(self, message: String) -> None:
        try:
            decoded = decode_status_frame(message.data)

        except ValueError as error:
            self.get_logger().warning(
                f"Trama SeaTrac descartada: {error}"
            )
            return

        output = String()
        output.data = json.dumps(
            decoded,
            ensure_ascii=False,
            sort_keys=True,
        )

        self.status_publisher.publish(output)

        if self.first_valid_message:
            self.get_logger().info(
                "Primera trama $10 decodificada y con CRC válido."
            )
            self.first_valid_message = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeaTracStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
