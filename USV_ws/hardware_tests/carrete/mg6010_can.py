from __future__ import annotations
import logging
import struct
import time
from dataclasses import dataclass
from typing import Optional
import can

logger = logging.getLogger("mg6010")

class MG6010Error(Exception):
    """Raised for communication failures or protocol-level errors."""

@dataclass
class MotorStatus:
    temperature_c: int
    torque_current_raw: int
    speed_dps: int
    encoder: int

    @property
    def torque_current_a(self) -> float:
        return self.torque_current_raw * 33.0 / 2048.0

@dataclass
class MotorState1:
    temperature_c: int
    voltage_v: float
    error_state: int

    @property
    def under_voltage(self) -> bool:
        return bool(self.error_state & (1 << 0))

    @property
    def over_temperature(self) -> bool:
        return bool(self.error_state & (1 << 3))

    @property
    def has_fault(self) -> bool:
        return self.under_voltage or self.over_temperature

class MG6010:
    CMD_MOTOR_OFF = 0x80
    CMD_MOTOR_ON = 0x88
    CMD_MOTOR_STOP = 0x81
    CMD_MULTI_ANGLE_2 = 0xA4
    CMD_READ_STATE2 = 0x9C
    CMD_CLEAR_ERROR = 0x9B

    def __init__(self, motor_id: int, channel: str = "can0", bustype: str = "socketcan", response_timeout: float = 0.5, reduction: int = 1):
        self.motor_id = motor_id
        self.arb_id = 0x140 + motor_id
        self.channel = channel
        self.bustype = bustype
        self.response_timeout = response_timeout
        self.bus: Optional[can.BusABC] = None
        self.reduction = reduction

    def connect(self) -> None:
        self.bus = can.Bus(channel=self.channel, bustype=self.bustype)

    def disconnect(self) -> None:
        if self.bus is not None:
            try:
                self.bus.shutdown()
            finally:
                self.bus = None

    def __enter__(self) -> "MG6010":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def _send_recv(self, data: bytes) -> bytes:
        if self.bus is None:
            raise MG6010Error("Not connected")
        
        cmd_byte = data[0]
        msg = can.Message(arbitration_id=self.arb_id, data=data, is_extended_id=False)

        try:
            self.bus.send(msg)
        except can.CanError as exc:
            raise MG6010Error(f"CAN send failed: {exc}") from exc

        deadline = time.monotonic() + self.response_timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            resp = self.bus.recv(timeout=remaining)
            if resp is None:
                break
            
            if resp.arbitration_id == self.arb_id and len(resp.data) == 8:
                if resp.data[0] == cmd_byte:
                    if (len(resp.data) > 1 and resp.data[1] != 0x00) or cmd_byte == 0x88:
                        return bytes(resp.data)

        raise MG6010Error(f"Timeout: no reply from motor ID {self.motor_id} for command 0x{cmd_byte:02X}")

    @staticmethod
    def _parse_status(resp: bytes) -> MotorStatus:
        temperature = struct.unpack_from("<b", resp, 1)[0]
        iq = struct.unpack_from("<h", resp, 2)[0]
        speed = struct.unpack_from("<h", resp, 4)[0]
        encoder = struct.unpack_from("<H", resp, 6)[0]
        return MotorStatus(temperature_c=temperature, torque_current_raw=iq, speed_dps=speed, encoder=encoder)

    def enable_motor(self) -> None:
        self._send_recv(bytes([self.CMD_MOTOR_ON]) + b"\x00" * 7)

    def disable_motor(self) -> None:
        self._send_recv(bytes([self.CMD_MOTOR_OFF]) + b"\x00" * 7)

    def stop(self) -> None:
        self._send_recv(bytes([self.CMD_MOTOR_STOP]) + b"\x00" * 7)

    def move_to_angle(self, angle_deg: float, max_speed_dps: float) -> MotorStatus:
        raw_angle = int(round(angle_deg * 100) * self.reduction)
        speed = max(0, min(65535, int(round(max_speed_dps))))
        data = bytes([self.CMD_MULTI_ANGLE_2, 0]) + struct.pack("<H", speed) + struct.pack("<i", raw_angle)
        return self._parse_status(self._send_recv(data))

    def read_status(self) -> MotorStatus:
        data = bytes([self.CMD_READ_STATE2]) + b"\x00" * 7
        return self._parse_status(self._send_recv(data))

    def read_position(self) -> float:
        data = bytes([0x92]) + b"\x00" * 7
        resp = self._send_recv(data)
        raw7 = resp[1:8]
        sign_byte = b"\xff" if (raw7[-1] & 0x80) else b"\x00"
        motor_angle = struct.unpack("<q", raw7 + sign_byte)[0]
        return motor_angle / 100.0 / self.reduction

    def clear_error(self) -> MotorState1:
        data = bytes([self.CMD_CLEAR_ERROR]) + b"\x00" * 7
        resp = self._send_recv(data)
        temperature = struct.unpack_from("<b", resp, 1)[0]
        voltage_raw = struct.unpack_from("<H", resp, 3)[0]
        error_state = resp[7]
        return MotorState1(temperature_c=temperature, voltage_v=voltage_raw / 10.0, error_state=error_state)
    
    def set_zero_position(self) -> None:
        """
        Comando 0x19: Escribe en la ROM. 
        Se envía directo al bus porque el motor NO envía respuesta a este comando.
        """
        if self.bus is None:
            raise MG6010Error("Not connected")
            
        data = bytes([0x19]) + b"\x00" * 7
        msg = can.Message(arbitration_id=self.arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)
