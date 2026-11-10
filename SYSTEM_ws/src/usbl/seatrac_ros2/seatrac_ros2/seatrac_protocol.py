"""
Pure SeaTrac ASCII framing helpers.

This module deliberately stops at transport integrity and native message
representation.  It does not identify any CID as a position measurement and
must not be used to publish ``/usbl/relative_pose`` until that CID has been
captured, documented and fixture-tested for the installed firmware.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Mapping, Optional


class SeaTracFrameError(ValueError):
    """Raised when an ASCII SeaTrac frame is incomplete or invalid."""


def crc16_ibm(data: bytes) -> int:
    """Return the CRC-16-IBM checksum used by SeaTrac frames."""
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


@dataclass(frozen=True)
class SeaTracFrame:
    """An integrity-checked SeaTrac message, without CID-specific decoding."""

    command_id: int
    payload: bytes
    message_data: bytes
    raw_text: str


@dataclass(frozen=True)
class NativeAcousticMeasurement:
    """
    Preserve a future acoustic measurement in its native convention.

    ``native_fields`` intentionally has no forward/left/up interpretation.
    A CID-specific decoder must first establish units, axes, validity bits and
    source-time semantics.  Only a separately reviewed frame normalizer may
    turn this model into a body-frame USBL fix.
    """

    command_id: int
    beacon_id: Optional[int]
    source_timestamp_s: Optional[float]
    range_m: Optional[float]
    bearing_deg: Optional[float]
    elevation_deg: Optional[float]
    position_valid: Optional[bool]
    raw_payload: bytes
    native_fields: Mapping[str, object]


@dataclass(frozen=True)
class StreamFeedResult:
    """Complete frames and fail-closed overflow information from one feed."""

    frames: tuple[bytes, ...]
    discarded_data: bytes = b''

    @property
    def discarded_bytes(self) -> int:
        """Return the number of unterminated bytes rejected by the framer."""
        return len(self.discarded_data)


class SeaTracStreamFramer:
    """
    Split a byte stream on CRLF while keeping residual memory bounded.

    Returned frames include their CRLF terminator so a capture can preserve
    the exact bytes received from the transport.  A residual stream that
    exceeds ``max_buffer_bytes`` is discarded in full: retaining an arbitrary
    suffix could turn corrupt input into an apparently valid frame.
    """

    def __init__(self, max_buffer_bytes: int = 65536) -> None:
        """Create a framer with a strict residual-buffer limit."""
        if max_buffer_bytes < 16:
            raise ValueError('max_buffer_bytes must be at least 16')
        self.max_buffer_bytes = max_buffer_bytes
        self._buffer = bytearray()

    @property
    def buffered_bytes(self) -> int:
        """Return the number of unterminated bytes currently retained."""
        return len(self._buffer)

    def clear(self) -> None:
        """Discard all residual bytes."""
        self._buffer.clear()

    def feed(self, chunk: bytes) -> StreamFeedResult:
        """Consume one transport chunk and return all complete frames."""
        if not isinstance(chunk, bytes):
            raise TypeError('chunk must be bytes')

        self._buffer.extend(chunk)
        frames = []
        terminator = b'\r\n'
        while terminator in self._buffer:
            raw_line, _, remaining = self._buffer.partition(terminator)
            self._buffer = bytearray(remaining)
            if raw_line:
                frames.append(bytes(raw_line) + terminator)

        discarded_data = b''
        if len(self._buffer) > self.max_buffer_bytes:
            discarded_data = bytes(self._buffer)
            self._buffer.clear()

        return StreamFeedResult(tuple(frames), discarded_data)


def encode_capture_record(
    *,
    sequence: int,
    received_unix_ns: int,
    received_monotonic_ns: int,
    transport: str,
    raw_bytes: bytes,
    record_kind: str = 'frame',
) -> str:
    """
    Return a stable JSON record for one byte-exact received frame.

    Integrity is evaluated without assigning semantics to the command ID.
    Invalid or non-ASCII frames remain evidence and are never classified as a
    system-info, status or acoustic-position message.
    """
    if sequence < 0:
        raise ValueError('sequence must be non-negative')
    if received_unix_ns < 0 or received_monotonic_ns < 0:
        raise ValueError('capture timestamps must be non-negative')
    if not transport:
        raise ValueError('transport must not be empty')
    if record_kind not in ('frame', 'unterminated_overflow'):
        raise ValueError('unsupported capture record_kind')
    if not isinstance(raw_bytes, bytes):
        raise TypeError('raw_bytes must be bytes')

    command_id = None
    crc_status = 'invalid'
    parse_error = None
    try:
        frame = parse_ascii_frame(raw_bytes)
    except SeaTracFrameError as error:
        parse_error = str(error)
    else:
        command_id = frame.command_id
        crc_status = 'valid'

    record = {
        'schema': 'seatrac.byte_capture.v1',
        'record_kind': record_kind,
        'sequence': sequence,
        'received_unix_ns': received_unix_ns,
        'received_monotonic_ns': received_monotonic_ns,
        'transport': transport,
        'raw_bytes_hex': raw_bytes.hex().upper(),
        'byte_count': len(raw_bytes),
        'crc_status': crc_status,
        'command_id': command_id,
        'parse_error': parse_error,
    }
    return json.dumps(record, sort_keys=True, separators=(',', ':'))


def parse_ascii_frame(raw_frame: str | bytes) -> SeaTracFrame:
    """
    Parse one complete ``$...`` SeaTrac ASCII response.

    The optional CR/LF terminator is accepted because transports may preserve
    it.  All other whitespace, malformed hex, short frames and bad CRCs are
    rejected.  Successful parsing says only that a frame is intact; it says
    nothing about the meaning or safety of its command ID.
    """
    if isinstance(raw_frame, bytes):
        try:
            text = raw_frame.decode('ascii')
        except UnicodeDecodeError as error:
            raise SeaTracFrameError('frame is not ASCII') from error
    elif isinstance(raw_frame, str):
        text = raw_frame
    else:
        raise TypeError('raw_frame must be str or bytes')

    if text.endswith('\r\n'):
        text = text[:-2]
    elif text.endswith('\n') or text.endswith('\r'):
        raise SeaTracFrameError('frame has an incomplete line terminator')

    if not text.startswith('$'):
        raise SeaTracFrameError('frame does not start with $')

    hex_payload = text[1:]
    if not hex_payload:
        raise SeaTracFrameError('frame has no hex payload')
    if len(hex_payload) % 2:
        raise SeaTracFrameError('frame has an odd number of hex characters')
    if any(
        character not in '0123456789abcdefABCDEF'
        for character in hex_payload
    ):
        raise SeaTracFrameError('frame contains non-hex characters')

    binary = bytes.fromhex(hex_payload)
    if len(binary) < 3:
        raise SeaTracFrameError('frame is shorter than CID and CRC')

    message_data = binary[:-2]
    received_crc = int.from_bytes(binary[-2:], byteorder='little')
    expected_crc = crc16_ibm(message_data)
    if received_crc != expected_crc:
        raise SeaTracFrameError(
            'CRC mismatch: expected 0x%04X, received 0x%04X'
            % (expected_crc, received_crc)
        )

    return SeaTracFrame(
        command_id=message_data[0],
        payload=message_data[1:],
        message_data=message_data,
        raw_text=text,
    )


def encode_ascii_frame(message_data: bytes) -> str:
    """Encode a message for fixtures and commands without CID semantics."""
    if not message_data:
        raise ValueError('message_data must include a command ID')
    checksum = crc16_ibm(message_data).to_bytes(2, byteorder='little')
    return '$' + (message_data + checksum).hex().upper()
