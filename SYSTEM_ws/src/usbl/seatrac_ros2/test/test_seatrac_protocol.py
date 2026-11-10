"""Tests for strict, CID-agnostic SeaTrac transport parsing."""

import json

import pytest

from seatrac_ros2.seatrac_protocol import (
    crc16_ibm,
    encode_ascii_frame,
    encode_capture_record,
    parse_ascii_frame,
    SeaTracFrameError,
    SeaTracStreamFramer,
)


def test_parses_valid_frame_and_preserves_unknown_cid():
    """Integrity parsing preserves an unknown CID without interpreting it."""
    raw = encode_ascii_frame(bytes([0xE7, 0x01, 0xA5]))

    frame = parse_ascii_frame(raw + '\r\n')

    assert frame.command_id == 0xE7
    assert frame.payload == bytes([0x01, 0xA5])
    assert frame.message_data == bytes([0xE7, 0x01, 0xA5])
    assert frame.raw_text == raw


def test_rejects_corrupt_crc():
    """A modified checksum can never be treated as an intact frame."""
    raw = encode_ascii_frame(bytes([0x10, 0x00]))
    corrupt = raw[:-1] + ('0' if raw[-1] != '0' else '1')

    with pytest.raises(SeaTracFrameError, match='CRC mismatch'):
        parse_ascii_frame(corrupt)


@pytest.mark.parametrize(
    'raw, error',
    [
        ('$02', 'shorter'),
        ('$020', 'odd'),
        ('$02GG00', 'non-hex'),
        ('02AABB', 'does not start'),
        ('$0200\n', 'incomplete line terminator'),
    ],
)
def test_rejects_truncated_or_malformed_frames(raw, error):
    """Transport defects are rejected before CID-specific logic can run."""
    with pytest.raises(SeaTracFrameError, match=error):
        parse_ascii_frame(raw)


def test_encoder_rejects_message_without_cid():
    """Fixture encoder requires the one-byte message command identifier."""
    with pytest.raises(ValueError, match='command ID'):
        encode_ascii_frame(b'')


def test_crc_matches_independent_standard_check_vector():
    """CRC-16/ARC has the published 123456789 check value 0xBB3D."""
    assert crc16_ibm(b'123456789') == 0xBB3D


@pytest.mark.parametrize(
    'message_data, expected',
    [
        (b'\x02', '$0281C1'),
        (b'\x10\x00', '$10000DC0'),
        (b'\x20\x04', '$20041803'),
        (b'\x20\x05', '$2005D9C3'),
    ],
)
def test_encoder_matches_known_seatrac_command_constants(
    message_data, expected
):
    """Command fixtures guard byte order independently of runtime routing."""
    assert encode_ascii_frame(message_data) == expected


def test_stream_framer_preserves_exact_fragmented_frames():
    """Framing retains CRLF and works across arbitrary transport chunks."""
    framer = SeaTracStreamFramer(max_buffer_bytes=32)

    first = framer.feed(b'$0281')
    second = framer.feed(b'C1\r\n$10000DC0\r\n')

    assert first.frames == ()
    assert second.frames == (b'$0281C1\r\n', b'$10000DC0\r\n')
    assert second.discarded_bytes == 0
    assert framer.buffered_bytes == 0


def test_stream_framer_discards_oversized_unterminated_input():
    """Missing terminators cannot grow the receive buffer indefinitely."""
    framer = SeaTracStreamFramer(max_buffer_bytes=16)

    result = framer.feed(b'X' * 17)

    assert result.frames == ()
    assert result.discarded_bytes == 17
    assert result.discarded_data == b'X' * 17
    assert framer.buffered_bytes == 0


def test_capture_record_preserves_bytes_timing_sequence_and_crc():
    """The forensic record is sufficient to reconstruct an intact frame."""
    raw = b'$0281C1\r\n'

    record = json.loads(
        encode_capture_record(
            sequence=7,
            received_unix_ns=123,
            received_monotonic_ns=456,
            transport='blueos_udp',
            raw_bytes=raw,
        )
    )

    assert bytes.fromhex(record['raw_bytes_hex']) == raw
    assert record['sequence'] == 7
    assert record['record_kind'] == 'frame'
    assert record['received_unix_ns'] == 123
    assert record['received_monotonic_ns'] == 456
    assert record['crc_status'] == 'valid'
    assert record['command_id'] == 0x02
    assert record['parse_error'] is None


def test_capture_record_retains_invalid_frame_without_classification():
    """Corrupt input remains evidence but receives no command semantics."""
    raw = b'$0281C0\r\n'

    record = json.loads(
        encode_capture_record(
            sequence=8,
            received_unix_ns=124,
            received_monotonic_ns=457,
            transport='blueos_udp',
            raw_bytes=raw,
        )
    )

    assert bytes.fromhex(record['raw_bytes_hex']) == raw
    assert record['crc_status'] == 'invalid'
    assert record['command_id'] is None
    assert 'CRC mismatch' in record['parse_error']


def test_capture_record_labels_unterminated_overflow():
    """Overflow evidence is byte exact and cannot masquerade as a frame."""
    raw = b'X' * 17

    record = json.loads(
        encode_capture_record(
            sequence=9,
            received_unix_ns=125,
            received_monotonic_ns=458,
            transport='blueos_udp',
            raw_bytes=raw,
            record_kind='unterminated_overflow',
        )
    )

    assert bytes.fromhex(record['raw_bytes_hex']) == raw
    assert record['record_kind'] == 'unterminated_overflow'
    assert record['crc_status'] == 'invalid'
