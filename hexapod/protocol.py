"""Chica serial protocol codec for the Servo2040 firmware.

Wire format, read off the firmware source
(EddieCarrera/chica-servo2040-simpleDriver, chica-servo2040/chica-servo2040.cpp):

    host -> board, SET:  0xD3  start_idx  count  (lo hi) * count
    host -> board, GET:  0xC7  start_idx  count
    board -> host, GET:  0xC7  start_idx  count  (lo hi) * count

Every byte after the command byte is 7-bit; the firmware masks values with 0x7F.
Values are 14-bit little-endian split as lo = v & 0x7F, hi = (v >> 7) & 0x7F.
The command byte is the only byte with bit 7 set, which is how the parser
resynchronises: it skips bytes until it sees one with the high bit.

Channel indices are the `cmdPins` enum in the firmware's main.h, and they line up
exactly with the `P00`..`P26` pin names in chica-config-2040.txt.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

SET_CMD = 0xD3
GET_CMD = 0xC7

# Channel map (firmware `cmdPins` enum == config-file P-numbers).
SERVO_BASE = 0
NUM_SERVOS = 18
TOUCH_BASE = 18
NUM_TOUCH = 6
CH_CURRENT = 24
CH_VOLTAGE = 25
CH_RELAY = 26  # A0: servo power relay + software PWM enable
CH_A1 = 27
CH_A2 = 28
NUM_CHANNELS = 29

MAX_COUNT = 127  # firmware's MAX_COUNT_VALUE
MAX_VALUE = 0x3FFF  # 14 bits over two 7-bit bytes

# Firmware scaling constants (main.h).
_B1024_3V3_RATIO = 310.3  # 1024 / 3.3
_CURRENT_LSB = 0.0814  # amps per count
_CURRENT_OFFSET = 512  # counts at zero amps


class ProtocolError(ValueError):
    """Malformed frame."""


def _encode_value(value: int) -> Tuple[int, int]:
    if not 0 <= value <= MAX_VALUE:
        raise ProtocolError(f"value {value} outside 0..{MAX_VALUE}")
    return value & 0x7F, (value >> 7) & 0x7F


def _decode_value(lo: int, hi: int) -> int:
    return (lo & 0x7F) | ((hi & 0x7F) << 7)


def _check_span(start_idx: int, count: int) -> None:
    if not 0 <= start_idx < NUM_CHANNELS:
        raise ProtocolError(f"start_idx {start_idx} outside 0..{NUM_CHANNELS - 1}")
    if not 1 <= count <= MAX_COUNT:
        raise ProtocolError(f"count {count} outside 1..{MAX_COUNT}")
    if start_idx + count > NUM_CHANNELS:
        raise ProtocolError(f"span {start_idx}..{start_idx + count - 1} runs past the last channel")


def encode_set(start_idx: int, values: Sequence[int]) -> bytes:
    """Frame a SET for `len(values)` consecutive channels starting at `start_idx`."""
    _check_span(start_idx, len(values))
    out = bytearray((SET_CMD, start_idx, len(values)))
    for value in values:
        lo, hi = _encode_value(int(value))
        out.append(lo)
        out.append(hi)
    return bytes(out)


def encode_get(start_idx: int, count: int) -> bytes:
    """Frame a GET for `count` consecutive channels starting at `start_idx`."""
    _check_span(start_idx, count)
    return bytes((GET_CMD, start_idx, count))


def encode_servo_pulses(pulses: Sequence[int], start_servo: int = 0) -> bytes:
    """Frame the pulse widths (microseconds) for a run of servos."""
    if start_servo + len(pulses) > NUM_SERVOS:
        raise ProtocolError("servo run past SERVO18")
    return encode_set(SERVO_BASE + start_servo, pulses)


def encode_torque(enabled: bool) -> bytes:
    """Frame the relay/torque command. Zero drops PWM on every servo."""
    return encode_set(CH_RELAY, [1 if enabled else 0])


@dataclass(frozen=True)
class GetReply:
    start_idx: int
    values: List[int]

    def value_for(self, channel: int) -> Optional[int]:
        offset = channel - self.start_idx
        if 0 <= offset < len(self.values):
            return self.values[offset]
        return None


def decode_replies(buffer: bytes) -> Tuple[List[GetReply], bytes]:
    """Pull every complete GET reply out of `buffer`.

    Returns the replies and the unconsumed tail, so the caller can keep the tail
    and append the next read. Bytes before a command byte are dropped: that is
    the resync rule the firmware itself uses.
    """
    replies: List[GetReply] = []
    pos = 0
    n = len(buffer)
    while True:
        # Resync: advance to the next byte with bit 7 set.
        while pos < n and not (buffer[pos] & 0x80):
            pos += 1
        if pos + 3 > n:
            break
        cmd, start_idx, count = buffer[pos], buffer[pos + 1], buffer[pos + 2]
        if cmd != GET_CMD:
            pos += 1  # not a reply we understand; skip this command byte
            continue
        end = pos + 3 + 2 * count
        if end > n:
            break  # frame still in flight
        payload = buffer[pos + 3 : end]
        values = [_decode_value(payload[i], payload[i + 1]) for i in range(0, len(payload), 2)]
        replies.append(GetReply(start_idx=start_idx, values=values))
        pos = end
    return replies, buffer[pos:]


# --- raw counts -> engineering units -------------------------------------------------
# The firmware packs each reading as an integer; these undo that packing.


def counts_to_volts(counts: int) -> float:
    """Battery volts. Firmware sends round(read_voltage() * 1024/3.3)."""
    return counts / _B1024_3V3_RATIO


def counts_to_amps(counts: int) -> float:
    """Servo rail amps. Firmware sends round(amps / 0.0814) + 512, so it is signed."""
    return (counts - _CURRENT_OFFSET) * _CURRENT_LSB


def counts_to_sensor_volts(counts: int) -> float:
    """Touch-sensor pin volts, 0..3.3."""
    return counts / _B1024_3V3_RATIO
