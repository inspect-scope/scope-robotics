import pytest

from hexapod import protocol


def test_set_frame_matches_the_firmware_layout():
    assert protocol.encode_set(0, [1500, 1000]) == bytes([0xD3, 0, 2, 0x5C, 0x0B, 0x68, 0x07])


def test_every_byte_after_the_command_has_bit_7_clear():
    frame = protocol.encode_servo_pulses([protocol.MAX_VALUE] * 18)
    assert frame[0] == protocol.SET_CMD
    assert all(byte < 0x80 for byte in frame[1:])


def test_torque_uses_the_relay_channel():
    assert protocol.encode_torque(True) == bytes([0xD3, protocol.CH_RELAY, 1, 1, 0])
    assert protocol.encode_torque(False) == bytes([0xD3, protocol.CH_RELAY, 1, 0, 0])


@pytest.mark.parametrize("value", [0, 1, 127, 128, 1500, 2500, protocol.MAX_VALUE])
def test_value_round_trip(value):
    replies, tail = protocol.decode_replies(bytes([protocol.GET_CMD, 0, 1]) + protocol.encode_set(0, [value])[3:])
    assert tail == b""
    assert replies[0].values == [value]


def test_decode_keeps_a_partial_frame_for_the_next_read():
    whole = bytes([protocol.GET_CMD, 24, 2, 0x00, 0x04, 0x5C, 0x0B])
    replies, tail = protocol.decode_replies(whole[:5])
    assert replies == [] and tail == whole[:5]
    replies, tail = protocol.decode_replies(whole)
    assert tail == b""
    assert replies[0].value_for(protocol.CH_VOLTAGE) == 1500


def test_decode_resyncs_past_junk():
    noise = bytes([0x11, 0x22, 0x33])
    replies, _ = protocol.decode_replies(noise + bytes([protocol.GET_CMD, 25, 1, 0x5C, 0x0B]))
    assert len(replies) == 1 and replies[0].values == [1500]


def test_decode_handles_back_to_back_replies():
    one = bytes([protocol.GET_CMD, 25, 1, 0x5C, 0x0B])
    replies, tail = protocol.decode_replies(one * 3)
    assert len(replies) == 3 and tail == b""


def test_units_undo_the_firmware_scaling():
    assert protocol.counts_to_volts(round(7.4 * 310.3)) == pytest.approx(7.4, abs=0.01)
    assert protocol.counts_to_amps(512) == pytest.approx(0.0)
    assert protocol.counts_to_amps(512 + 100) == pytest.approx(8.14, abs=0.01)


@pytest.mark.parametrize(
    "call",
    [
        lambda: protocol.encode_set(0, [protocol.MAX_VALUE + 1]),
        lambda: protocol.encode_set(-1, [0]),
        lambda: protocol.encode_get(protocol.NUM_CHANNELS, 1),
        lambda: protocol.encode_get(0, 0),
        lambda: protocol.encode_servo_pulses([1500] * 19),
    ],
)
def test_bad_frames_are_refused(call):
    with pytest.raises(protocol.ProtocolError):
        call()
