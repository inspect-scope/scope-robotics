"""tools/servolog.py is a script, not a module, so it is loaded by path."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "tools", "servolog.py")


@pytest.fixture(scope="module")
def servolog():
    spec = importlib.util.spec_from_file_location("servolog", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CONFIG = {
    "servos": {
        "R3": {"coxa": {"channel": 0, "direction": 1}, "femur": {"channel": 1, "direction": -1},
               "tibia": {"channel": 2, "direction": 1}},
        "L3": {"coxa": {"channel": 3, "direction": 1}, "femur": {"channel": 4, "direction": 1},
               "tibia": {"channel": 5, "direction": 1}},
    },
    "pulse_us": [600, 2400],
}


def _state(pulses, femur_r3=77.8, femur_l3=77.8):
    return {
        "pulses": pulses,
        "angles": {"R3": {"coxa": 0.0, "femur": femur_r3, "tibia": 131.7},
                   "L3": {"coxa": 0.0, "femur": femur_l3, "tibia": 131.7}},
    }


def test_channels_are_named_from_the_config(servolog):
    names = servolog.channel_names(CONFIG)
    assert names[1] == ("R3", "femur", -1)
    assert servolog.label(1, names) == "R3 femur"
    assert servolog.label(9, names) == "ch 9"  # not in this config: bare channel


def test_change_line_names_the_header_and_both_numbers(servolog):
    names = servolog.channel_names(CONFIG)
    before = _state([1500] * 6)
    after = _state([1500, 1622, 1500, 1500, 1500, 1500], femur_r3=46.2)
    assert servolog.changed(before["pulses"], after["pulses"]) == [1]
    line = servolog.change_line(0.0, 1, before, after, names)
    assert "R3 femur" in line and "SERVO  2" in line and "ch  1" in line
    assert "77.8 ->   46.2 deg" in line and "1500 -> 1622 us (+122)" in line


def test_summary_flags_what_did_not_move_and_what_hit_the_clamp(servolog):
    names = servolog.channel_names(CONFIG)
    first = _state([1500, 1500, 1500, 1500, 1500, 1500])
    tracks = {c: servolog.Track(p, servolog.joint_angle(first, c, names)) for c, p in enumerate(first["pulses"])}
    last = _state([1500, 1622, 2400, 1500, 1622, 1500], femur_r3=46.2, femur_l3=46.2)
    for c, p in enumerate(last["pulses"]):
        tracks[c].update(p, servolog.joint_angle(last, c, names))
    text = "\n".join(servolog.summarize(tracks, names, CONFIG["pulse_us"]))
    assert "never moved: R3 coxa, L3 coxa, L3 tibia" in text
    assert "hit the pulse clamp [600, 2400]: R3 tibia at 2400" in text
    assert "differs between legs" not in text  # both femurs moved by the same joint angle


def test_summary_flags_a_leg_that_moved_differently(servolog):
    names = servolog.channel_names(CONFIG)
    first = _state([1500] * 6)
    tracks = {c: servolog.Track(p, servolog.joint_angle(first, c, names)) for c, p in enumerate(first["pulses"])}
    last = _state([1500, 1622, 1500, 1500, 1510, 1500], femur_r3=46.2, femur_l3=75.0)
    for c, p in enumerate(last["pulses"]):
        tracks[c].update(p, servolog.joint_angle(last, c, names))
    text = "\n".join(servolog.summarize(tracks, names, CONFIG["pulse_us"]))
    assert "differs between legs" in text
    assert "femur: L3 -2.8, R3 -31.6" in text
