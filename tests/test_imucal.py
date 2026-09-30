"""tools/imucal.py is a script, not a module, so it is loaded by path."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "tools", "imucal.py")


@pytest.fixture(scope="module")
def imucal():
    spec = importlib.util.spec_from_file_location("imucal", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _readings(axis_map, offset=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    """What a chip mounted per `axis_map` reads with gravity along each body axis."""
    out = {}
    for body_index, body in enumerate("xyz"):
        entry = axis_map[body_index]
        chip = "xyz".index(entry.lstrip("-"))
        sign = -1.0 if entry.startswith("-") else 1.0
        for up, label in ((1.0, f"+{body}"), (-1.0, f"-{body}")):
            vector = [offset[i] for i in range(3)]
            vector[chip] += sign * up * scale[chip]
            out[label] = tuple(vector)
    return out


def test_an_aligned_chip_in_spec(imucal):
    result = imucal.analyse(_readings(["x", "y", "z"]), "xyz")
    assert result["axis_map"] == ["x", "y", "z"]
    assert result["problems"] == []
    assert all(row["offset_ok"] and row["scale_ok"] for row in result["axes"].values())


def test_a_chip_flipped_face_down_gives_the_axis_map(imucal):
    result = imucal.analyse(_readings(["x", "-y", "-z"]), "xyz")
    assert result["axis_map"] == ["x", "-y", "-z"]


def test_an_offset_is_told_apart_from_a_scale_error(imucal):
    # The 29 Sep preflight: z read -1.233 g. Face-down chip with a -0.233 g offset.
    shifted = imucal.analyse(_readings(["x", "y", "-z"], offset=(0, 0, -0.233)), "z")["axes"]["z"]
    assert shifted["entry"] == "-z"
    assert shifted["plus"] == pytest.approx(-1.233)
    assert shifted["offset"] == pytest.approx(-0.233)
    assert shifted["scale"] == pytest.approx(1.0)
    assert not shifted["offset_ok"] and shifted["scale_ok"]

    stretched = imucal.analyse(_readings(["x", "y", "-z"], scale=(1, 1, 1.235)), "z")["axes"]["z"]
    assert stretched["offset"] == pytest.approx(0.0)
    assert stretched["scale"] == pytest.approx(1.235)
    assert stretched["offset_ok"] and not stretched["scale_ok"]


def test_the_report_says_what_to_do(imucal):
    shifted = imucal.report(imucal.analyse(_readings(["x", "y", "z"], offset=(0, 0, -0.233)), "z"))
    assert any("chip works and reads shifted" in line for line in shifted)
    stretched = imucal.report(imucal.analyse(_readings(["x", "y", "z"], scale=(1, 1, 1.235)), "z"))
    assert any("Replace the GY-521" in line for line in stretched)


def test_one_axis_gives_no_axis_map(imucal):
    assert imucal.analyse(_readings(["x", "y", "z"]), "z")["axis_map"] is None


def test_two_body_axes_on_one_chip_axis_is_flagged(imucal):
    readings = _readings(["x", "y", "z"])
    readings["+y"], readings["-y"] = readings["+x"], readings["-x"]
    result = imucal.analyse(readings, "xyz")
    assert result["axis_map"] is None
    assert any("each chip axis should appear once" in p for p in result["problems"])


def test_a_board_mounted_at_an_angle_is_flagged(imucal):
    readings = _readings(["x", "y", "z"])
    readings["+z"], readings["-z"] = (0.26, 0.0, 0.97), (-0.26, 0.0, -0.97)  # about 15 deg
    result = imucal.analyse(readings, "z")
    assert any("mounted at an angle" in p for p in result["problems"])
