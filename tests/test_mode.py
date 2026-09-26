import pytest

from hexapod import config as config_mod
from hexapod.mode import StanceMode, lookup_mode, mode_catalog, modes_for


@pytest.fixture()
def base():
    return config_mod.load().stance


def test_catalog_lists_chica_mode_names(base):
    rows = mode_catalog(modes_for(base))
    assert [row["id"] for row in rows] == ["normal", "speed", "offroad"]
    assert {row["chica"] for row in rows} == {"standard", "race", "offroad"}


def test_speed_is_lower_faster(base):
    spec = modes_for(base)[StanceMode.Speed]
    assert spec.ride_height < base.ride_height
    assert spec.step_lift < base.step_lift
    assert spec.cycle_s < base.cycle_s
    assert spec.max_speed > base.max_speed


def test_offroad_is_higher_slower(base):
    spec = modes_for(base)[StanceMode.Offroad]
    assert spec.ride_height > base.ride_height
    assert spec.step_lift > base.step_lift
    assert spec.cycle_s > base.cycle_s
    assert spec.max_speed < base.max_speed


def test_chica_name_selects_the_mode(base):
    modes = modes_for(base)
    assert lookup_mode(modes, "race").kind == StanceMode.Speed
    assert lookup_mode(modes, "standard").kind == StanceMode.Normal
    assert lookup_mode(modes, "gallop") is None


def test_apply_writes_the_live_stance(base):
    stance = config_mod.load().stance
    modes_for(stance)[StanceMode.Speed].apply(stance)
    assert stance.max_speed > base.max_speed
    assert stance.cycle_s < base.cycle_s
