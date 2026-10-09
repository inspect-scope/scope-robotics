"""Named stance presets.

Chica's standard / race / offroad retune body lift, step lift and speed
factor. Ours keep the current stepper and apply those ratios to the yaml
stance. A new mode is another ModeSpec row.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from .config import Stance

# Chica MODE_* vs MODE_STANDARD (body lift 40, step lift 40, speed 1.0).
RACE_HEIGHT = 35.0 / 40.0
RACE_LIFT = 30.0 / 40.0
RACE_SPEED = 2.0
OFFROAD_HEIGHT = 60.0 / 40.0
OFFROAD_LIFT = 99.0 / 40.0
OFFROAD_SPEED = 0.6

RIDE_MAX_MM = 160.0  # offroad is 1.5x stand; 160 still solves inside the joint box
LIFT_RANGE = (5.0, 70.0)
CYCLE_RANGE = (0.3, 2.5)
SPEED_RANGE = (10.0, 250.0)
YAW_RANGE = (5.0, 90.0)


class StanceMode:
    Normal = "normal"
    Speed = "speed"
    Offroad = "offroad"


MODE_KINDS = (StanceMode.Normal, StanceMode.Speed, StanceMode.Offroad)


@dataclass(frozen=True)
class ModeSpec:
    """One stance preset. `apply` writes into the live Stance."""

    kind: str
    chica: str
    ride_height: float
    step_lift: float
    cycle_s: float
    max_speed: float
    max_yaw_rate: float

    def apply(self, stance: Stance) -> None:
        stance.ride_height = self.ride_height
        stance.step_lift = self.step_lift
        stance.cycle_s = self.cycle_s
        stance.max_speed = self.max_speed
        stance.max_yaw_rate = self.max_yaw_rate


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _scaled(base: Stance, kind: str, chica: str, height: float, lift: float, speed: float) -> ModeSpec:
    sit = base.sit_height
    return ModeSpec(
        kind,
        chica,
        ride_height=_clamp(base.ride_height * height, sit, RIDE_MAX_MM),
        step_lift=_clamp(base.step_lift * lift, *LIFT_RANGE),
        cycle_s=_clamp(base.cycle_s / speed, *CYCLE_RANGE),
        max_speed=_clamp(base.max_speed * speed, *SPEED_RANGE),
        max_yaw_rate=_clamp(base.max_yaw_rate * speed, *YAW_RANGE),
    )


def modes_for(base: Stance) -> Dict[str, ModeSpec]:
    """Build the mode catalog from the yaml stance (Chica standard)."""
    specs = (
        ModeSpec(
            StanceMode.Normal,
            "standard",
            base.ride_height,
            base.step_lift,
            base.cycle_s,
            base.max_speed,
            base.max_yaw_rate,
        ),
        _scaled(base, StanceMode.Speed, "race", RACE_HEIGHT, RACE_LIFT, RACE_SPEED),
        _scaled(base, StanceMode.Offroad, "offroad", OFFROAD_HEIGHT, OFFROAD_LIFT, OFFROAD_SPEED),
    )
    return {spec.kind: spec for spec in specs}


def mode_catalog(modes: Dict[str, ModeSpec]) -> List[Dict[str, str]]:
    return [{"id": kind, "chica": modes[kind].chica} for kind in MODE_KINDS]


def lookup_mode(modes: Dict[str, ModeSpec], name: str) -> Optional[ModeSpec]:
    spec = modes.get(name)
    if spec is not None:
        return spec
    return next((row for row in modes.values() if row.chica == name), None)
