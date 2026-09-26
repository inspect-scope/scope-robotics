"""Gait generator.

Produces ground-frame foot targets from a body velocity command. Patterns
share the same stroke math (`v + omega x r`) and differ by phase layout:

  tripod  two groups of three, 50/50 stance (Chica walk3)
  ripple  one leg at a time, 5/6 stance (Chica walk1)
  wave    one leg around the body, 5/6 stance (Chica walkwave)

Per leg the ground velocity is `v_body + omega x r`, where `r` is the leg's
neutral foot position, so translation and turning compose without a special
case for spinning on the spot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

from .config import Config

Vec3 = Tuple[float, float, float]


class GaitKind:
    Tripod = "tripod"
    Ripple = "ripple"
    Wave = "wave"


GAIT_KINDS = (GaitKind.Tripod, GaitKind.Ripple, GaitKind.Wave)
WAVE_ORDER = ("R1", "R2", "R3", "L3", "L2", "L1")
STANCE_FRACTION = 0.5  # tripod: half the cycle on the ground, half in the air
_STOP_EPSILON = 1.0  # mm of remaining stroke below which we call it stopped


@dataclass(frozen=True)
class Velocity:
    """Body velocity command. mm/s and deg/s, in the body frame."""

    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0

    def scaled(self, config: Config) -> "Velocity":
        """Interpret -1..1 joystick axes as a velocity, using the configured maxima."""
        s = config.stance
        speed = math.hypot(self.vx, self.vy)
        if speed > 1.0:  # keep diagonals from exceeding full stick
            self = Velocity(self.vx / speed, self.vy / speed, self.yaw_rate)
        return Velocity(
            vx=self.vx * s.max_speed,
            vy=self.vy * s.max_speed,
            yaw_rate=max(-1.0, min(1.0, self.yaw_rate)) * s.max_yaw_rate,
        )

    @property
    def is_zero(self) -> bool:
        return abs(self.vx) < 1e-6 and abs(self.vy) < 1e-6 and abs(self.yaw_rate) < 1e-6

    def approach(self, target: "Velocity", max_delta: float, max_yaw_delta: float) -> "Velocity":
        """Rate-limit toward `target` so stick slams do not turn into stumbles."""
        return Velocity(
            vx=_step(self.vx, target.vx, max_delta),
            vy=_step(self.vy, target.vy, max_delta),
            yaw_rate=_step(self.yaw_rate, target.yaw_rate, max_yaw_delta),
        )


def _step(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if delta > max_delta:
        return current + max_delta
    if delta < -max_delta:
        return current - max_delta
    return target


class TripodGait:
    def __init__(self, config: Config, pattern: str = GaitKind.Tripod):
        self.config = config
        self.groups: List[List[str]] = config.tripod_groups
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False
        self.set_pattern(pattern)

    def set_pattern(self, pattern: str) -> None:
        if pattern not in GAIT_KINDS:
            raise ValueError(f"unknown gait pattern {pattern!r}")
        self.pattern = pattern
        self._offset, self.stance_fraction = _phase_layout(pattern, self.groups)
        self.phase = 0.0

    def reset(self) -> None:
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False

    def strokes(self, velocity: Velocity) -> Dict[str, Tuple[float, float]]:
        """Ground displacement each foot covers during one stance phase, per leg."""
        s = self.config.stance
        stance_time = s.cycle_s * self.stance_fraction
        omega = math.radians(velocity.yaw_rate)
        out: Dict[str, Tuple[float, float]] = {}
        for name, leg in self.config.legs.items():
            rx, ry = leg.neutral_xy
            vx = velocity.vx - omega * ry
            vy = velocity.vy + omega * rx
            sx, sy = vx * stance_time, vy * stance_time
            magnitude = math.hypot(sx, sy)
            if magnitude > s.max_stride:
                scale = s.max_stride / magnitude
                sx, sy = sx * scale, sy * scale
            out[name] = (sx, sy)
        return out

    def step(self, dt: float, command: Velocity, height: float) -> Dict[str, Vec3]:
        """Advance the gait by `dt` and return foot targets at the given ride height."""
        s = self.config.stance
        # A full stop takes ~2 cycles of ramp; that is gentle enough to stay upright.
        max_delta = s.max_speed * dt / (s.cycle_s * 2.0) * 4.0
        max_yaw_delta = s.max_yaw_rate * dt / (s.cycle_s * 2.0) * 4.0
        self.velocity = self.velocity.approach(command, max_delta, max_yaw_delta)

        strokes = self.strokes(self.velocity)
        # Never lift a foot above the body plane: at low ride heights that puts the
        # target inside the leg's inner reach limit and IK folds up degenerately.
        lift_cap = max(0.0, height - s.min_foot_depth)
        step_lift = min(s.step_lift, lift_cap)
        biggest = max((math.hypot(*stroke) for stroke in strokes.values()), default=0.0)

        if command.is_zero and biggest < _STOP_EPSILON:
            # Settle: park the phase so every foot is on the ground at neutral.
            self.walking = False
            self.phase = 0.0
            self.velocity = Velocity()
            strokes = {name: (0.0, 0.0) for name in strokes}
        else:
            self.walking = True
            self.phase = (self.phase + dt / s.cycle_s) % 1.0

        swing_fraction = 1.0 - self.stance_fraction
        feet: Dict[str, Vec3] = {}
        for name, leg in self.config.legs.items():
            nx, ny = leg.neutral_xy
            sx, sy = strokes[name]
            leg_phase = (self.phase + self._offset[name]) % 1.0
            if leg_phase < swing_fraction:
                # Swing: return to the front of the stroke, lifting over a sine arc.
                progress = leg_phase / swing_fraction
                along = progress - 0.5
                lift = step_lift * math.sin(math.pi * progress) if biggest >= _STOP_EPSILON else 0.0
            else:
                progress = (leg_phase - swing_fraction) / self.stance_fraction
                along = 0.5 - progress
                lift = 0.0
            feet[name] = (nx + sx * along, ny + sy * along, -height + lift)
        return feet

    def group_in_swing(self) -> int:
        return 0 if self.phase < (1.0 - self.stance_fraction) else 1


def _phase_layout(pattern: str, groups: List[List[str]]) -> Tuple[Dict[str, float], float]:
    """Per-leg phase offset in [0, 1) and the stance duty cycle."""
    if pattern == GaitKind.Tripod:
        offsets = {name: 0.5 * index for index, group in enumerate(groups) for name in group}
        return offsets, STANCE_FRACTION

    if pattern == GaitKind.Ripple:
        # Zip the two tripods so lift stays balanced: L1, R1, R2, L2, L3, R3.
        order = [name for pair in zip(*groups) for name in pair]
        n = len(order)
        return {name: i / n for i, name in enumerate(order)}, 1.0 - 1.0 / n

    if pattern == GaitKind.Wave:
        n = len(WAVE_ORDER)
        return {name: i / n for i, name in enumerate(WAVE_ORDER)}, 1.0 - 1.0 / n

    raise ValueError(f"unknown gait pattern {pattern!r}")
