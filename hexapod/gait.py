"""Tripod gait generator.

Produces ground-frame foot targets from a body velocity command. Two groups of
three legs alternate: while one group is on the ground pushing the body along,
the other is in the air returning to the front of its stroke.

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
    def __init__(self, config: Config):
        self.config = config
        self.groups: List[List[str]] = config.tripod_groups
        self._group_of: Dict[str, int] = {
            name: index for index, group in enumerate(self.groups) for name in group
        }
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False

    def reset(self) -> None:
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False

    def strokes(self, velocity: Velocity) -> Dict[str, Tuple[float, float]]:
        """Ground displacement each foot covers during one stance phase, per leg."""
        s = self.config.stance
        stance_time = s.cycle_s * STANCE_FRACTION
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

        feet: Dict[str, Vec3] = {}
        for name, leg in self.config.legs.items():
            nx, ny = leg.neutral_xy
            sx, sy = strokes[name]
            leg_phase = (self.phase + 0.5 * self._group_of[name]) % 1.0
            if leg_phase < STANCE_FRACTION:
                # Swing: return to the front of the stroke, lifting over a sine arc.
                progress = leg_phase / STANCE_FRACTION
                along = progress - 0.5
                lift = step_lift * math.sin(math.pi * progress) if biggest >= _STOP_EPSILON else 0.0
            else:
                progress = (leg_phase - STANCE_FRACTION) / (1.0 - STANCE_FRACTION)
                along = 0.5 - progress
                lift = 0.0
            feet[name] = (nx + sx * along, ny + sy * along, -height + lift)
        return feet

    def group_in_swing(self) -> int:
        return 0 if self.phase < STANCE_FRACTION else 1
