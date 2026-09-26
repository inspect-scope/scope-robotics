"""Gait generator.

Walks share one stroke (`v + omega x r`) and differ only by a phase table.
Tricks (bounce, jump) sit beside that loop so a new walk is a catalog row,
not a new class.

Chica names are the client commands from server 0.0.4a. Ours are the ids.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Dict, List, Optional, Tuple

from .config import Config
from .kinematics import BodyPose

Vec3 = Tuple[float, float, float]
Group = Tuple[str, ...]


class GaitKind:
    Tripod = "tripod"
    Triple = "triple"
    Triple25 = "triple25"
    Ripple = "ripple"
    Ripple15 = "ripple15"
    Wave = "wave"


class TrickKind:
    Bounce = "bounce"
    Jump = "jump"


WAVE_ORDER = ("R1", "R2", "R3", "L3", "L2", "L1")
STANCE_FRACTION = 0.5  # tripod: half the cycle on the ground
_STOP_EPSILON = 1.0  # mm of remaining stroke below which we call it stopped

BOUNCE_AMP_MM = 8.0
BOUNCE_HZ = 2.5
JUMP_CROUCH_MM = 18.0
JUMP_AIR_MM = 28.0
JUMP_CROUCH_S = 0.16
JUMP_PUSH_S = 0.10
JUMP_LAND_S = 0.18


@dataclass(frozen=True)
class WalkSpec:
    """One cyclic walk. `groups` swing in order; `stance` is the down duty."""

    kind: str
    chica: str
    stance: float
    groups: Tuple[Group, ...]

    @property
    def offsets(self) -> Dict[str, float]:
        n = len(self.groups)
        return {name: i / n for i, group in enumerate(self.groups) for name in group}


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


def _pairs(left: str, right: str) -> Group:
    return (left, right)


def walks_for(tripod_groups: List[List[str]]) -> Dict[str, WalkSpec]:
    """Build the walk catalog from the robot's tripod grouping.

    A new walk is another WalkSpec in this table. The stepper does not change.
    """
    a, b = (tuple(group) for group in tripod_groups)
    ripple = tuple(name for pair in zip(a, b) for name in pair)
    fronts = _pairs("L1", "R1")
    mids = _pairs("L2", "R2")
    rears = _pairs("L3", "R3")
    singles: Callable[[Tuple[str, ...]], Tuple[Group, ...]] = lambda order: tuple((name,) for name in order)

    specs = (
        WalkSpec(GaitKind.Tripod, "walk3", STANCE_FRACTION, (a, b)),
        WalkSpec(GaitKind.Triple, "walk2", 2.0 / 3.0, (fronts, mids, rears)),
        WalkSpec(GaitKind.Triple25, "walk25", 0.6, (fronts, mids, rears)),
        WalkSpec(GaitKind.Ripple, "walk1", 5.0 / 6.0, singles(ripple)),
        WalkSpec(GaitKind.Ripple15, "walk15", 0.75, singles(ripple)),
        WalkSpec(GaitKind.Wave, "walkwave", 5.0 / 6.0, singles(WAVE_ORDER)),
    )
    return {spec.kind: spec for spec in specs}


WALK_KINDS = (
    GaitKind.Tripod,
    GaitKind.Triple,
    GaitKind.Triple25,
    GaitKind.Ripple,
    GaitKind.Ripple15,
    GaitKind.Wave,
)
GAIT_KINDS = WALK_KINDS  # older import name


def walk_catalog(tripod_groups: List[List[str]]) -> List[Dict[str, str]]:
    """Ids and Chica names for the client. Order is WALK_KINDS."""
    walks = walks_for(tripod_groups)
    return [{"id": kind, "chica": walks[kind].chica} for kind in WALK_KINDS]


@dataclass
class TrickOut:
    """Overlay the stepper applies on top of a (possibly settled) stance."""

    height_adj: float = 0.0
    body_z: float = 0.0
    blocking: bool = False


class Gait:
    def __init__(self, config: Config, pattern: str = GaitKind.Tripod):
        self.config = config
        self.groups: List[List[str]] = config.tripod_groups
        self.walks = walks_for(config.tripod_groups)
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False
        self.trick: Optional[str] = None
        self._bounce_t = 0.0
        self._jump_t = 0.0
        self._body_z = 0.0
        self.set_pattern(pattern)

    def set_pattern(self, pattern: str) -> None:
        spec = self.walks.get(pattern)
        if spec is None:
            spec = next((row for row in self.walks.values() if row.chica == pattern), None)
        if spec is None:
            raise ValueError(f"unknown gait pattern {pattern!r}")
        self.pattern = spec.kind
        self.spec = spec
        self.stance_fraction = spec.stance
        self._offset = spec.offsets
        self.phase = 0.0

    def reset(self) -> None:
        self.phase = 0.0
        self.velocity = Velocity()
        self.walking = False
        self.trick = None
        self._bounce_t = 0.0
        self._jump_t = 0.0

    def start_jump(self) -> None:
        self.trick = TrickKind.Jump
        self._jump_t = 0.0
        self.walking = False
        self.velocity = Velocity()
        self.phase = 0.0

    def set_bounce(self, enabled: bool) -> None:
        if enabled:
            self.trick = TrickKind.Bounce
            self._bounce_t = 0.0
            return
        if self.trick == TrickKind.Bounce:
            self.trick = None

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
        overlay = self._advance_trick(dt)
        if overlay.blocking:
            command = Velocity()

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
        ride = height + overlay.height_adj
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
            feet[name] = (nx + sx * along, ny + sy * along, -ride + lift)
        return feet

    def pose_overlay(self, pose: BodyPose) -> BodyPose:
        """Body-frame hop. Bounce and jump lift the chassis without retargeting feet."""
        if self.trick is None:
            return pose
        return replace(pose, z=pose.z + self._body_z)

    def group_in_swing(self) -> int:
        return 0 if self.phase < (1.0 - self.stance_fraction) else 1

    def _advance_trick(self, dt: float) -> TrickOut:
        self._body_z = 0.0
        if self.trick == TrickKind.Bounce:
            self._bounce_t += dt
            self._body_z = BOUNCE_AMP_MM * math.sin(2.0 * math.pi * BOUNCE_HZ * self._bounce_t)
            return TrickOut(body_z=self._body_z, blocking=False)

        if self.trick != TrickKind.Jump:
            return TrickOut()

        self._jump_t += dt
        t = self._jump_t
        crouch_end = JUMP_CROUCH_S
        push_end = crouch_end + JUMP_PUSH_S
        land_end = push_end + JUMP_LAND_S

        if t < crouch_end:
            u = t / crouch_end
            return TrickOut(height_adj=-JUMP_CROUCH_MM * u, blocking=True)
        if t < push_end:
            u = (t - crouch_end) / JUMP_PUSH_S
            self._body_z = JUMP_AIR_MM * u
            return TrickOut(height_adj=-JUMP_CROUCH_MM * (1.0 - u), body_z=self._body_z, blocking=True)
        if t < land_end:
            u = (t - push_end) / JUMP_LAND_S
            self._body_z = JUMP_AIR_MM * (1.0 - u)
            return TrickOut(body_z=self._body_z, blocking=True)

        self.trick = None
        self._jump_t = 0.0
        return TrickOut()


TripodGait = Gait
