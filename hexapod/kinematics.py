"""Body pose and 3-DOF leg kinematics.

Frames
------
body   +X right, +Y forward, +Z up. Origin at the centre of the plane where the
       legs bolt on. Standing feet sit at z = -ride_height.
leg    origin at the coxa rotation centre, rotated by the leg's mount yaw so +x
       points straight out along the leg. Coxa turns about +z; femur and tibia
       turn in the leg's vertical plane.

Joint angles, all degrees
-------------------------
coxa   0 = foot straight out along the leg's mount direction, + = toward +y.
femur  elevation of the femur above the leg's horizontal, + = up.
tibia  fold at the knee, 0 = tibia in line with the femur, + = folded under.

`ServoCal.direction` and the attach angles turn joint angles into servo angles.
The signs cannot be confirmed without the robot; check them with `hexapod jog`
one joint at a time before you let it walk.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Tuple

from .config import JOINTS, Config, Geometry, LegConfig, Limits

Vec3 = Tuple[float, float, float]


class UnreachableFoot(ValueError):
    """Foot target outside the leg's workspace."""


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


@dataclass(frozen=True)
class JointAngles:
    coxa: float
    femur: float
    tibia: float

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.coxa, self.femur, self.tibia)


@dataclass(frozen=True)
class BodyPose:
    """Body offset from its neutral pose. Rotations in degrees, applied Z-Y-X."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def foot_to_body(self, point: Vec3) -> Vec3:
        """Map a foot position from the ground frame into the (moved) body frame."""
        px, py, pz = point[0] - self.x, point[1] - self.y, point[2] - self.z
        # Inverse of Rz(yaw) @ Ry(pitch) @ Rx(roll), i.e. transpose applied in reverse.
        cy, sy = _cos_sin(self.yaw)
        cp, sp = _cos_sin(self.pitch)
        cr, sr = _cos_sin(self.roll)
        # Rz^T
        ax, ay, az = cy * px + sy * py, -sy * px + cy * py, pz
        # Ry^T
        bx, by, bz = cp * ax - sp * az, ay, sp * ax + cp * az
        # Rx^T
        return (bx, cr * by + sr * bz, -sr * by + cr * bz)


def _cos_sin(deg: float) -> Tuple[float, float]:
    rad = math.radians(deg)
    return math.cos(rad), math.sin(rad)


class LegKinematics:
    """Kinematics for one leg, plus the joint-angle-to-pulse-width mapping."""

    def __init__(self, leg: LegConfig, geometry: Geometry, limits: Limits):
        self.leg = leg
        self.geometry = geometry
        self.limits = limits
        self._cos_yaw, self._sin_yaw = _cos_sin(leg.yaw_deg)
        self.origin: Vec3 = (leg.coxa_xy[0], leg.coxa_xy[1], geometry.leg_connection_z)

    # --- frame conversion ---------------------------------------------------------

    def body_to_leg(self, point: Vec3) -> Vec3:
        dx = point[0] - self.origin[0]
        dy = point[1] - self.origin[1]
        dz = point[2] - self.origin[2]
        return (self._cos_yaw * dx + self._sin_yaw * dy, -self._sin_yaw * dx + self._cos_yaw * dy, dz)

    def leg_to_body(self, point: Vec3) -> Vec3:
        x, y, z = point
        return (
            self.origin[0] + self._cos_yaw * x - self._sin_yaw * y,
            self.origin[1] + self._sin_yaw * x + self._cos_yaw * y,
            self.origin[2] + z,
        )

    # --- kinematics ---------------------------------------------------------------

    def ik(self, foot_leg: Vec3, strict: bool = False) -> JointAngles:
        """Foot position in the leg frame -> joint angles.

        With `strict=False` an out-of-reach target is pulled onto the workspace
        boundary instead of raising, so a bad joystick input bends the leg to its
        limit rather than dropping the control frame.
        """
        g = self.geometry
        x, y, z = foot_leg
        coxa = math.degrees(math.atan2(y, x))

        radial = math.hypot(x, y) - g.coxa_len
        reach = math.hypot(radial, z)
        lo = abs(g.femur_len - g.tibia_len) + 1e-3
        hi = g.femur_len + g.tibia_len - 1e-3
        if reach < lo or reach > hi:
            if strict:
                raise UnreachableFoot(f"{self.leg.name}: reach {reach:.1f}mm outside [{lo:.1f}, {hi:.1f}]")
            reach = _clamp(reach, lo, hi)

        cos_knee = (g.femur_len**2 + g.tibia_len**2 - reach**2) / (2 * g.femur_len * g.tibia_len)
        knee_interior = math.degrees(math.acos(_clamp(cos_knee, -1.0, 1.0)))
        tibia = 180.0 - knee_interior

        cos_beta = (g.femur_len**2 + reach**2 - g.tibia_len**2) / (2 * g.femur_len * reach)
        beta = math.degrees(math.acos(_clamp(cos_beta, -1.0, 1.0)))
        alpha = math.degrees(math.atan2(z, radial))
        femur = alpha + beta

        return JointAngles(coxa=coxa, femur=femur, tibia=tibia)

    def fk(self, angles: JointAngles) -> Vec3:
        """Joint angles -> foot position in the leg frame. Used to check `ik`."""
        g = self.geometry
        femur_rad = math.radians(angles.femur)
        tibia_rad = math.radians(angles.femur - angles.tibia)
        radial = g.coxa_len + g.femur_len * math.cos(femur_rad) + g.tibia_len * math.cos(tibia_rad)
        z = g.femur_len * math.sin(femur_rad) + g.tibia_len * math.sin(tibia_rad)
        coxa_rad = math.radians(angles.coxa)
        return (radial * math.cos(coxa_rad), radial * math.sin(coxa_rad), z)

    # --- servo mapping ------------------------------------------------------------

    def clamp_angles(self, angles: JointAngles) -> JointAngles:
        return JointAngles(
            *[_clamp(value, *self.limits.joint_range(joint)) for joint, value in zip(JOINTS, angles.as_tuple())]
        )

    def servo_angle(self, joint: str, joint_angle_deg: float) -> float:
        cal = self.leg.servos[joint]
        return cal.direction * (joint_angle_deg - self.geometry.attach_angle(joint))

    def pulses(self, angles: JointAngles) -> Dict[int, int]:
        """Joint angles -> {channel: microseconds}, clamped to the configured range."""
        low, high = self.limits.pulse_us
        out: Dict[int, int] = {}
        for joint, value in zip(JOINTS, angles.as_tuple()):
            cal = self.leg.servos[joint]
            pulse = cal.pulse_for(self.servo_angle(joint, value))
            out[cal.channel] = int(round(_clamp(pulse, low, high)))
        return out


class HexapodKinematics:
    """All six legs, plus the body-pose step in front of them."""

    def __init__(self, config: Config):
        self.config = config
        self.legs: Dict[str, LegKinematics] = {
            name: LegKinematics(leg, config.geometry, config.limits) for name, leg in config.legs.items()
        }
        self.order: List[str] = list(config.leg_order)

    def neutral_feet(self, height: float = None) -> Dict[str, Vec3]:
        """Standing foot positions in the ground frame."""
        z = -(self.config.stance.ride_height if height is None else height)
        return {name: (leg.neutral_xy[0], leg.neutral_xy[1], z) for name, leg in self.config.legs.items()}

    def solve(self, feet: Dict[str, Vec3], pose: BodyPose = BodyPose()) -> Dict[str, JointAngles]:
        """Ground-frame foot targets + body pose -> clamped joint angles per leg."""
        out: Dict[str, JointAngles] = {}
        for name, target in feet.items():
            leg = self.legs[name]
            in_body = pose.foot_to_body(target)
            out[name] = leg.clamp_angles(leg.ik(leg.body_to_leg(in_body)))
        return out

    def pulse_frame(self, angles: Dict[str, JointAngles]) -> List[int]:
        """Joint angles -> the 18 pulse widths, indexed by servo channel 0..17."""
        frame = [1500] * 18
        for name, value in angles.items():
            for channel, pulse in self.legs[name].pulses(value).items():
                frame[channel] = pulse
        return frame
