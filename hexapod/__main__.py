"""Command line entry points: bring-up tools and the server."""

from __future__ import annotations

import argparse
import logging
import math
import sys
import time
from typing import List, Optional

from . import config as config_mod
from .board import FakeBoard, Servo2040
from .config import JOINTS, Config
from .controller import POSE_LIMITS, Controller
from .gait import TripodGait, Velocity
from .kinematics import BodyPose, HexapodKinematics
from .net import interface_addresses, is_tunnel


def _board(config: Config, args: argparse.Namespace) -> Servo2040:
    cls = FakeBoard if args.dry_run else Servo2040
    return cls(config, port=args.serial)


def _load(args: argparse.Namespace) -> Config:
    config = config_mod.load(args.config)
    if args.serial:
        object.__setattr__(config, "port", args.serial)
    return config


# --- commands ---------------------------------------------------------------------


def cmd_ports(args: argparse.Namespace) -> int:
    from serial.tools import list_ports

    found = list(list_ports.comports())
    if not found:
        print("no serial ports found")
        return 1
    for port in found:
        mark = "  <- looks like an RP2040" if (port.vid, port.pid) == (0x2E8A, 0x000A) else ""
        print(f"{port.device:24} {port.description}{mark}")
    return 0


def cmd_check(args: argparse.Namespace) -> int:
    """Offline: sweep the command envelope and report joint travel and clamping.

    Run this after editing limits or geometry, before powering the servos.
    """
    config = _load(args)
    kinematics = HexapodKinematics(config)
    limits = config.limits

    print("servo pulse range vs joint limits")
    worst = False
    for joint in JOINTS:
        low, high = limits.joint_range(joint)
        attach = config.geometry.attach_angle(joint)
        for name, leg in config.legs.items():
            cal = leg.servos[joint]
            for angle in (low, high):
                pulse = cal.pulse_for(cal.direction * (angle - attach))
                if not limits.pulse_us[0] <= pulse <= limits.pulse_us[1]:
                    print(f"  ! {name}.{joint} at {angle:+.0f} deg wants {pulse:.0f}us, outside pulse_us")
                    worst = True
    print("  ok" if not worst else "  joint limits reach past pulse_us; the pulse clamp will win")

    commands = [Velocity(1, 0, 0), Velocity(0, 1, 0), Velocity(0, -1, 0), Velocity(-1, 0, 0),
                Velocity(0, 0, 1), Velocity(0, 0, -1), Velocity(0.7, 0.7, 1)]
    shift, roll, pitch, yaw = (POSE_LIMITS[k] for k in ("shift", "roll", "pitch", "yaw"))
    poses = {
        "flat": BodyPose(),
        "tilt": BodyPose(roll=roll, pitch=pitch),
        "tilt-": BodyPose(roll=-roll, pitch=-pitch),
        "shift": BodyPose(x=shift, y=shift, yaw=yaw),
        "shift-": BodyPose(x=-shift, y=-shift, yaw=-yaw),
    }
    heights = sorted({config.stance.sit_height, 45.0, 50.0, 55.0, 60.0, 65.0,
                      70.0, 75.0, config.stance.ride_height, 90.0, 105.0})
    travel = {joint: [1e9, -1e9] for joint in JOINTS}
    clamped = {}
    for velocity in commands:
        for height in heights:
            for pose_name, pose in poses.items():
                gait = TripodGait(config)
                scaled = velocity.scaled(config)
                hits = total = 0
                for _ in range(int(3 * config.rate_hz)):
                    feet = gait.step(1 / config.rate_hz, scaled, height)
                    _, limited = kinematics.solve_reporting(feet, pose)
                    total += 1
                    hits += 1 if limited else 0
                    for name, leg in kinematics.legs.items():
                        raw = leg.ik(leg.body_to_leg(pose.foot_to_body(feet[name])))
                        for joint, value in zip(JOINTS, raw.as_tuple()):
                            travel[joint][0] = min(travel[joint][0], value)
                            travel[joint][1] = max(travel[joint][1], value)
                key = (round(height), pose_name)
                clamped[key] = (clamped.get(key, (0, 0))[0] + hits, clamped.get(key, (0, 0))[1] + total)

    print("\njoint travel over the full command envelope")
    for joint in JOINTS:
        low, high = limits.joint_range(joint)
        flag = "" if travel[joint][0] >= low and travel[joint][1] <= high else "   <- exceeds limit"
        print(f"  {joint:6} {travel[joint][0]:7.1f} .. {travel[joint][1]:7.1f} deg   limit {low:.0f} .. {high:.0f}{flag}")

    print("\nframes needing a clamp")
    for (height, pose_name) in sorted(clamped):
        hits, total = clamped[(height, pose_name)]
        note = "" if not hits else f"   <- {100 * hits / total:.0f}% of frames"
        print(f"  height {height:3d}mm  {pose_name:5}  {hits:5}/{total}{note}")
    return 0


def cmd_neutral(args: argparse.Namespace) -> int:
    """Torque on and hold the neutral stance. The first thing to run on hardware."""
    config = _load(args)
    kinematics = HexapodKinematics(config)
    height = args.height if args.height is not None else config.stance.sit_height
    frame = kinematics.pulse_frame(kinematics.solve(kinematics.neutral_feet(height)))
    print(f"neutral stance at {height:.0f}mm; pulse widths: {frame}")
    if args.print_only:
        return 0

    board = _board(config, args)
    with board:
        board.set_frame(frame)
        board.set_torque(True)
        print("torque on, Ctrl-C to release")
        try:
            while True:
                board.set_frame(frame)  # keeps feeding the watchdog
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nreleasing")
    return 0


def cmd_jog(args: argparse.Namespace) -> int:
    """Drive one joint at a time, in degrees. Use it to fix directions and attach angles.

    Commands:  L1 femur 20     set a joint
               L1 0 0 0        set all three joints of a leg
               all             back to neutral
               t on | t off    torque
               q               quit
    """
    config = _load(args)
    kinematics = HexapodKinematics(config)
    angles = kinematics.solve(kinematics.neutral_feet(config.stance.sit_height))
    board = _board(config, args)

    with board:
        board.set_frame(kinematics.pulse_frame(angles))
        board.set_torque(True)
        print(cmd_jog.__doc__)
        stop = False
        import threading

        def feed() -> None:
            while not stop:
                board.set_frame(kinematics.pulse_frame(angles))
                time.sleep(0.05)

        pump = threading.Thread(target=feed, daemon=True)
        pump.start()
        try:
            for line in sys.stdin:
                parts = line.split()
                if not parts:
                    continue
                if parts[0] in ("q", "quit", "exit"):
                    break
                if parts[0] == "t":
                    board.set_torque(len(parts) > 1 and parts[1] == "on")
                    continue
                if parts[0] == "all":
                    angles = kinematics.solve(kinematics.neutral_feet(config.stance.sit_height))
                    continue
                leg = parts[0].upper()
                if leg not in kinematics.legs:
                    print(f"unknown leg {leg}; expected one of {', '.join(kinematics.order)}")
                    continue
                from .kinematics import JointAngles

                try:
                    if len(parts) == 3:
                        joint = parts[1].lower()
                        current = dict(zip(JOINTS, angles[leg].as_tuple()))
                        current[joint] = float(parts[2])
                        angles[leg] = JointAngles(**current)
                    elif len(parts) == 4:
                        angles[leg] = JointAngles(*[float(v) for v in parts[1:]])
                    else:
                        print("expected: <leg> <joint> <deg>  or  <leg> <coxa> <femur> <tibia>")
                        continue
                except (KeyError, TypeError, ValueError) as exc:
                    print(f"bad command: {exc}")
                    continue
                clamped = kinematics.legs[leg].clamp_angles(angles[leg])
                if clamped.as_tuple() != angles[leg].as_tuple():
                    print("  clamped to a joint limit")
                    angles[leg] = clamped
                shown = ", ".join(f"{joint} {value:.1f}" for joint, value in zip(JOINTS, angles[leg].as_tuple()))
                pulses = kinematics.legs[leg].pulses(angles[leg])
                print(f"  {leg}  {shown}   pulses {[pulses[c] for c in sorted(pulses)]}")
        except KeyboardInterrupt:
            pass
        finally:
            stop = True
            pump.join(timeout=0.5)
    return 0


def cmd_telemetry(args: argparse.Namespace) -> int:
    """Stream volts, amps and foot contacts. Does not enable torque."""
    config = _load(args)
    board = _board(config, args)
    with board:
        print("reading; Ctrl-C to stop")
        try:
            while True:
                telemetry = board.telemetry
                contacts = " ".join(f"{n}:{'down' if v else '  up'}" for n, v in sorted(telemetry.contacts.items()))
                volts = f"{telemetry.volts:5.2f}V" if telemetry.volts is not None else "  --V"
                amps = f"{telemetry.amps:6.2f}A" if telemetry.amps is not None else "   --A"
                print(f"\r{volts} {amps}  {contacts}   age {telemetry.age_s:4.1f}s", end="", flush=True)
                time.sleep(0.2)
        except KeyboardInterrupt:
            print()
    return 0


def cmd_serve(args: argparse.Namespace) -> int:
    import uvicorn

    from .camera import Camera, FakeCamera
    from .imu import FakeImu, Imu
    from .server import create_app
    from .state import RobotState

    config = _load(args)
    board = _board(config, args)
    board.open()
    controller = Controller(config, board)
    controller.start()

    # This process owns every device. The state object opens the sensors itself so
    # a missing one shows up on the status page instead of stopping the server.
    imu = None
    if config.imu.enabled and not args.no_imu:
        imu = (FakeImu if args.dry_run else Imu)(config.imu)
    camera = None
    if config.camera.enabled and not args.no_camera:
        camera = (FakeCamera if args.dry_run else Camera)(config.camera)
    state = RobotState(config, controller, imu=imu, camera=camera)
    state.start()
    app = create_app(state, config)

    print(f"\n  {'dry run, no serial port' if args.dry_run else 'board on ' + config.port}")
    imu_note = "off" if imu is None else "fake" if args.dry_run else f"i2c-{config.imu.bus} {config.imu.address:#04x}"
    camera_note = "off" if camera is None else "fake" if args.dry_run else (
        f"{config.camera.lores[0]}x{config.camera.lores[1]} live, "
        f"{config.camera.still[0]}x{config.camera.still[1]} stills")
    print(f"  imu {imu_note}; camera {camera_note}")
    _print_urls(args.port)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning", ws_ping_interval=5)
    finally:
        state.stop()
        controller.stop()
        board.close()
    return 0


def _print_urls(port: int) -> None:
    """List every address the pages are reachable at, best guess first."""
    addresses = interface_addresses()
    if not addresses:
        print(f"\n  hexapod control on http://localhost:{port}  (status panel at /status)\n", flush=True)
        return
    rows = [(f"http://{address}:{port}", name, "VPN or virtual, probably not your LAN" if is_tunnel(name) else "")
            for name, address in addresses]
    rows.append((f"http://localhost:{port}", "", "this machine only"))
    width = max(len(url) for url, _, _ in rows)
    print("\n  hexapod control on:")
    for url, name, note in rows:
        print(f"    {url:<{width}}  {name:<8} {note}".rstrip())
    print(f"  status panel at /status on any of them", flush=True)


# --- argument parsing -------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="hexapod", description=__doc__)
    parser.add_argument("--config", default=config_mod.DEFAULT_CONFIG, help="path to hexapod.yaml")
    parser.add_argument("--serial", default=None, help="override the serial port")
    parser.add_argument("--dry-run", action="store_true", help="no hardware; run the stack against a fake board")
    parser.add_argument("-v", "--verbose", action="store_true")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ports", help="list serial ports").set_defaults(func=cmd_ports)
    subparsers.add_parser("check", help="offline kinematics and limit check").set_defaults(func=cmd_check)
    subparsers.add_parser("telemetry", help="stream volts, amps and foot contacts").set_defaults(func=cmd_telemetry)
    subparsers.add_parser("jog", help="drive individual joints during bring-up").set_defaults(func=cmd_jog)

    neutral = subparsers.add_parser("neutral", help="hold the neutral stance")
    neutral.add_argument("--height", type=float, default=None, help="ride height in mm")
    neutral.add_argument("--print-only", action="store_true", help="print the pulse widths and exit")
    neutral.set_defaults(func=cmd_neutral)

    serve = subparsers.add_parser("serve", help="run the web interface")
    serve.add_argument("--host", default="0.0.0.0", help="bind address")
    serve.add_argument("--port", type=int, default=8000)
    serve.add_argument("--no-camera", action="store_true", help="do not open the camera")
    serve.add_argument("--no-imu", action="store_true", help="do not open the IMU")
    serve.set_defaults(func=cmd_serve)

    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
