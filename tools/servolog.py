#!/usr/bin/env python3
"""Watch every servo while you press buttons, and say what moved.

    python3 tools/servolog.py                       # server on localhost:8000
    python3 tools/servolog.py --host hexapod.local
    python3 tools/servolog.py --csv stand.csv       # every sample to a file too
    python3 tools/servolog.py --seconds 10 --quiet  # summary only, then exit

Polls /api/state and prints one line per channel each time a pulse width
changes: which leg and joint, which SERVO header, the joint angle the solver
asked for and the pulse that went out. A pause in the motion prints a blank
line, so each button press reads as a block. Ctrl-C prints a summary: where
every channel started and ended, how far it travelled, and anything that
looks off.

It cannot tell whether a servo turned the right way; only you can see that.
It tells you exactly which servo moved and by how much, so "SERVO 2 went
1500 -> 1622 and the knee went up" is a sentence you can write down.

Nothing here imports the hexapod package. Standard library only, so it runs
from the Pi or from a laptop on the same network.
"""

import argparse
import csv
import json
import ssl
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

JOINTS = ("coxa", "femur", "tibia")
QUIET_GAP_S = 0.5  # no change for this long ends a block


def fetch(url: str, timeout: float = 1.0):
    ctx = ssl._create_unverified_context() if url.startswith("https:") else None
    with urllib.request.urlopen(url, timeout=timeout, context=ctx) as response:
        return json.load(response)


def channel_names(config: dict) -> Dict[int, Tuple[str, str, int]]:
    """channel -> (leg, joint, direction). Empty on a server without `servos`."""
    names: Dict[int, Tuple[str, str, int]] = {}
    for leg, joints in (config.get("servos") or {}).items():
        for joint, servo in joints.items():
            names[int(servo["channel"])] = (leg, joint, int(servo.get("direction", 1)))
    return names


def label(channel: int, names: Dict[int, Tuple[str, str, int]]) -> str:
    """`R3 femur` if the server told us, else the bare channel."""
    if channel in names:
        leg, joint, _ = names[channel]
        return f"{leg} {joint}"
    return f"ch {channel}"


def changed(prev: Sequence[int], cur: Sequence[int]) -> List[int]:
    return [i for i, (a, b) in enumerate(zip(prev, cur)) if a != b]


def joint_angle(state: dict, channel: int, names: Dict[int, Tuple[str, str, int]]) -> Optional[float]:
    if channel not in names:
        return None
    leg, joint, _ = names[channel]
    return (state.get("angles") or {}).get(leg, {}).get(joint)


def change_line(when: float, channel: int, before: dict, after: dict,
                names: Dict[int, Tuple[str, str, int]]) -> str:
    stamp = datetime.fromtimestamp(when).strftime("%H:%M:%S.%f")[:-3]
    a0, a1 = joint_angle(before, channel, names), joint_angle(after, channel, names)
    angle = f"{a0:6.1f} -> {a1:6.1f} deg" if a0 is not None and a1 is not None else " " * 21
    p0, p1 = before["pulses"][channel], after["pulses"][channel]
    return (f"{stamp}  {label(channel, names):<9} SERVO {channel + 1:>2} (ch {channel:>2})  "
            f"{angle}   {p0:>4} -> {p1:>4} us ({p1 - p0:+d})")


class Track:
    """What one channel did over the whole capture."""

    def __init__(self, pulse: int, angle: Optional[float]) -> None:
        self.first = self.last = self.low = self.high = pulse
        self.angle_first = self.angle_last = angle
        self.moves = 0

    def update(self, pulse: int, angle: Optional[float]) -> None:
        if pulse != self.last:
            self.moves += 1
        self.last = pulse
        self.low, self.high = min(self.low, pulse), max(self.high, pulse)
        if angle is not None:
            self.angle_last = angle


def summarize(tracks: Dict[int, Track], names: Dict[int, Tuple[str, str, int]],
              pulse_us: Optional[Sequence[int]]) -> List[str]:
    lines = [f"{'channel':<8}{'servo':<7}{'leg joint':<11}{'dir':<5}{'start':>6}{'end':>6}{'net':>6}"
             f"{'range':>12}   joint start -> end"]
    for channel in sorted(tracks):
        t = tracks[channel]
        leg_joint = label(channel, names)
        direction = f"{names[channel][2]:+d}" if channel in names else ""
        if t.angle_first is not None and t.angle_last is not None:
            angle = f"{t.angle_first:6.1f} -> {t.angle_last:6.1f} ({t.angle_last - t.angle_first:+.1f})"
        else:
            angle = ""
        lines.append(f"ch {channel:<5}{channel + 1:<7}{leg_joint:<11}{direction:<5}{t.first:>6}{t.last:>6}"
                     f"{t.last - t.first:>+6}{t.low:>6}-{t.high:<5}   {angle}")

    still = [label(c, names) for c in sorted(tracks) if tracks[c].moves == 0]
    if still:
        lines.append("")
        lines.append("never moved: " + ", ".join(still))

    if pulse_us:
        low, high = pulse_us
        clamped = [f"{label(c, names)} at {tracks[c].low if tracks[c].low <= low else tracks[c].high}"
                   for c in sorted(tracks) if tracks[c].low <= low or tracks[c].high >= high]
        if clamped:
            lines.append("")
            lines.append(f"hit the pulse clamp {list(pulse_us)}: " + ", ".join(clamped))

    # Stand and Sit ask every leg for the same joint change. A leg that differs
    # was limited, or is not the leg its config says it is.
    by_joint: Dict[str, Dict[str, float]] = {}
    for channel, (leg, joint, _) in names.items():
        t = tracks.get(channel)
        if t and t.angle_first is not None and t.angle_last is not None:
            by_joint.setdefault(joint, {})[leg] = t.angle_last - t.angle_first
    uneven = []
    for joint in JOINTS:
        deltas = by_joint.get(joint, {})
        if len(deltas) > 1 and max(deltas.values()) - min(deltas.values()) > 1.0:
            uneven.append(f"{joint}: " + ", ".join(f"{leg} {d:+.1f}" for leg, d in sorted(deltas.items())))
    if uneven:
        lines.append("")
        lines.append("joint change differs between legs (fine when walking, not for Stand or Sit):")
        lines.extend("  " + line for line in uneven)
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--hz", type=float, default=20.0, help="poll rate; the control loop runs at 50")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this long; 0 = until Ctrl-C")
    parser.add_argument("--csv", help="also write every sample here")
    parser.add_argument("--quiet", action="store_true", help="no live lines, only the summary")
    args = parser.parse_args()
    base = f"https://{args.host}:{args.port}"

    try:
        config = fetch(f"{base}/api/config")
        state = fetch(f"{base}/api/state")
    except (urllib.error.URLError, OSError) as exc:
        print(f"cannot reach {base}: {exc}")
        return 1
    if "pulses" not in state:
        print("this server does not report pulses; update it")
        return 1

    names = channel_names(config)
    pulse_us = config.get("pulse_us")
    tracks = {c: Track(p, joint_angle(state, c, names)) for c, p in enumerate(state["pulses"])}

    writer = None
    if args.csv:
        handle = open(args.csv, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(["t"] + [f"ch{c}" for c in range(len(state["pulses"]))]
                        + [f"{leg}_{joint}" for leg in sorted(state.get("angles", {})) for joint in JOINTS])

    def row(when: float, s: dict) -> list:
        return ([f"{when:.3f}"] + list(s["pulses"])
                + [s["angles"][leg][joint] for leg in sorted(s.get("angles", {})) for joint in JOINTS])

    print(f"watching {base} at {args.hz:g} Hz, {len(names) or len(state['pulses'])} channels named. "
          f"Press the buttons; Ctrl-C for the summary.")
    period = 1.0 / args.hz
    started = last_change = time.time()
    in_block = False
    if writer:
        writer.writerow(row(started, state))
    try:
        while not args.seconds or time.time() - started < args.seconds:
            time.sleep(period)
            try:
                cur = fetch(f"{base}/api/state")
            except (urllib.error.URLError, OSError) as exc:
                print(f"lost the server ({exc}); retrying")
                time.sleep(1.0)
                continue
            now = time.time()
            if writer:
                writer.writerow(row(now, cur))
            moved = changed(state["pulses"], cur["pulses"])
            if moved:
                if not args.quiet:
                    for channel in moved:
                        print(change_line(now, channel, state, cur, names))
                in_block, last_change = True, now
            elif in_block and now - last_change > QUIET_GAP_S:
                if not args.quiet:
                    print()
                in_block = False
            for channel, pulse in enumerate(cur["pulses"]):
                tracks[channel].update(pulse, joint_angle(cur, channel, names))
            state = cur
    except KeyboardInterrupt:
        pass
    finally:
        if writer:
            handle.close()

    print()
    print(f"--- {time.time() - started:.1f} s ---")
    print("\n".join(summarize(tracks, names, pulse_us)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
