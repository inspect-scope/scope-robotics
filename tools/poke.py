#!/usr/bin/env python3
"""Standalone check that the Servo2040 link works. Only needs pyserial.

    python3 tools/poke.py /dev/ttyACM0            # read volts/amps, no torque
    python3 tools/poke.py /dev/ttyACM0 --centre   # torque on, every servo to 1500us
    python3 tools/poke.py /dev/ttyACM0 --census   # wiggle one header at a time, you name what moved
    python3 tools/poke.py /dev/ttyACM0 --probe    # wiggle each header, report which ones draw current

The census builds the header -> joint map with the config out of the loop. Each
header wiggles in turn; you type which leg and joint moved, by position, and it
prints the `servos:` block for hexapod.yaml at the end. Legs off the ground.

The probe needs no eyes. It wiggles each header and watches the board's current
sensor: a servo that moves pulls one to five amps, an empty or dead header pulls
nothing. It also reports any foot switch that changed. Torque comes on at the
--base pose (1500 us on all 18 without it), then it stops before wiggling any
header if the robot already pulls more than 0.5 A at rest, because a servo
stalled at rest would hide inside every per-header reading. Run it legs free.
Pass --base with the 18 pulse widths the server last sent (`curl -s
localhost:8000/api/state`, key `pulses`, before you stop it) so nothing jumps.

Every mode drops the relay on its own when the total current's mean passes
10 A for 1 s, or when no current reading arrives for 2 s. --centre also drops
it at 1.5 A once the pose has settled, and the probe when a header is still
pulling 0.8 A over idle a second after it returns.

Nothing here imports the hexapod package, on purpose: if this script talks to the
board then the wiring, the firmware and the port are all fine, and anything that
breaks after this is our code.
"""

import argparse
import signal
import sys
import time
from collections import deque

import serial

SET_CMD, GET_CMD = 0xD3, 0xC7
# The board reports one TOTAL current for all 18 servos. One stalled FT5330M adds
# about 4 A (3.9 A spec at 7.4 V); two read about 8 A on the stand, so the 10 A
# cut needs three simultaneous stalls, or two with other load on top.
CUT_A, CUT_S = 10.0, 1.0
STALE_S = 2.0  # torque on and no full current window for this long: we are blind, cut
# At a static pose with legs free the total sits near 0.2 A; one stalled servo
# lifts it to about 4 A. --centre holds a static pose, so it can cut much lower.
HOLD_CUT_A, HOLD_S, HOLD_SETTLE_S = 1.5, 1.5, 2.0
IDLE_MAX_A = 0.5  # probe baseline, legs free, nothing moving


class CurrentTrip(Exception):
    """The board's total current stayed too high for too long."""


class CurrentWatch:
    """Mean of every total-current read over the last `span` seconds. Use one per
    run, so the window survives the probe's short sample windows; a mean, so one
    low reading cannot reset it. Also the blind check: call arm() at every
    relay-on, and check_blind() raises when no full window has been seen since."""

    def __init__(self, limit=CUT_A, span=CUT_S):
        self.limit, self.span = limit, span
        self.samples = deque()
        self.covered_at = None

    def arm(self, now):
        self.samples.clear()
        self.covered_at = now

    def add(self, amps, now):
        """Record a reading. Returns the window mean if the window is covered and
        over the limit, else None."""
        self.samples.append((now, amps))
        while self.samples[0][0] < now - self.span:
            self.samples.popleft()
        if now - self.samples[0][0] < 0.8 * self.span:
            return None
        self.covered_at = now
        mean = sum(a for _, a in self.samples) / len(self.samples)
        return mean if mean > self.limit else None

    def check_blind(self, now, limit=STALE_S):
        if self.covered_at is not None and now - self.covered_at > limit:
            raise CurrentTrip(f"no current reading for {now - self.covered_at:.1f} s with torque on. The board may"
                              " not have received the relay-off either: if the servos still hold, unplug the"
                              " servo battery")


def _release(port, frame=None):
    """Relay off, and Ctrl-C cannot stop it. With `frame`, the servos go back to
    it first so nothing is left mid-wiggle; after a trip, pass no frame so the
    relay goes first."""
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        if frame is not None:
            try:
                port.write(frame_set(0, frame))
                time.sleep(0.5)
            finally:
                port.write(frame_set(CH_RELAY, [0]))
        else:
            port.write(frame_set(CH_RELAY, [0]))
    finally:
        signal.signal(signal.SIGINT, previous)
        print("\ntorque off")

CH_CURRENT, CH_VOLTAGE, CH_RELAY = 24, 25, 26


def frame_set(start, values):
    out = bytearray((SET_CMD, start, len(values)))
    for value in values:
        out += bytes((value & 0x7F, (value >> 7) & 0x7F))
    return bytes(out)


def read_get(port, start, count, timeout=1.0):
    port.reset_input_buffer()
    port.write(bytes((GET_CMD, start, count)))
    deadline, buf = time.monotonic() + timeout, b""
    while time.monotonic() < deadline:
        buf += port.read(port.in_waiting or 1)
        head = buf.find(bytes((GET_CMD, start, count)))
        if head >= 0 and len(buf) >= head + 3 + 2 * count:
            body = buf[head + 3 : head + 3 + 2 * count]
            return [body[i] | (body[i + 1] << 7) for i in range(0, len(body), 2)]
    raise TimeoutError(f"no reply to GET {start}+{count} (got {buf!r})")


# Physical position -> config name. Front is the camera end; left and right are
# the robot's own, so stand behind it looking the way the camera looks.
POSITIONS = {"FR": "R1", "MR": "R2", "RR": "R3", "FL": "L1", "ML": "L2", "RL": "L3"}
JOINTS = ("coxa", "femur", "tibia")
LEG_ORDER = ("L1", "L2", "L3", "R1", "R2", "R3")


def parse_answer(text):
    """'RR femur' or 'R3 femur' -> ('R3', 'femur'). None if it is not a leg and a joint."""
    parts = text.strip().split()
    if len(parts) != 2:
        return None
    pos, joint = parts[0].upper(), parts[1].lower()
    if joint not in JOINTS:
        return None
    if pos in POSITIONS:
        return POSITIONS[pos], joint
    if pos in POSITIONS.values():
        return pos, joint
    return None


def census_config(answers):
    """{channel: (leg, joint)} -> (lines of a `servos:` block, list of problems).

    Calibration columns are copied as the config has them today, identical for
    every servo. Redo them per servo if that ever changes."""
    by_leg = {}
    for channel, (leg, joint) in sorted(answers.items()):
        by_leg.setdefault(leg, {})[joint] = channel
    lines, problems = ["servos:"], []
    for leg in LEG_ORDER:
        joints = by_leg.get(leg, {})
        missing = [j for j in JOINTS if j not in joints]
        if missing:
            problems.append(f"{leg}: no header moved its {', '.join(missing)}")
            continue
        cal = "us_neg45: 2000, us_pos45: 1000, direction:  1"
        lines.append(f"  {leg}: {{coxa: {{channel: {joints['coxa']:2d}, {cal}}},")
        lines.append(f"       femur: {{channel: {joints['femur']:2d}, {cal}}},")
        lines.append(f"       tibia: {{channel: {joints['tibia']:2d}, {cal}}}}}")
    named = {}
    for channel, key in answers.items():
        named.setdefault(key, []).append(channel)
    for (leg, joint), channels in sorted(named.items()):
        if len(channels) > 1:
            problems.append(f"{leg} {joint} was named for channels {sorted(channels)}; one of those is wrong")
    return lines, problems


def census(port, sweep):
    print("\ncensus: every header wiggles in turn, +-%d us. Legs must be free to move." % sweep)
    print("Name what moved as position + joint, e.g. 'RR femur'. Positions: FL FR ML MR RL RR,")
    print("front = camera end, left/right = the robot's own. Enter = nothing moved, r = again, q = stop.")
    print("Torque drops while you type each answer. Feetech legs hold; a coreless servo's leg droops.")
    print("\nstarting in 3s -- hold the robot")
    time.sleep(3)
    port.write(frame_set(0, [1500] * 18))
    answers, channel = {}, 0
    watch = CurrentWatch()
    try:
        while channel < 18:
            print(f"\nSERVO {channel + 1:>2}  (channel {channel:>2})")
            port.write(frame_set(CH_RELAY, [1]))
            watch.arm(time.monotonic())
            for value in (1500 + sweep, 1500 - sweep, 1500 + sweep, 1500 - sweep, 1500):
                port.write(frame_set(channel, [value]))
                _sample(port, 0.4, watch=watch)
            port.write(frame_set(CH_RELAY, [0]))  # nothing is held while the operator thinks
            reply = input("  what moved? ").strip()
            if reply.lower() == "q":
                break
            if reply.lower() == "r":
                continue
            if reply:
                parsed = parse_answer(reply)
                if parsed is None:
                    print("  did not understand that; 'RR femur' is the shape")
                    continue
                answers[channel] = parsed
            channel += 1
    except KeyboardInterrupt:
        pass
    except CurrentTrip as exc:
        print(f"\nCUT: {exc}. Relay off. Fix the cause before continuing.")
    finally:
        _release(port)

    print("\nheader   channel  moved")
    for ch in range(18):
        what = " ".join(answers[ch]) if ch in answers else ("nothing" if ch < channel else "not tested")
        print(f"SERVO {ch + 1:>2}   ch {ch:>2}    {what}")
    lines, problems = census_config(answers)
    if problems:
        print("\nnot a complete map yet:")
        for problem in problems:
            print("  " + problem)
    if len(lines) > 1:
        print("\nfor config/hexapod.yaml:\n")
        print("\n".join(lines))
    return 0


TOUCH_LEG = {0: "R3", 1: "L3", 2: "R2", 3: "L2", 4: "R1", 5: "L1"}  # channels 18..23, per hexapod.yaml
CH_TOUCH = 18


STALL_SPEC_A = 3.9      # FT5330M locked-rotor current at 7.4 V, per its spec sheet
HIGH_A = STALL_SPEC_A * 1.15   # a peak well above the stall spec suggests a bind or damage.
# Set for the FT5330M: a healthy DS3235 PRO peaks up to 4.4 A, close to this line.
STALLED_EXCESS_A = 0.8  # still pulling this much over idle a second after the move
NOISE_A = 0.15  # one ADC count is 0.08 A; under two counts is noise
MOVED_A = 0.5   # healthy servos rise 0.9 A or more; two counts over a zero idle is not a move


def verdict(delta_amps, settled_excess=0.0):
    """What the current while one header is driven means. `delta_amps` is the
    peak rise over idle during the move; `settled_excess` is the rise still
    present a second after it should have finished."""
    if delta_amps < NOISE_A:
        return "NOTHING drew current"
    if settled_excess >= STALLED_EXCESS_A:
        return "STILL PULLING: stalled or bound"
    if delta_amps >= HIGH_A:
        return "HIGH: above stall spec, bind or damage"
    if delta_amps < MOVED_A:
        return "WEAK: barely drew current"
    return "servo moved"


def fleet_outliers(peaks, factor=1.4):
    """Channels whose peak is well above the pack. Catches a servo that is
    'within spec' but 40% hungrier than its 17 siblings, which is how the
    L1 femur read on all five probes one afternoon before it died."""
    live = sorted(v for v in peaks.values() if v >= MOVED_A)
    if len(live) < 4:
        return []
    median = live[len(live) // 2]
    return sorted(ch for ch, v in peaks.items() if v >= median * factor and v >= MOVED_A)


def _telemetry(port):
    vals = read_get(port, CH_TOUCH, 8)  # 18..23 touch, 24 current, 25 voltage
    return [v / 310.3 for v in vals[:6]], (vals[6] - 512) * 0.0814, vals[7] / 310.3


def _sample(port, seconds, tail=0.0, watch=None):
    """Peak current and any closed foot switch over `seconds`, plus the mean
    current over the final `tail` seconds (0 = not wanted). Every read also goes
    to `watch`, which raises CurrentTrip; pass the same watch for a whole run. A
    lost reply is skipped, not fatal; the board drops the odd one under a 3 A
    servo start."""
    peak, count, closed = -99.0, 0, set()
    end = time.monotonic() + seconds
    late = []
    if watch is None:
        watch = CurrentWatch()
        watch.arm(time.monotonic())
    while time.monotonic() < end:
        try:
            touch, amps, _ = _telemetry(port)
        except TimeoutError:
            watch.check_blind(time.monotonic())
            continue
        now = time.monotonic()
        mean = watch.add(amps, now)
        if mean is not None:
            raise CurrentTrip(f"{mean:.1f} A mean over {watch.span:.1f} s")
        watch.check_blind(now)
        peak, count = max(peak, amps), count + 1
        closed |= {i for i, v in enumerate(touch) if v > 1.6}
        if tail and now > end - tail:
            late.append(amps)
    settled = sum(late) / len(late) if late else float("nan")
    return peak, count, closed, settled


def probe(port, base, sweep):
    frame = list(base)
    quiet, weak, bad, peaks = [], [], [], {}
    ch = None
    watch = CurrentWatch()
    done = tripped = False
    try:
        # Everything that turns the relay on sits inside the try, so any failure
        # on the way, a lost reply included, still opens it in the finally.
        port.write(frame_set(0, frame))
        port.write(frame_set(CH_RELAY, [1]))
        watch.arm(time.monotonic())
        time.sleep(0.8)
        touch, amps, volts = _telemetry(port)
        print(f"battery {volts:.2f} V   idle {amps:+.2f} A   touch pins " + " ".join(f"{v:.2f}" for v in touch))
        print(f"A servo that moves peaks {MOVED_A} A or more over idle; healthy FT5330Ms have read 0.9 to 3.7 A"
              f" and settle within a second. Stall spec {STALL_SPEC_A} A.")
        idle_peak, count, closed, idle_mean = _sample(port, 1.5, tail=1.0, watch=watch)
        print(f"idle peak {idle_peak:+.2f} A, mean {idle_mean:+.2f} A over {count} samples; foot switches closed: "
              f"{sorted(TOUCH_LEG[i] for i in closed) or 'none'}")
        if not idle_mean <= IDLE_MAX_A:  # written this way so NaN, no reading at all, also stops
            what = "no idle reading" if idle_mean != idle_mean else f"{idle_mean:.2f} A at rest with nothing moving"
            print(f"\nSTOP: {what}; legs free reads about 0.2 A. Something is already pulling: an arm against a"
                  " stop, a failed servo, or feet on the ground. A servo stalled at rest would hide inside every"
                  " per-header reading, so nothing was wiggled. Torque off, check each joint by hand, run again.")
            return 1
        print(f"\n{'header':>8} {'ch':>3} {'from us':>8} {'peak A':>7} {'delta':>6} {'settled':>8}  verdict                          foot switch")
        for ch in range(18):
            peak, seen = -99.0, set()
            for target in (frame[ch] + sweep, frame[ch], frame[ch] - sweep):
                port.write(frame_set(ch, [max(600, min(2400, target))]))
                p, _, s, _ = _sample(port, 0.6, watch=watch)
                peak, seen = max(peak, p), seen | s
            # back to where it started, then hold: a healthy servo is at idle current
            # well before the second is up. One that is not is fighting something.
            port.write(frame_set(ch, [frame[ch]]))
            p, _, s, settled = _sample(port, 1.0, tail=0.4, watch=watch)
            peak, seen = max(peak, p), seen | s
            delta = peak - idle_peak
            excess = settled - idle_mean
            what = verdict(delta, excess)
            peaks[ch] = delta
            if what.startswith("NOTHING"):
                quiet.append(ch)
            elif what.startswith("WEAK"):
                weak.append(ch)
            elif what.startswith(("HIGH", "STILL")):
                bad.append(ch)
            print(f"SERVO {ch + 1:>2} {ch:>3} {frame[ch]:>8} {peak:>+7.2f} {delta:>+6.2f} {excess:>+8.2f}  {what:<32} "
                  f"{sorted(TOUCH_LEG[i] for i in seen) or ''}")
            # A stall does not reach the 10 A cut on its own. Stop here rather than
            # keep it powered for the rest of the run.
            if excess >= STALLED_EXCESS_A:
                raise CurrentTrip(f"still {settled:.2f} A a second after SERVO {ch + 1} returned; idle was"
                                  f" {idle_mean:.2f} A")
        done = True
    except KeyboardInterrupt:
        print("\nprobe interrupted")
    except TimeoutError as exc:
        print(f"\nprobe stopped early: {exc}")
    except CurrentTrip as exc:
        tripped = True
        where = (f"SERVO {ch + 1} was being driven. The reading is the total for all 18, so it or another"
                 " servo is near stall.") if ch is not None else \
            ("Nothing was being driven yet: something is stalled at rest, an arm against a stop or a failed"
             " servo. Torque off and check each joint by hand.")
        print(f"\nCUT: {exc}. Relay off. {where}")
    finally:
        _release(port, None if tripped else frame)
    if quiet:
        print("no servo answered on: " + ", ".join(f"SERVO {c + 1}" for c in quiet)
              + ". Swap that plug with a neighbour to tell a dead servo from a dead header.")
    if weak:
        print("barely drew current: " + ", ".join(f"SERVO {c + 1}" for c in weak)
              + ". A working servo rises 0.9 A or more. Check the plug, then swap it with a neighbour.")
    hungry = [c for c in fleet_outliers(peaks) if c not in bad]
    if bad:
        print("bound or damaged: " + ", ".join(f"SERVO {c + 1}" for c in bad)
              + ". Torque off, move that joint by hand: a catch is a bind to fix first; free, suspect the servo.")
    if hungry:
        print("well above the pack, watch these: " + ", ".join(f"SERVO {c + 1}" for c in hungry)
              + ". Watch them: re-run after a session and compare.")
    if not done:
        print("run incomplete: the lists above cover only the headers that finished.")
    elif not quiet and not weak and not bad and not hungry:
        print("all 18 answered, none high, none still pulling.")
    return 0 if done else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--centre", action="store_true", help="enable torque and centre all 18 servos")
    parser.add_argument("--census", action="store_true", help="wiggle one header at a time and map it to a joint")
    parser.add_argument("--sweep", type=int, default=200, help="census wiggle either side of 1500 us")
    parser.add_argument("--probe", action="store_true", help="wiggle each header and report which draw current")
    parser.add_argument("--base", help="JSON list of the 18 pulses to start the probe from; default centres everything")
    args = parser.parse_args()
    if args.base:
        import json

        base = json.loads(args.base)
        if len(base) != 18:
            parser.error("--base needs exactly 18 pulse widths")
    else:
        base = [1500] * 18

    try:
        link = serial.Serial(args.port, 115200, timeout=0.2, exclusive=True)
    except serial.SerialException as exc:
        if "lock" in str(exc).lower():
            print(f"{args.port} is held by another process, almost certainly the server.")
            print("  sudo systemctl stop hexapod    then run this again")
            return 1
        raise
    with link as port:
        time.sleep(0.3)  # the firmware waits for the CDC connection before it parses
        amps, volts = read_get(port, CH_CURRENT, 2)
        print(f"voltage {volts / 310.3:.2f} V     current {(amps - 512) * 0.0814:+.2f} A")
        touch = read_get(port, 18, 6)
        print("touch pins", [f"{v / 310.3:.2f}V" for v in touch])

        if args.probe:
            if not args.base:
                print("\nno --base: every servo goes to 1500 us first. Legs off the ground. 3s")
                time.sleep(3)
            return probe(port, base, max(args.sweep, 250))
        if args.census:
            return census(port, args.sweep)
        if not args.centre:
            print("\nlink works. re-run with --centre to move the servos.")
            return 0

        print("\ncentering in 3s -- hold the robot")
        time.sleep(3)
        hard = CurrentWatch(CUT_A, CUT_S)
        hold = CurrentWatch(HOLD_CUT_A, HOLD_S)
        shown = 0.0
        try:
            port.write(frame_set(0, [1500] * 18))
            port.write(frame_set(CH_RELAY, [1]))
            started = time.monotonic()
            hard.arm(started)
            print(f"torque on, Ctrl-C to release. Relay drops on its own over {HOLD_CUT_A} A mean once settled"
                  f" (legs free reads about 0.2 A; one stalled servo about 4), over {CUT_A:.0f} A at any time,"
                  f" or after {STALE_S:.0f} s without a reading.")
            while True:
                time.sleep(0.1)
                try:
                    amps, volts = read_get(port, CH_CURRENT, 2)
                except TimeoutError:
                    hard.check_blind(time.monotonic())
                    continue
                a, now = (amps - 512) * 0.0814, time.monotonic()
                if now - shown >= 0.5:
                    print(f"\r{volts / 310.3:5.2f} V  {a:+6.2f} A", end="", flush=True)
                    shown = now
                mean = hard.add(a, now)
                if mean is not None:
                    raise CurrentTrip(f"{mean:.1f} A total mean over {CUT_S:.0f} s, several servos near stall")
                hard.check_blind(now)
                if now - started >= HOLD_SETTLE_S:
                    mean = hold.add(a, now)
                    if mean is not None:
                        raise CurrentTrip(f"{mean:.2f} A total at a static pose; legs free reads about 0.2 A, so"
                                          " something is pulling: an arm against a stop, a failed servo, or feet"
                                          " on the ground")
        except KeyboardInterrupt:
            pass
        except CurrentTrip as exc:
            print(f"\nCUT: {exc}. Torque off; check each joint by hand.")
        finally:
            _release(port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
