#!/usr/bin/env python3
"""Hammer an i2c device and print a rolling success rate. Finds loose leads.

    python3 tools/i2cwatch.py              # the IMU at 0x68 on bus 1
    python3 tools/i2cwatch.py --address 0x69

Run it, then wiggle one lead at a time: VCC, GND, SDA, SCL. The lead that
changes the number is the bad one. A good connection reads 100% and never
moves, even when you flex the loom.

Nothing here imports the hexapod package, on purpose: it is a wiring test, not
a software test. Stop the server first, or it will be reading the same chip.
"""

import argparse
import fcntl
import os
import sys
import time
from collections import deque

I2C_SLAVE = 0x0703
WINDOW = 50  # attempts in the rolling window


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    parser.add_argument("--register", type=lambda s: int(s, 0), default=0x75,
                        help="register to read, default WHO_AM_I")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    try:
        fd = os.open(f"/dev/i2c-{args.bus}", os.O_RDWR)
        fcntl.ioctl(fd, I2C_SLAVE, args.address)
    except OSError as exc:
        print(f"cannot open bus {args.bus} at {args.address:#04x}: {exc}")
        return 1

    live = sys.stdout.isatty()
    recent = deque(maxlen=WINDOW)
    total = good = 0
    period = 1.0 / args.hz
    print(f"reading {args.register:#04x} from {args.address:#04x} at {args.hz:.0f} Hz. "
          f"Wiggle one lead at a time. Ctrl-C to stop.")
    try:
        while True:
            try:
                os.write(fd, bytes((args.register,)))
                os.read(fd, 1)
                recent.append(True)
                good += 1
            except OSError:
                recent.append(False)
            total += 1
            rolling = 100.0 * sum(recent) / len(recent)
            if live:
                bar = "#" * int(rolling / 4) + "." * (25 - int(rolling / 4))
                print(f"\rlast {len(recent):>3}: [{bar}] {rolling:5.1f}%   "
                      f"overall {100.0 * good / total:5.1f}% of {total}", end="", flush=True)
            elif total % WINDOW == 0:
                # Piped or logged: one line per window instead of redrawing.
                print(f"last {len(recent)}: {rolling:.1f}%   overall {100.0 * good / total:.1f}% of {total}",
                      flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n{good} of {total} reads succeeded ({100.0 * good / total:.1f}%)")
    finally:
        os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
