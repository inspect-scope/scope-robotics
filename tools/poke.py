#!/usr/bin/env python3
"""Standalone check that the Servo2040 link works. Only needs pyserial.

    python3 tools/poke.py /dev/ttyACM0            # read volts/amps, no torque
    python3 tools/poke.py /dev/ttyACM0 --centre   # torque on, every servo to 1500us

Nothing here imports the hexapod package, on purpose: if this script talks to the
board then the wiring, the firmware and the port are all fine, and anything that
breaks after this is our code.
"""

import argparse
import sys
import time

import serial

SET_CMD, GET_CMD = 0xD3, 0xC7
CH_CURRENT, CH_VOLTAGE, CH_RELAY = 24, 25, 26


def frame_set(start, values):
    out = bytearray((SET_CMD, start, len(values)))
    for value in values:
        out += bytes((value & 0x7F, (value >> 7) & 0x7F))
    return bytes(out)


def read_get(port, start, count, timeout=1.0):
    port.reset_input_buffer()
    port.write(bytes((GET_CMD, start, count)))
    deadline, buf = time.time() + timeout, b""
    while time.time() < deadline:
        buf += port.read(port.in_waiting or 1)
        head = buf.find(bytes((GET_CMD, start, count)))
        if head >= 0 and len(buf) >= head + 3 + 2 * count:
            body = buf[head + 3 : head + 3 + 2 * count]
            return [body[i] | (body[i + 1] << 7) for i in range(0, len(body), 2)]
    raise TimeoutError(f"no reply to GET {start}+{count} (got {buf!r})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--centre", action="store_true", help="enable torque and centre all 18 servos")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.2) as port:
        time.sleep(0.3)  # the firmware waits for the CDC connection before it parses
        amps, volts = read_get(port, CH_CURRENT, 2)
        print(f"voltage {volts / 310.3:.2f} V     current {(amps - 512) * 0.0814:+.2f} A")
        touch = read_get(port, 18, 6)
        print("touch pins", [f"{v / 310.3:.2f}V" for v in touch])

        if not args.centre:
            print("\nlink works. re-run with --centre to move the servos.")
            return 0

        print("\ncentering in 3s -- hold the robot")
        time.sleep(3)
        port.write(frame_set(0, [1500] * 18))
        port.write(frame_set(CH_RELAY, [1]))
        print("torque on, Ctrl-C to release")
        try:
            while True:
                time.sleep(0.5)
                amps, volts = read_get(port, CH_CURRENT, 2)
                print(f"\r{volts / 310.3:5.2f} V  {(amps - 512) * 0.0814:+6.2f} A", end="", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            port.write(frame_set(CH_RELAY, [0]))
            print("\ntorque off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
