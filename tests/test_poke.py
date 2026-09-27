"""tools/poke.py is a script, not a module, so it is loaded by path. Only the
pure parts are tested; the serial link needs a board."""

import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "tools", "poke.py")


@pytest.fixture(scope="module")
def poke():
    spec = importlib.util.spec_from_file_location("poke", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("text, expected", [
    ("RR femur", ("R3", "femur")),
    ("fl coxa", ("L1", "coxa")),
    ("  MR  Tibia ", ("R2", "tibia")),
    ("R3 femur", ("R3", "femur")),  # config names are accepted too
    ("femur", None),
    ("RR knee", None),
    ("XX femur", None),
    ("", None),
])
def test_answers_are_parsed_by_position(poke, text, expected):
    assert poke.parse_answer(text) == expected


def test_census_emits_a_config_block(poke):
    # Every leg's three headers rotated by one: SERVO 1 is the femur, 2 the tibia, 3 the coxa.
    answers = {}
    for base, leg in ((0, "R3"), (3, "L3"), (6, "R2"), (9, "L2"), (12, "R1"), (15, "L1")):
        answers[base] = (leg, "femur")
        answers[base + 1] = (leg, "tibia")
        answers[base + 2] = (leg, "coxa")
    lines, problems = poke.census_config(answers)
    assert problems == []
    text = "\n".join(lines)
    assert text.startswith("servos:")
    assert "R3: {coxa: {channel:  2," in text
    assert "femur: {channel:  0," in text
    assert "tibia: {channel:  1," in text
    assert text.count("channel:") == 18


def test_census_reports_gaps_and_duplicates(poke):
    answers = {0: ("R3", "coxa"), 1: ("R3", "femur"), 4: ("R3", "femur")}
    lines, problems = poke.census_config(answers)
    assert lines == ["servos:"]  # no leg is complete, so no block
    assert "R3: no header moved its tibia" in problems
    assert any("R3 femur was named for channels [1, 4]" in p for p in problems)
    assert any(p.startswith("L1: no header moved its coxa, femur, tibia") for p in problems)


@pytest.mark.parametrize("delta, settled, expected", [
    (3.1, 0.0, "servo moved"),
    (0.15, 0.0, "servo moved"),
    (0.1, 0.0, "NOTHING drew current"),                   # one ADC count (0.08 A) is noise
    (0.0, 0.0, "NOTHING drew current"),
    (5.0, 0.0, "HIGH: above stall spec, bind or damage"),        # the L1 femur, the day it died
    (4.4, 0.0, "servo moved"),                             # just under: caught by the fleet check instead
    (2.5, 1.2, "STILL PULLING: stalled or bound"),        # settled current says it never stopped working
    (0.0, 1.2, "NOTHING drew current"),                   # a dead servo cannot be stalled
])
def test_probe_verdict_thresholds(poke, delta, settled, expected):
    assert poke.verdict(delta, settled) == expected


def test_fleet_outliers_catch_the_hungry_one(poke):
    # 26 Sep 2026 sweep, deltas over idle: everything 1.3 to 3.5, SERVO 17 at 5.0 before it died.
    peaks = {0: 3.34, 1: 3.50, 2: 3.10, 3: 3.10, 4: 1.31, 5: 2.53, 6: 1.79, 7: 2.36, 8: 2.85,
             9: 2.61, 10: 3.01, 11: 3.18, 12: 3.01, 13: 2.77, 14: 1.31, 15: 1.47, 16: 4.84, 17: 1.55}
    assert poke.fleet_outliers(peaks) == [16]
    peaks[16] = 0.08  # now dead: not an outlier, it is quiet
    assert poke.fleet_outliers(peaks) == []


def test_fleet_outliers_need_a_pack_to_compare_against(poke):
    assert poke.fleet_outliers({0: 5.0, 1: 1.0}) == []


def test_current_watch_needs_the_window_covered(poke):
    w = poke.CurrentWatch(limit=10.0, span=1.0)
    assert w.add(12.0, 100.0) is None          # first sample: window not covered yet
    assert w.add(12.0, 100.5) is None          # half a second: still not
    assert w.add(12.0, 101.0) == 12.0          # covered and over: trip, returns the mean


def test_current_watch_is_not_reset_by_one_low_sample(poke):
    # The old every-sample-over timer reset on any dip; a current swinging around
    # the cut never tripped. A mean does.
    w = poke.CurrentWatch(limit=10.0, span=1.0)
    trips = [w.add(12.0 if i % 2 else 9.5, 100.0 + i * 0.1) for i in range(12)]
    assert any(m is not None for m in trips)


def test_current_watch_ignores_a_short_spike(poke):
    w = poke.CurrentWatch(limit=10.0, span=1.0)
    trips = [w.add(15.0 if i == 3 else 1.0, 100.0 + i * 0.1) for i in range(15)]
    assert all(m is None for m in trips)


class FakePort:
    """Enough of pyserial for poke.py: answers every GET with canned counts and
    records every SET so the test can see the relay frame."""

    def __init__(self, current_counts):
        # channel 24 raw; 512 = 0 A, 0.0814 A per count. An int, or a callable
        # taking the number of telemetry GETs so far, to change current mid-run.
        self.current_counts = current_counts
        self.gets = 0
        self.sets = []
        self._rx = b""
        self.in_waiting = 0

    def reset_input_buffer(self):
        self._rx = b""; self.in_waiting = 0

    def write(self, data):
        if data[0] == 0xD3:                                   # SET: remember start + values
            start, count = data[1], data[2]
            vals = [data[3 + 2 * i] | (data[4 + 2 * i] << 7) for i in range(count)]
            self.sets.append((start, vals))
        elif data[0] == 0xC7:                                 # GET: queue a reply
            start, count = data[1], data[2]
            self.gets += 1
            cur = self.current_counts(self.gets) if callable(self.current_counts) else self.current_counts
            if cur is None:          # a board that has stopped answering
                return
            body = bytearray()
            for ch in range(start, start + count):
                v = cur if ch == 24 else (2400 if ch == 25 else 10)
                body += bytes((v & 0x7F, (v >> 7) & 0x7F))
            self._rx = bytes((0xC7, start, count)) + bytes(body)
            self.in_waiting = len(self._rx)

    def read(self, n):
        out, self._rx = self._rx[:n], self._rx[n:]
        self.in_waiting = len(self._rx)
        return out

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class FakeClock:
    """poke.py samples until time.time() passes a deadline. Advance a fake clock
    50 ms per read and by the requested amount per sleep, so a full 18-header
    probe runs in milliseconds of real time and is deterministic."""

    def __init__(self):
        self.t = 1000.0

    def time(self):
        self.t += 0.05
        return self.t

    def monotonic(self):
        return self.time()

    def sleep(self, seconds):
        self.t += seconds

    def strftime(self, *args, **kwargs):
        return "00:00:00"


def test_probe_opens_the_relay_when_already_stalled_at_rest(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=512 + int(12.0 / 0.0814))  # 12 A from the first sample
    poke.probe(port, [1500] * 18, sweep=200)
    out = capsys.readouterr().out
    assert "CUT: 12.0 A" in out and "stalled at rest" in out
    assert port.sets[-1] == (26, [0]), "last thing written must be relay off"


def test_probe_runs_clean_at_idle_current(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=512 + int(0.2 / 0.0814))
    poke.probe(port, [1500] * 18, sweep=200)
    out = capsys.readouterr().out
    assert "CUT" not in out
    assert "no servo answered on: SERVO  1, SERVO  2" in out or "no servo answered on" in out  # flat current = nothing moved
    assert port.sets[-1] == (26, [0])


def _counts(amps):
    return 512 + int(round(amps / 0.0814))


def test_probe_cuts_a_stall_that_starts_mid_run(poke, monkeypatch, capsys):
    # Before the fix the 1 s timer reset for every 0.6 s window, so this ran all
    # 18 headers at 12 A and only opened the relay at the end.
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=lambda n: _counts(0.2) if n <= 40 else _counts(12.0))
    poke.probe(port, [1500] * 18, sweep=200)
    out = capsys.readouterr().out
    assert "CUT: 1" in out and "was being driven" in out
    headers_done = sum(1 for line in out.splitlines() if line.startswith("SERVO "))
    assert headers_done < 18, "the cut must land during the header loop, not after it"
    assert port.sets[-1] == (26, [0])


def test_probe_stops_before_moving_anything_if_already_pulling(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=_counts(1.0))   # 1 A at rest: over IDLE_MAX_A, under the 10 A cut
    assert poke.probe(port, [1500] * 18, sweep=200) == 1
    out = capsys.readouterr().out
    assert "STOP: 0.98 A at rest" in out and "nothing was wiggled" in out   # 1.0 A quantises to 12 counts
    assert not any(line.startswith("SERVO ") for line in out.splitlines())
    single_channel_sets = [s for s in port.sets if s[0] < 18 and len(s[1]) == 1]
    assert single_channel_sets == [], "no header may be wiggled after the idle check fails"
    assert port.sets[-1] == (26, [0])


def test_census_drops_torque_before_every_prompt(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=_counts(0.2))
    relay_at_prompt = []

    def fake_input(prompt=""):
        relay = [s for s in port.sets if s[0] == 26]
        relay_at_prompt.append(relay[-1])
        return "q" if len(relay_at_prompt) >= 3 else ""

    monkeypatch.setattr("builtins.input", fake_input)
    poke.census(port, sweep=200)
    assert relay_at_prompt and all(r == (26, [0]) for r in relay_at_prompt)


def test_current_watch_blind_check(poke):
    w = poke.CurrentWatch()
    w.arm(100.0)
    w.check_blind(101.9)                       # inside the grace period: fine
    with pytest.raises(poke.CurrentTrip, match="no current reading"):
        w.check_blind(102.1)


def test_probe_stops_on_a_single_stall(poke, monkeypatch, capsys):
    # One stalled servo lifts the total to about 4 A: never the 10 A cut. The
    # STILL PULLING check must stop the run instead of driving on for 50 s.
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=lambda n: _counts(0.2) if n <= 40 else _counts(4.1))
    assert poke.probe(port, [1500] * 18, sweep=200) == 1
    out = capsys.readouterr().out
    assert "CUT: still 4.07 A" in out   # 4.1 A quantises to 50 counts
    assert sum(1 for line in out.splitlines() if line.startswith("SERVO ")) < 18
    assert "run incomplete" in out and "all 18 answered" not in out
    assert port.sets[-1] == (26, [0])
    assert len(port.sets[-2][1]) == 1, "after a trip the relay goes first, no return-to-base frame"


def test_probe_cuts_when_the_board_goes_silent(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=lambda n: _counts(0.2) if n <= 20 else None)
    poke.probe(port, [1500] * 18, sweep=200)
    out = capsys.readouterr().out
    assert "CUT: no current reading" in out and "unplug the servo battery" in out
    assert port.sets[-1] == (26, [0])


def test_probe_releases_the_relay_if_the_first_read_fails(poke, monkeypatch, capsys):
    monkeypatch.setattr(poke, "time", FakeClock())
    port = FakePort(current_counts=lambda n: None if n == 1 else _counts(0.2))
    poke.probe(port, [1500] * 18, sweep=200)
    assert "probe stopped early" in capsys.readouterr().out
    assert port.sets[-1] == (26, [0])


def test_a_second_ctrl_c_during_cleanup_still_drops_the_relay(poke, monkeypatch):
    class InterruptingClock(FakeClock):
        fired = False

        def sleep(self, seconds):
            if seconds == 0.5 and not self.fired:   # the return-to-base pause in _release
                self.fired = True
                raise KeyboardInterrupt
            super().sleep(seconds)

    monkeypatch.setattr(poke, "time", InterruptingClock())
    port = FakePort(current_counts=_counts(0.2))
    with pytest.raises(KeyboardInterrupt):
        poke.probe(port, [1500] * 18, sweep=200)
    assert port.sets[-1] == (26, [0])


def _run_centre(poke, monkeypatch, port):
    monkeypatch.setattr(poke, "time", FakeClock())
    monkeypatch.setattr(poke.serial, "Serial", lambda *a, **k: port)
    monkeypatch.setattr(poke.sys, "argv", ["poke.py", "/dev/fake", "--centre"])
    return poke.main()


def test_centre_cuts_a_single_stall_at_a_static_pose(poke, monkeypatch, capsys):
    port = FakePort(current_counts=lambda n: _counts(0.2) if n <= 30 else _counts(4.0))
    assert _run_centre(poke, monkeypatch, port) == 0
    out = capsys.readouterr().out
    # trips as soon as the rolling mean passes 1.5 A, with part of the window still at idle
    assert "A total at a static pose" in out and "CUT: " in out
    assert port.sets[-1] == (26, [0])


def test_centre_cuts_when_the_board_goes_silent(poke, monkeypatch, capsys):
    port = FakePort(current_counts=lambda n: _counts(0.2) if n <= 12 else None)
    assert _run_centre(poke, monkeypatch, port) == 0
    out = capsys.readouterr().out
    assert "no current reading" in out
    assert port.sets[-1] == (26, [0])
