"""Testy pinecone_bot.sequence (sekwencje jazda + ramie) bez sprzetu i bez sieci.

Uruchomienie z katalogu repo:
    python -m pytest tests/test_sequence.py -q
"""
from __future__ import annotations

import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot.sequence import (  # noqa: E402
    MAX_STEPS,
    SequenceRunner,
    delete_sequence,
    load_sequences,
    parse_steps,
    save_sequence,
    step_text,
)


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.hook = None  # wolane przy kazdym sleep (np. STOP w trakcie kroku)

    def now(self):
        return self.t

    def sleep(self, s):
        assert s >= 0
        self.t += s
        if self.hook:
            self.hook()


class FakeArmPanel:
    """Udaje snapshot tools/arm_web.py: ruch trwa `duration` s od zlecenia."""

    def __init__(self, motions=("grasp_cam",), duration=2.0, clock=None):
        self.motions = set(motions)
        self.duration = duration
        self.clock = clock
        self.started = []
        self.stops = 0
        self.busy_until = -1.0
        self.error = None
        self.result = ""
        self.offline = False

    def start(self, name):
        if name not in self.motions:
            return False, f"brak ruchu '{name}' w motions/"
        self.started.append(name)
        self.busy_until = self.clock.now() + self.duration
        return True, "w kolejce"

    def state(self):
        if self.offline:
            return None
        busy = self.clock.now() < self.busy_until
        return {"busy": {"cmd": "motion"} if busy else None, "queue": 0,
                "error": None if busy else self.error, "result": self.result}

    def stop(self):
        self.stops += 1
        self.busy_until = -1.0


def make_runner(arm=None, clock=None, **kw):
    clock = clock or FakeClock()
    arm = arm or FakeArmPanel(clock=clock)
    arm.clock = clock
    drives = []
    runner = SequenceRunner(
        drive=lambda sp, st: drives.append((clock.now(), sp, st)),
        arm_start=arm.start, arm_state=arm.state, arm_stop=arm.stop,
        sleep=clock.sleep, clock=clock.now, **kw,
    )
    return runner, drives, arm, clock


# ---------------------------------------------------------------------------
# format
# ---------------------------------------------------------------------------

def test_parse_normalizes_and_validates():
    steps = parse_steps([
        {"type": "drive", "speed": "0.3", "seconds": 2},
        {"type": "arm", "name": "grasp_cam"},
        {"type": "wait", "seconds": 1.5},
    ])
    assert steps[0] == {"type": "drive", "speed": 0.3, "steer": 0.0, "seconds": 2.0}
    assert steps[1] == {"type": "arm", "name": "grasp_cam"}
    assert steps[2] == {"type": "wait", "seconds": 1.5}
    assert "jazda 2 s" in step_text(steps[0])


@pytest.mark.parametrize("bad", [
    [],
    "nie lista",
    [{"type": "drive", "speed": 1.5, "seconds": 1}],
    [{"type": "drive", "speed": 0.2, "seconds": 999}],
    [{"type": "drive", "speed": "abc", "seconds": 1}],
    [{"type": "arm", "name": "../etc"}],
    [{"type": "wait"}],
    [{"type": "fly", "seconds": 1}],
    [{"type": "wait", "seconds": 1}] * (MAX_STEPS + 1),
])
def test_parse_rejects_bad_steps(bad):
    with pytest.raises(ValueError):
        parse_steps(bad)


def test_save_load_delete_roundtrip(tmp_path):
    d = str(tmp_path / "sequences")
    steps = [{"type": "drive", "speed": 0.25, "steer": 0.0, "seconds": 3.0}, {"type": "arm", "name": "grasp_cam"}]
    path = save_sequence(d, "szyszka1", steps)
    assert os.path.exists(path)
    with open(path, "rb") as fh:
        assert all(b < 128 for b in fh.read()), "plik ma byc czystym ASCII"
    (tmp_path / "sequences" / "zepsuty.json").write_text("{nie json", encoding="ascii")
    loaded = load_sequences(d)
    assert loaded == {"szyszka1": steps}
    assert delete_sequence(d, "szyszka1")
    assert not delete_sequence(d, "szyszka1")
    assert load_sequences(d) == {}
    with pytest.raises(ValueError):
        save_sequence(d, "zla nazwa!", steps)
    assert load_sequences(str(tmp_path / "nie_ma")) == {}


# ---------------------------------------------------------------------------
# odtwarzanie
# ---------------------------------------------------------------------------

def test_drive_step_sets_target_then_stops_after_seconds():
    runner, drives, _, clock = make_runner(settle_s=0.3)
    ok = runner.run("t", parse_steps([{"type": "drive", "speed": 0.3, "steer": -0.1, "seconds": 2.0}]))
    assert ok
    assert drives[0] == (0.0, 0.3, -0.1)
    assert drives[1][1:] == (0.0, 0.0) and drives[1][0] == pytest.approx(2.0)
    assert clock.now() == pytest.approx(2.3)     # settle po jezdzie
    st = runner.status()
    assert not st["running"] and st["finished"] == "ukonczona" and st["error"] is None


def test_arm_step_waits_until_panel_idle():
    arm = FakeArmPanel(duration=2.0)
    runner, drives, arm, clock = make_runner(arm=arm, poll_s=0.25)
    ok = runner.run("t", parse_steps([{"type": "arm", "name": "grasp_cam"}, {"type": "wait", "seconds": 1}]))
    assert ok
    assert arm.started == ["grasp_cam"]
    assert 2.0 <= clock.now() <= 3.5
    assert arm.stops == 0, "udana sekwencja nie wysyla STOP do ramienia"


def test_stop_during_drive_zeroes_drive_and_stops_arm():
    runner, drives, arm, clock = make_runner()
    clock.hook = lambda: runner.stop() if clock.now() >= 0.5 else None
    ok = runner.run("t", parse_steps([
        {"type": "drive", "speed": 0.5, "seconds": 5.0},
        {"type": "arm", "name": "grasp_cam"},
    ]))
    assert not ok
    assert drives[-1][1:] == (0.0, 0.0)
    assert clock.now() < 1.0, "STOP ma dzialac w ciagu ~0.1 s, nie po 5 s"
    assert arm.started == [], "kolejne kroki po STOP sie nie wykonuja"
    assert arm.stops == 1
    assert runner.status()["finished"] == "przerwana"


def test_arm_rejection_and_errors_abort_sequence():
    runner, drives, arm, clock = make_runner()
    assert not runner.run("t", parse_steps([{"type": "arm", "name": "nie_ma"}]))
    assert "odrzucil" in runner.status()["error"]

    arm.error = "motion: There is no status packet!"
    assert not runner.run("t", parse_steps([{"type": "arm", "name": "grasp_cam"}]))
    assert "status packet" in runner.status()["error"]

    arm.error = None
    arm.result = "przerwane: motion"
    assert not runner.run("t", parse_steps([{"type": "arm", "name": "grasp_cam"}]))
    assert runner.status()["finished"].startswith("przerwana")

    arm.result = ""
    arm.offline = True
    assert not runner.run("t", parse_steps([{"type": "arm", "name": "grasp_cam"}]))
    assert "nie odpowiada" in runner.status()["error"]
    assert drives[-1][1:] == (0.0, 0.0)


def test_arm_timeout():
    arm = FakeArmPanel(duration=1000.0)
    runner, _, arm, clock = make_runner(arm=arm, arm_timeout_s=10.0)
    assert not runner.run("t", parse_steps([{"type": "arm", "name": "grasp_cam"}]))
    assert "10 s" in runner.status()["error"]
    assert arm.stops == 1


def test_start_runs_in_thread_and_refuses_second():
    import threading
    import time

    gate = threading.Event()
    drives = []
    runner = SequenceRunner(
        drive=lambda sp, st: drives.append((sp, st)),
        arm_start=lambda n: (True, ""), arm_state=lambda: {"busy": None, "queue": 0}, arm_stop=lambda: None,
        sleep=lambda s: gate.wait(0.05),
    )
    assert runner.start("a", parse_steps([{"type": "wait", "seconds": 5}]))
    time.sleep(0.02)
    assert runner.status()["running"]
    assert not runner.start("b", parse_steps([{"type": "wait", "seconds": 1}]))
    runner.stop()
    gate.set()
    runner._thread.join(2.0)
    assert not runner.status()["running"]
    assert drives[-1] == (0.0, 0.0)
