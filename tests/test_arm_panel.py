"""Testy pinecone_bot.arm_panel (panel webowy ramienia) bez sprzetu i bez lerobot.

Uruchomienie z katalogu repo:
    python -m pytest tests/test_arm_panel.py -q
"""
from __future__ import annotations

import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot.arm import HOME_POSE, JOINT_NAMES  # noqa: E402
from pinecone_bot.arm_panel import (  # noqa: E402
    FAKE_CALIBRATION,
    FAKE_NORM_MODES,
    MAX_QUEUE,
    ArmPanel,
    FakeSO101,
    jog_target,
    limits_from_calibration,
    list_motions,
)
from pinecone_bot.config import Config  # noqa: E402

MOTIONS_DIR = os.path.join(REPO_ROOT, "motions")


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.hook = None  # wolane przy kazdym sleep (np. zeby wcisnac STOP w trakcie ruchu)

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        assert s >= 0
        self.t += s
        if self.hook:
            self.hook()


def make_panel(arm=None, homed=True):
    cfg = Config()
    cfg.arm.motions_dir = MOTIONS_DIR
    clock = FakeClock()
    arm = arm or FakeSO101()
    limits = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
    panel = ArmPanel(arm, cfg, limits, sleep=clock.sleep, clock=clock.now)
    if homed:
        assert panel.submit({"cmd": "home"})[0]
        assert panel.process_one()
        assert panel.homed
        arm.actions.clear()
        arm.reads = 0
    return panel, arm, clock


def body_steps(actions, joint):
    vals = [a[f"{joint}.pos"] for a in actions if f"{joint}.pos" in a]
    return [abs(b - a) for a, b in zip(vals, vals[1:])], vals


# ---------------------------------------------------------------------------
# zakres z kalibracji
# ---------------------------------------------------------------------------

def test_limits_degrees_like_lerobot():
    limits = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
    # shoulder_lift 1006..3089 (docs/HARDWARE.md, pulapka 3): polowa zakresu w stopniach
    half = (3089 - 1006) / 2 * 360 / 4095
    assert limits["shoulder_lift"] == pytest.approx((-half, half))
    assert limits["gripper"] == (0.0, 100.0)
    assert set(limits) == set(JOINT_NAMES)


def test_limits_accept_objects_and_m100_mode():
    class Cal:
        def __init__(self, lo, hi):
            self.range_min, self.range_max = lo, hi

    calib = {j: Cal(1000, 3000) for j in JOINT_NAMES}
    modes = {j: "RANGE_M100_100" for j in JOINT_NAMES}
    modes["gripper"] = "RANGE_0_100"
    limits = limits_from_calibration(calib, modes)
    assert limits["elbow_flex"] == (-100.0, 100.0)


def test_limits_reject_bad_calibration():
    calib = dict(FAKE_CALIBRATION)
    del calib["wrist_roll"]
    with pytest.raises(ValueError):
        limits_from_calibration(calib, FAKE_NORM_MODES)
    calib = dict(FAKE_CALIBRATION, elbow_flex={"range_min": 3000, "range_max": 1000})
    with pytest.raises(ValueError):
        limits_from_calibration(calib, FAKE_NORM_MODES)


def test_jog_target_clips_and_never_reverses():
    assert jog_target(0.0, 10.0, -50.0, 50.0) == 10.0
    assert jog_target(45.0, 10.0, -50.0, 50.0) == 50.0
    assert jog_target(-48.0, -5.0, -50.0, 50.0) == -50.0
    # przegub juz poza zakresem: "+" nie moze go sciagnac w dol do hi
    assert jog_target(60.0, 5.0, -50.0, 50.0) == 60.0
    # za to "-" wraca w zakres
    assert jog_target(60.0, -5.0, -50.0, 50.0) == 50.0


# ---------------------------------------------------------------------------
# HOME przy starcie
# ---------------------------------------------------------------------------

def test_commands_rejected_until_home():
    panel, arm, _ = make_panel(homed=False)
    ok, msg = panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 5})
    assert not ok and "HOME" in msg
    assert not panel.submit({"cmd": "open"})[0]
    assert panel.submit({"cmd": "stop"})[0]          # STOP zawsze
    assert panel.submit({"cmd": "home"})[0]
    panel.process_one()
    assert panel.homed
    assert arm.pose == pytest.approx(HOME_POSE)
    assert panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 5})[0]


def test_start_queues_home_first():
    import time

    panel, arm, _ = make_panel(homed=False)
    panel.start(home_first=True)
    deadline = time.monotonic() + 5.0
    while not panel.homed and time.monotonic() < deadline:
        time.sleep(0.01)
    panel.shutdown()
    assert panel.homed
    assert arm.pose == pytest.approx(HOME_POSE)


# ---------------------------------------------------------------------------
# jog
# ---------------------------------------------------------------------------

def test_jog_moves_one_joint_with_small_steps_and_no_reads():
    panel, arm, _ = make_panel()
    start = arm.pose["elbow_flex"]
    assert panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": -10})[0]
    panel.process_one()
    assert arm.pose["elbow_flex"] == pytest.approx(start - 10)
    steps, _ = body_steps(arm.actions, "elbow_flex")
    assert max(steps) <= panel.max_step["elbow_flex"] + 1e-9
    assert all(set(a) == {"elbow_flex.pos"} for a in arm.actions)
    assert arm.reads == 0, "jog nie moze robic sync_read (pulapka 10)"


def test_jogs_accumulate_from_setpoint():
    panel, arm, _ = make_panel()
    start = arm.pose["shoulder_pan"]
    for _ in range(3):
        panel.submit({"cmd": "jog", "joint": "shoulder_pan", "step": 5})
    while panel.process_one():
        pass
    assert arm.pose["shoulder_pan"] == pytest.approx(start + 15)


def test_jog_clipped_to_calibration_range():
    panel, arm, _ = make_panel()
    lo, hi = panel.limits["wrist_roll"]
    for _ in range(40):
        panel.submit({"cmd": "jog", "joint": "wrist_roll", "step": 10})
        panel.process_one()
    assert arm.pose["wrist_roll"] == pytest.approx(hi)
    assert max(a["wrist_roll.pos"] for a in arm.actions) <= hi + 1e-9


def test_invalid_jogs_rejected():
    panel, _, _ = make_panel()
    assert not panel.submit({"cmd": "jog", "joint": "nos", "step": 5})[0]
    assert not panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 7})[0]
    assert not panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": "x"})[0]
    assert not panel.submit({"cmd": "rm -rf"})[0]


def test_queue_is_bounded():
    panel, _, _ = make_panel()
    for _ in range(MAX_QUEUE):
        assert panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 1})[0]
    ok, msg = panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 1})
    assert not ok and "pelna" in msg


# ---------------------------------------------------------------------------
# chwytak, ruchy, STOP
# ---------------------------------------------------------------------------

def test_open_and_close_gripper():
    panel, arm, _ = make_panel()
    panel.submit({"cmd": "open"})
    panel.process_one()
    assert arm.pose["gripper"] == pytest.approx(100.0)
    steps, _ = body_steps(arm.actions, "gripper")
    assert max(steps) <= panel.max_step["gripper"] + 1e-9
    panel.submit({"cmd": "close"})
    panel.process_one()
    assert arm.pose["gripper"] == pytest.approx(0.0)


def test_motion_list_and_replay():
    names = list_motions(MOTIONS_DIR)
    assert {"home", "grasp_mid", "drop_box"} <= set(names)
    panel, arm, _ = make_panel()
    assert panel.submit({"cmd": "motion", "name": "drop_box"})[0]
    panel.process_one()
    assert panel.last_error is None, panel.last_error
    assert "drop_box" in panel.last_result
    assert not panel.submit({"cmd": "motion", "name": "../pinecone_config"})[0]


def test_stop_interrupts_motion_and_clears_queue():
    panel, arm, clock = make_panel()
    panel.submit({"cmd": "jog", "joint": "shoulder_lift", "step": 10})
    panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 10})
    start = arm.pose["shoulder_lift"]
    ticks = {"n": 0}

    def press_stop():
        ticks["n"] += 1
        if ticks["n"] == 2:
            panel.stop()

    clock.hook = press_stop
    panel.process_one()
    clock.hook = None
    assert "przerwane" in panel.last_result
    moved = arm.pose["shoulder_lift"] - start
    assert 0 < moved < 10
    assert not panel.process_one(), "STOP musi wyczyscic kolejke"
    # po STOP nowe komendy dzialaja i startuja od miejsca zatrzymania
    assert panel.submit({"cmd": "jog", "joint": "shoulder_lift", "step": 1})[0]
    panel.process_one()
    assert arm.pose["shoulder_lift"] == pytest.approx(start + moved + 1)


def test_stop_interrupts_waypoint_replay():
    panel, arm, clock = make_panel()
    panel.submit({"cmd": "motion", "name": "grasp_mid"})
    ticks = {"n": 0}

    def press_stop():
        ticks["n"] += 1
        if ticks["n"] == 5:
            panel.stop()

    clock.hook = press_stop
    panel.process_one()
    assert "przerwane" in panel.last_result
    sent = len(arm.actions)
    assert 0 < sent < 20


# ---------------------------------------------------------------------------
# odczyt pozycji
# ---------------------------------------------------------------------------

def test_reads_rate_limited_to_2hz():
    panel, arm, clock = make_panel()
    reads = 0
    for _ in range(40):          # 40 x 0.05 s = 2 s
        reads += panel.maybe_read()
        clock.t += 0.05
    assert reads == 4
    assert panel.snapshot()["positions"]["elbow_flex"] == pytest.approx(arm.pose["elbow_flex"])


def test_read_error_reported_not_raised():
    panel, arm, clock = make_panel()
    arm.fail_reads = 1
    assert not panel.maybe_read()
    assert "status packet" in panel.last_error
    clock.t += 1.0
    assert panel.maybe_read()


def test_command_error_keeps_panel_alive():
    panel, arm, _ = make_panel()

    def boom(action):
        raise ConnectionError("There is no status packet!")

    arm.send_action = boom
    panel.submit({"cmd": "open"})
    panel.process_one()
    assert "status packet" in panel.last_error
    snap = panel.snapshot()
    assert snap["busy"] is None and snap["homed"]


# ---------------------------------------------------------------------------
# serwer: kto moze wolac API (tools/arm_web.py)
# ---------------------------------------------------------------------------

def test_origin_check_allows_only_our_panels():
    sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))
    import arm_web

    ports = [*arm_web.PANEL_PORTS, 8010]
    assert arm_web.origin_port_ok("http://172.20.10.4:8000", ports)
    assert arm_web.origin_port_ok("http://localhost:8010", ports)
    assert arm_web.origin_port_ok("http://robot.local:8090", ports)  # tools/robot_panel.py
    assert not arm_web.origin_port_ok("http://evil.example", ports)
    assert not arm_web.origin_port_ok("http://evil.example:8080", ports)
    assert not arm_web.origin_port_ok("https://172.20.10.4:8000", ports)
    assert not arm_web.origin_port_ok("http://evil.example/x:8000", ports)


# ---------------------------------------------------------------------------
# --no-home (kamera na ramieniu)
# ---------------------------------------------------------------------------

def test_manual_only_never_homes_and_jogs_from_read():
    cfg = Config()
    cfg.arm.motions_dir = MOTIONS_DIR
    clock = FakeClock()
    arm = FakeSO101()
    start_pose = dict(arm.pose)
    limits = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
    panel = ArmPanel(arm, cfg, limits, sleep=clock.sleep, clock=clock.now, manual_only=True)
    panel.start(home_first=True)   # watek: tylko odczyty, zadnego HOME
    import time
    deadline = time.monotonic() + 5.0
    while not panel.positions and time.monotonic() < deadline:
        time.sleep(0.01)
    panel.shutdown()
    assert arm.actions == [], "manual_only nie moze ruszyc ramieniem sam z siebie"
    assert panel.snapshot()["manual_only"]

    for cmd in ({"cmd": "home"}, {"cmd": "motion", "name": "grasp_mid"}):
        ok, msg = panel.submit(cmd)
        assert not ok and "no-home" in msg
    assert panel.submit({"cmd": "jog", "joint": "shoulder_lift", "step": -5})[0]
    panel.process_one()
    assert panel.last_error is None, panel.last_error
    assert arm.pose["shoulder_lift"] == pytest.approx(start_pose["shoulder_lift"] - 5)
    # pozostale stawy nietkniete, wysylany tylko jogowany
    assert all(set(a) == {"shoulder_lift.pos"} for a in arm.actions)
    assert {j: arm.pose[j] for j in JOINT_NAMES if j != "shoulder_lift"} == \
        {j: start_pose[j] for j in JOINT_NAMES if j != "shoulder_lift"}


def test_manual_only_jog_before_first_read_is_refused():
    panel, arm, _ = make_panel(homed=False)
    panel.manual_only = True
    panel.homed = True
    panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 1})
    panel.process_one()
    assert "odczyt" in panel.last_error
    assert arm.actions == []


def test_jog_blocked_when_joint_outside_calibration_range():
    # Pi 2026-09-26: shoulder_lift odczyt 127.7 przy zakresie +-91.6 (kamera na ramieniu);
    # "-1" skonczyloby sie skokiem serwa o ~36 st do granicy
    cfg = Config()
    cfg.arm.motions_dir = MOTIONS_DIR
    clock = FakeClock()
    pose = dict(HOME_POSE)
    pose["shoulder_lift"] = 127.7
    arm = FakeSO101(pose=pose)
    limits = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
    panel = ArmPanel(arm, cfg, limits, sleep=clock.sleep, clock=clock.now, manual_only=True)
    assert panel.maybe_read()
    for step in (-1, 1, -10):
        panel.submit({"cmd": "jog", "joint": "shoulder_lift", "step": step})
        panel.process_one()
        assert "poza zakresem" in panel.last_error
    assert arm.actions == []
    # inne stawy dalej dzialaja
    panel.submit({"cmd": "jog", "joint": "elbow_flex", "step": 1})
    panel.process_one()
    assert panel.last_error is None
    assert all(set(a) == {"elbow_flex.pos"} for a in arm.actions)
