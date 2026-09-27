"""Testy pinecone_bot/zygzak.py - plan, sledzenie pozycji i jazda w symulacji (bez sprzetu)."""
from __future__ import annotations

import math
import os
import random
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinecone_bot.config import Config  # noqa: E402
from pinecone_bot.localize import Pose2D  # noqa: E402
from pinecone_bot.sim import SimClock, SimDrive  # noqa: E402
from pinecone_bot.zygzak import PoseTracker, Zygzak, plan_zigzag  # noqa: E402


def test_plan_zigzag_local_frame():
    wps = plan_zigzag(Pose2D(0, 0, 0), 3.0, 0.5, 3)
    assert wps == [(3.0, 0.0), (3.0, 0.5), (0.0, 0.5), (0.0, 1.0), (3.0, 1.0)]
    right = plan_zigzag(Pose2D(0, 0, 0), 3.0, 0.5, 2, left=False)
    assert right == [(3.0, 0.0), (3.0, -0.5), (0.0, -0.5)]


def test_plan_zigzag_rotated_and_shifted():
    wps = plan_zigzag(Pose2D(1.0, 2.0, math.pi / 2), 2.0, 1.0, 2)
    exp = [(1.0, 4.0), (0.0, 4.0), (0.0, 2.0)]   # pas na +y, skret w lewo = w strone -x
    for (x, y), (ex, ey) in zip(wps, exp):
        assert abs(x - ex) < 1e-9 and abs(y - ey) < 1e-9


def test_tracker_learns_speed_and_rejects_jump():
    cfg = Config()
    tr = PoseTracker(cfg)
    assert tr.fix(Pose2D(0, 0, 0), gyro=0.3)[0]
    assert abs(tr.yaw(0.3)) < 1e-12              # kurs mapy = zyroskop + offset
    for _ in range(100):                          # liczenie: 1 m
        tr.advance(0.1, 0.1, 0.3)
    assert abs(tr.x - 1.0) < 1e-9
    assert tr.fix(Pose2D(0.5, 0, 0), gyro=0.3)[0]  # naprawde tylko 0.5 m
    assert abs(tr.speed_scale - 0.75) < 1e-9       # pol drogi do 0.5
    ok, why = tr.fix(Pose2D(5.0, 5.0, 0), gyro=0.3)
    assert not ok and "odskok" in why
    assert (tr.x, tr.y) == (0.5, 0.0)


class SlowDrive(SimDrive):
    """Robot jedzie wolniej niz zadane (niezmierzone PWM -> m/s) i znosi go w bok."""

    def __init__(self, cfg, speed_gain=0.6):
        super().__init__(cfg, x=0.0, y=0.0, theta=0.0)
        self.speed_gain = speed_gain
        self.track = []

    def set_speed(self, v, w):
        super().set_speed(v, w)
        self.v *= self.speed_gain

    def advance(self, dt):
        super().advance(dt)
        self.track.append((self.x, self.y))


def _run(cfg, drive, look_ok=True, seed=1):
    rng = random.Random(seed)
    clock = SimClock(drive)

    def look(prior):
        if not look_ok and drive.track:
            return None
        return Pose2D(drive.x + rng.gauss(0, 0.03), drive.y + rng.gauss(0, 0.03),
                      drive.theta + rng.gauss(0, math.radians(1)), 100)

    z = Zygzak(cfg, drive, lambda: drive.theta, look, clock, log=lambda *a: None)
    return z.run(), z


def _cfg():
    cfg = Config()
    cfg.control.lane_count = 3
    cfg.control.lane_length_m = 3.0
    cfg.control.lane_spacing_m = 0.6
    cfg.sim.drift_w = 0.03      # rad/s znoszenia na prostej
    return cfg


def _min_dist(track, p):
    return min(math.hypot(x - p[0], y - p[1]) for x, y in track)


def test_zigzag_reaches_all_points_with_map_fixes():
    cfg = _cfg()
    drive = SlowDrive(cfg, speed_gain=0.6)
    res, z = _run(cfg, drive)
    wps = plan_zigzag(Pose2D(0, 0, 0), 3.0, 0.6, 3)
    assert res.reached == res.total == len(wps)
    for p in wps:
        assert _min_dist(drive.track, p) < 0.3
    assert 0.45 < z.tr.speed_scale < 0.75          # nauczyl sie, ze jedzie ~0.6 zadanego


def test_zigzag_without_fixes_misses_points():
    """Kontrola: bez zdjec (tylko liczenie z czasu) wolniejszy robot nie dojezdza do punktow."""
    cfg = _cfg()
    drive = SlowDrive(cfg, speed_gain=0.6)
    _run(cfg, drive, look_ok=False)
    wps = plan_zigzag(Pose2D(0, 0, 0), 3.0, 0.6, 3)
    assert max(_min_dist(drive.track, p) for p in wps) > 0.5


def test_no_localization_at_start_stops():
    cfg = _cfg()
    drive = SlowDrive(cfg)
    clock = SimClock(drive)
    z = Zygzak(cfg, drive, lambda: drive.theta, lambda prior: None, clock, log=lambda *a: None)
    res = z.run()
    assert res.total == 0 and res.looks == cfg.nav.look_retries + 1
    assert abs(drive.theta - cfg.nav.look_retries * math.radians(cfg.nav.look_turn_deg)) < 3 * math.radians(cfg.heading.tol_deg)


def test_missing_gyro_raises():
    cfg = _cfg()
    drive = SlowDrive(cfg)
    z = Zygzak(cfg, drive, lambda: None, lambda p: Pose2D(0, 0, 0), SimClock(drive), log=lambda *a: None)
    with pytest.raises(RuntimeError):
        z.run()
