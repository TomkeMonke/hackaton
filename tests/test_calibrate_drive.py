"""tools/calibrate_drive.py bez sprzetu: atrapa hovera jadacego na sciane i atrapa glebi."""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from calibrate_drive import Run, drive_run, fit_drive, wall_distance  # noqa: E402


class FakeRobot:
    """Martwa strefa do PWM 90, potem 0.004 m/s na PWM. Sciana 2.0 m przed kamera."""

    def __init__(self, wall=2.0):
        self.t = 0.0
        self.x = 0.0
        self.a = 0
        self.wall = wall
        self.sent = []

    def speed(self):
        mag = max(0.0, abs(self.a) - 90) * 0.004
        return mag if self.a > 0 else -mag

    def send(self, a):
        self.a = a
        self.sent.append(a)

    def sleep(self, dt):
        self.x += self.speed() * dt
        self.t += dt

    def clock(self):
        return self.t

    def read_depth(self):
        d = np.full((240, 424), self.wall - self.x, dtype=np.float32)
        d[:10, :] = 0.0     # troche dziur jak w prawdziwej glebi
        return d


def test_wall_distance_median_and_missing():
    d = np.full((240, 424), 1.5, dtype=np.float32)
    assert abs(wall_distance(d) - 1.5) < 1e-6
    assert wall_distance(np.zeros((240, 424), np.float32)) is None
    assert wall_distance(None) is None


def test_drive_run_measures_distance_and_returns_to_start():
    bot = FakeRobot()
    r = drive_run(bot.send, bot.read_depth, bot.sleep, bot.clock, 130, 2.0, 0.5, log=lambda *_: None)
    assert abs(r.speed - 0.16) < 0.01, r.speed          # (130 - 90) * 0.004
    assert abs(bot.x) < 0.02, bot.x                      # wrocil tylem
    assert bot.sent[-1] == 0


def test_drive_run_stops_before_wall():
    bot = FakeRobot(wall=1.1)
    r = drive_run(bot.send, bot.read_depth, bot.sleep, bot.clock, 250, 3.0, 0.5, log=lambda *_: None)
    assert bot.wall - max(bot.x, 0.0) >= 0.45            # nie wjechal w sciane (0.64 m/s * 3 s = 1.9 m)
    assert r is not None


def test_drive_run_refuses_when_too_close():
    bot = FakeRobot(wall=0.8)
    assert drive_run(bot.send, bot.read_depth, bot.sleep, bot.clock, 130, 2.0, 0.5, log=lambda *_: None) is None
    assert all(a == 0 for a in bot.sent)


def test_fit_drive_line_through_moving_runs():
    runs = [Run(80, 0.0, 0.0), Run(100, 0.08, 0.04), Run(130, 0.32, 0.16)]
    fit = fit_drive(runs, v_max=0.25)
    assert abs(fit.pwm_min - 90) <= 1
    assert abs(fit.pwm_max - (90 + 0.25 / 0.004)) <= 2
    assert fit_drive([Run(80, 0.0, 0.0), Run(100, 0.05, 0.025)], v_max=0.25) is None
