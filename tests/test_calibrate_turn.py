"""tools/calibrate_turn.py bez sprzetu: atrapa hovera z martwa strefa i atrapa zyroskopu."""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from calibrate_turn import Step, apply_fit, fit_turn, sweep  # noqa: E402

from pinecone_bot.config import Config  # noqa: E402


class FakeHover:
    """Martwa strefa do 140, potem 0.028 rad/s na jednostke b (jak zmierzone: 150 -> ~0.2, 200 -> ~1.6)."""

    def __init__(self, sign=1.0):
        self.t = 0.0
        self.yaw_ = 0.0
        self.b = 0
        self.sign = sign
        self.sent = []

    def rate(self):
        mag = abs(self.b)
        r = 0.0 if mag <= 140 else (mag - 140) * 0.028
        return self.sign * (r if self.b < 0 else -r)   # ujemne b = w lewo = kurs rosnie

    def send(self, b):
        self.b = b
        self.sent.append(b)

    def sleep(self, dt):
        self.yaw_ += self.rate() * dt
        self.t += dt

    def yaw(self):
        return self.yaw_


def run(hover, **kw):
    args = dict(step_s=1.5, spinup_s=0.5, rest_s=1.0, max_rate=1.2, log=lambda *_: None)
    args.update(kw)
    return sweep(hover.send, hover.yaw, hover.sleep, list(range(100, 261, 10)), **args)


def test_sweep_finds_dead_zone_and_stops_when_too_fast():
    hover = FakeHover()
    steps = run(hover)
    assert [s.pwm for s in steps][:5] == [100, 110, 120, 130, 140]
    assert all(abs(s.rate) < 1e-9 for s in steps if s.pwm <= 140)
    assert steps[-1].rate > 1.2 and steps[-1].pwm < 260     # przerwal, zanim doszedl do konca zakresu
    assert hover.sent[-1] == 0                              # zawsze konczy stopem


def test_fit_turn_interpolates_w_max():
    fit = fit_turn(run(FakeHover()), w_max=0.6)
    assert fit.steer_min == 150
    assert fit.reached_w_max
    assert abs(fit.steer_max - (140 + 0.6 / 0.028)) <= 1     # ~161
    assert fit.steer_sign == 1.0


def test_fit_turn_detects_inverted_steering():
    fit = fit_turn(run(FakeHover(sign=-1.0)), w_max=0.6)
    assert fit.steer_sign == -1.0


def test_fit_turn_none_when_robot_never_moves():
    assert fit_turn([Step(100, 0.0), Step(200, 0.01)], w_max=0.6) is None


def test_sweep_stops_motors_on_missing_heading():
    hover = FakeHover()
    try:
        sweep(hover.send, lambda: None, hover.sleep, [150], step_s=1.5, spinup_s=0.5, rest_s=1.0,
              max_rate=1.2, log=lambda *_: None)
    except RuntimeError:
        pass
    else:
        raise AssertionError("brak kursu powinien przerwac pomiar")
    assert hover.sent[-1] == 0


def test_apply_fit_writes_config_fields():
    cfg = Config()
    fit = fit_turn(run(FakeHover()), w_max=cfg.control.w_max)
    apply_fit(cfg, fit)
    assert cfg.base.xiao_steer_min == 150
    assert cfg.base.xiao_steer_max > cfg.base.xiao_steer_min
    assert cfg.control.steer_sign == 1.0
