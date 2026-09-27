"""Testy tools/drive_calib.py - atrapa swiata: robot przed sciana, Xiao z tarciem, zyroskop phyphox."""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from drive_calib import Calibrator, apply, fit_pwm, signs_from_turn, wall_distance  # noqa: E402
from pinecone_bot.base import XIAO_SPEED_LIMIT, XIAO_STEER_LIMIT, xiao_map  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402

# "prawdziwe" podwozie: pwm = P0 + S * v (ponizej P0 kola stoja)
P0, S = 100, 800.0
STEER_P0, STEER_S = 50, 600.0


class World:
    def __init__(self, cfg, wall_m=2.0, steer_inverted=False, phone_face_down=False):
        self.cfg, self.wall_m = cfg, wall_m
        self.steer_inverted, self.phone_face_down = steer_inverted, phone_face_down
        self.x = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_turn_left = None

    # base
    def set_speed(self, v, w):
        c, b = self.cfg.control, self.cfg.base
        pwm = xiao_map(v, c.v_max, b.xiao_pwm_min, b.xiao_pwm_max, XIAO_SPEED_LIMIT)
        spwm = xiao_map(w, c.w_max, b.xiao_steer_min, b.xiao_steer_max, XIAO_STEER_LIMIT)
        self.v = math.copysign(max(0.0, abs(pwm) - P0) / S, pwm)
        self.w = math.copysign(max(0.0, abs(spwm) - STEER_P0) / STEER_S, spwm) * (-1 if self.steer_inverted else 1)
        if self.w:
            self.last_turn_left = self.w > 0

    def stop(self):
        self.v = self.w = 0.0

    def sleep(self, dt):
        self.x += self.v * dt
        self.yaw += self.w * dt

    # gyro: telefon ekranem w dol odwraca znak
    def yaw_reading(self):
        return self.yaw * (-1 if self.phone_face_down else 1) * self.cfg.heading.sign

    def depth(self):
        return np.full((48, 64), self.wall_m - self.x, np.float32)


def calibrate(world, speeds=(0.10, 0.20), rates=(0.3, 0.5)):
    gyro = type("G", (), {"yaw": lambda _self: world.yaw_reading()})()
    cal = Calibrator(world.cfg, world, gyro, world.depth, ask=lambda _p: world.last_turn_left,
                     sleep=world.sleep, log=lambda *_: None)
    return cal.run(list(speeds), 2.0, list(rates), 2.0)


def test_good_hardware_keeps_signs_and_recovers_pwm_line():
    cfg = Config()
    world = World(cfg)
    res = calibrate(world)
    assert res["steer_sign"] == 1.0 and res["heading_sign"] == 1.0
    pmin, pmax = res["xiao_pwm"]
    assert abs(pmin - P0) <= 3 and abs(pmax - (P0 + S * cfg.control.v_max)) <= 3
    smin, smax = res["xiao_steer"]
    assert abs(smin - STEER_P0) <= 3 and abs(smax - (STEER_P0 + STEER_S * cfg.control.w_max)) <= 3
    assert abs(res["forward"][0]["drift_deg"]) < 0.1
    assert abs(world.yaw) < math.radians(15)  # obroty na zmiane: robot wraca mniej wiecej do kierunku


def test_inverted_steering_and_face_down_phone_flip_both_signs():
    cfg = Config()
    res = calibrate(World(cfg, steer_inverted=True, phone_face_down=True))
    assert res["steer_sign"] == -1.0
    assert res["heading_sign"] == -1.0


def test_inverted_steering_with_good_phone_flips_only_steer():
    cfg = Config()
    res = calibrate(World(cfg, steer_inverted=True))
    assert res["steer_sign"] == -1.0 and res["heading_sign"] == 1.0


def test_signs_from_turn_table():
    assert signs_from_turn(0.3, +0.5, True, 1.0) == (1.0, 1.0)
    assert signs_from_turn(0.3, -0.5, True, 1.0) == (1.0, -1.0)
    assert signs_from_turn(0.3, -0.5, False, 1.0) == (-1.0, 1.0)
    assert signs_from_turn(-0.3, +0.5, True, -1.0) == (-1.0, -1.0)


def test_wheels_not_moving_are_excluded_from_fit():
    cfg = Config()
    res = calibrate(World(cfg), speeds=(0.001, 0.10, 0.20))
    assert res["forward"][0]["dist_m"] == 0.0
    assert res["xiao_pwm"] is not None  # dopasowane z dwoch pozostalych


def test_fit_pwm_rejects_single_point_and_nonsense():
    assert fit_pwm([(150, 0.06)], 0.25) is None
    assert fit_pwm([(150, 0.10), (200, 0.05)], 0.25) is None  # wiecej PWM = wolniej
    assert fit_pwm([(180, 0.10), (260, 0.20)], 0.25) == (100, 300)


def test_wall_distance_median_and_holes():
    d = np.full((48, 64), 1.5, np.float32)
    assert wall_distance([d] * 5) == pytest.approx(1.5)
    assert wall_distance([np.zeros((48, 64), np.float32)] * 5) is None


def test_too_close_to_wall_stops_before_driving():
    cfg = Config()
    world = World(cfg, wall_m=0.5)
    with pytest.raises(RuntimeError, match="za blisko"):
        calibrate(world)
    assert world.x == 0.0


def test_apply_writes_only_measured_fields():
    cfg = Config()
    res = {"steer_sign": -1.0, "heading_sign": None, "xiao_pwm": (95, 290), "xiao_steer": None}
    changes = apply(cfg, res)
    assert cfg.control.steer_sign == -1.0 and cfg.heading.sign == 1.0
    assert (cfg.base.xiao_pwm_min, cfg.base.xiao_pwm_max) == (95, 290)
    assert len(changes) == 3
