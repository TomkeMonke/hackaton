"""Petla predkosci obrotu na zyroskopie (pinecone_bot/turn_loop.py) na modelu hovera z symulatora."""
from __future__ import annotations

import math

from pinecone_bot.brain import Brain, State
from pinecone_bot.config import Config
from pinecone_bot.detector import HsvConeDetector
from pinecone_bot.sim import SimArmSimple, SimCamera, SimGyro, SimXiaoDrive, build_sim_robot, calibrate_grasps
from pinecone_bot.turn_loop import GyroTurnBase, TurnRateLoop


def hover_cfg(**sim):
    cfg = Config()
    cfg.sim.hover = True
    cfg.heading.source = "phyphox"
    for k, v in sim.items():
        setattr(cfg.sim, k, v)
    return cfg


def loop_rig(cfg):
    drive = SimXiaoDrive(cfg)
    gyro = SimGyro(drive, cfg)
    base = GyroTurnBase(drive, gyro, cfg, clock=lambda: drive.t, thread=False)
    return drive, gyro, base


def spin(base, drive, w, seconds, dt=1 / 15):
    rates, bs = [], []
    t_end = drive.t + seconds
    while drive.t < t_end:
        base.set_speed(0.0, w)
        base.advance(dt)
        rates.append(drive.w_act)
        bs.append(drive.b)
    return rates, bs


def test_breaks_static_friction_and_holds_rate():
    cfg = hover_cfg()
    drive, _, base = loop_rig(cfg)
    rates, bs = spin(base, drive, 0.35, 6.0)
    moving_at = next(i for i, r in enumerate(rates) if r > 0.05) / 15
    assert moving_at < 1.5, moving_at                       # ruszyl mimo tarcia statycznego 155
    tail = rates[-45:]                                      # ostatnie 3 s
    assert abs(sum(tail) / len(tail) - 0.35) < 0.12, sum(tail) / len(tail)
    assert max(abs(b) for b in bs) <= cfg.heading.rate_pwm_max
    assert all(b <= 0 for b in bs)                          # w lewo = ujemne b (XiaoBase)


def test_right_turn_uses_positive_pwm():
    cfg = hover_cfg()
    drive, _, base = loop_rig(cfg)
    rates, bs = spin(base, drive, -0.35, 4.0)
    assert min(rates) < -0.2 and all(b >= 0 for b in bs)


def test_never_exceeds_pwm_max_when_robot_cannot_move():
    cfg = hover_cfg(hover_static_pwm=500.0)               # zablokowany robot
    drive, _, base = loop_rig(cfg)
    rates, bs = spin(base, drive, 0.35, 5.0)
    assert max(rates) == 0.0
    assert max(abs(b) for b in bs) == cfg.heading.rate_pwm_max


def test_stop_sends_zero_and_resets():
    cfg = hover_cfg()
    drive, _, base = loop_rig(cfg)
    spin(base, drive, 0.35, 3.0)
    base.stop()
    assert drive.b == 0 and base.loop.u == 0.0


def test_falls_back_to_open_loop_mapping_without_heading():
    cfg = hover_cfg()
    drive, gyro, base = loop_rig(cfg)
    gyro.alive = False
    base.set_speed(0.0, 0.35)
    assert not base.closed_loop
    assert drive.b == drive.pwm_for(0.0, 0.35)[1]           # to samo co XiaoBase z configu


def test_rolling_corrections_start_small():
    """Jazda do przodu: kola sie tocza, korekta kursu nie moze startowac od progu ruszania w miejscu."""
    loop = TurnRateLoop(Config().heading)
    b = loop.update(0.05, 0.0, 0.0, rolling=True)
    assert abs(b) < 10


def run_lanes_hover(rate_loop: bool):
    cfg = hover_cfg()
    cfg.heading.rate_loop = rate_loop
    base, world, clock, heading = build_sim_robot(cfg, cones=[])
    drive = world.pose_fn.__self__
    x0, y0 = drive.x, drive.y
    calibrate_grasps(cfg, world)
    brain = Brain(cfg, SimCamera(world), HsvConeDetector(cfg.detector), base, SimArmSimple(world, clock),
                  clock=clock, verbose=False, heading=heading)
    brain.run(max_seconds=600)
    c = cfg.control
    end = math.hypot(drive.x - x0, drive.y - (y0 + c.lane_count * c.lane_spacing_m))
    return brain, end


def test_lanes_on_hover_model_with_loop():
    brain, end = run_lanes_hover(True)
    assert brain.state == State.DONE
    assert end < 0.3, end
    assert not any("bez kursu" in t[3] for t in brain.stats.transitions)


def test_lanes_on_hover_model_without_loop_do_not_turn():
    """Kontrola: mapowanie z configu (xiao_steer_min 60) jest za slabe na tarcie hovera - robot sie nie obraca."""
    brain, end = run_lanes_hover(False)
    assert end > 5.0, end
