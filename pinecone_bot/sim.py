"""
Symulator: szyszki na plaskiej ziemi, kamera pinhole pochylona w dol, robot na napedzie roznicowym.

Cel: cala petla sterowania (detektor -> regulator -> maszyna stanow -> chwyt) dziala na laptopie
bez sprzetu. Obraz jest prosty (brazowe elipsy na zielonym tle z szumem), ale geometria jest prawdziwa:
polozenie szyszki w obrazie wynika z jej polozenia na ziemi i z geometrii kamery. Dzieki temu
kalibracja 'target_row' w symulatorze wyglada dokladnie tak, jak na sprzecie.

Uklady:
  swiat:  x, y w metrach, theta w radianach (0 = wzdluz osi x, dodatni = w lewo / CCW)
  robot:  dx do przodu, dy w lewo, wzgledem srodka osi kol
  kamera: cam_forward_m przed srodkiem osi (ujemne = za), na wysokosci cam_height_m,
          pochylona o cam_pitch_deg w dol; obraz: x w prawo, y w dol
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

import cv2
import numpy as np

from .config import Config, Grasp

GREEN = (40, 140, 40)
BROWN = (30, 60, 110)


@dataclass
class Cone:
    x: float
    y: float
    alive: bool = True


class SimWorld:
    """Swiat + geometria kamery. Poze robota dostarcza `pose_fn()` -> (x, y, theta)."""

    def __init__(self, cfg: Config, pose_fn, seed: int | None = None, cones: list | None = None):
        self.cfg = cfg
        self.pose_fn = pose_fn
        s = cfg.sim
        rng = random.Random(cfg.sim.seed if seed is None else seed)
        if cones is None:
            cones = []
            while len(cones) < s.n_cones:
                x = rng.uniform(0.6, s.field_m)
                y = rng.uniform(-s.field_m / 2, s.field_m / 2)
                if all(math.hypot(x - c[0], y - c[1]) > 0.25 for c in cones):
                    cones.append((x, y))
        self.cones = [Cone(x, y) for x, y in cones]
        self.rng = np.random.default_rng(cfg.sim.seed if seed is None else seed)
        # bank gotowych klatek tla z szumem: losowanie szumu na kazda klatke bylo najdrozsza czescia symulacji
        w, h = cfg.image_w, cfg.image_h
        base = np.empty((h, w, 3), dtype=np.int16)
        base[:] = GREEN
        self._bg = [
            np.clip(base + self.rng.normal(0, 6, size=(h, w, 3)).astype(np.int16), 0, 255).astype(np.uint8)
            for _ in range(6)
        ]
        self.collected = 0
        self.grasp_attempts = 0
        self.failed_grasps = 0

    # --- geometria -------------------------------------------------------
    def to_robot(self, wx: float, wy: float) -> tuple[float, float]:
        x, y, th = self.pose_fn()
        ddx, ddy = wx - x, wy - y
        c, s = math.cos(-th), math.sin(-th)
        return ddx * c - ddy * s, ddx * s + ddy * c

    def project(self, dx: float, dy: float) -> tuple[float, float, float] | None:
        """Punkt na ziemi w ukladzie robota -> (px, py, z_cam). None, gdy za kamera."""
        s = self.cfg.sim
        pitch = math.radians(s.cam_pitch_deg)
        h = s.cam_height_m
        dxc = dx - s.cam_forward_m
        z_c = dxc * math.cos(pitch) + h * math.sin(pitch)
        if z_c <= 0.05:
            return None
        y_c = -dxc * math.sin(pitch) + h * math.cos(pitch)
        x_c = -dy
        px = self.cfg.image_w / 2 + s.fx * x_c / z_c
        py = self.cfg.image_h / 2 + s.fy * y_c / z_c
        return px, py, z_c

    def visible_cones(self) -> list[tuple[Cone, float, float, float]]:
        out = []
        for cone in self.cones:
            if not cone.alive:
                continue
            dx, dy = self.to_robot(cone.x, cone.y)
            p = self.project(dx, dy)
            if p is None:
                continue
            px, py, z = p
            if -50 <= px <= self.cfg.image_w + 50 and -50 <= py <= self.cfg.image_h + 50:
                out.append((cone, px, py, z))
        out.sort(key=lambda t: -t[3])  # dalsze najpierw, blizsze rysowane na wierzchu
        return out

    # --- obraz -----------------------------------------------------------
    def render(self) -> np.ndarray:
        bg = self._bg[int(self.rng.integers(len(self._bg)))]
        img = np.roll(bg, (int(self.rng.integers(0, 32)), int(self.rng.integers(0, 32))), axis=(0, 1))
        s = self.cfg.sim
        for _, px, py, z in self.visible_cones():
            r = max(2.0, s.fx * s.cone_radius_m / z)
            jx = self.rng.normal(0, s.noise_px)
            jy = self.rng.normal(0, s.noise_px)
            center = (int(round(px + jx)), int(round(py + jy)))
            axes = (int(round(r * 1.1)), int(round(r * 0.8)))
            cv2.ellipse(img, center, axes, 0, 0, 360, BROWN, -1)
        return img

    # --- chwyt -----------------------------------------------------------
    def grasp_spot(self, name: str) -> Grasp:
        for g in self.cfg.grasps:
            if g.name == name:
                return g
        raise KeyError(name)

    def try_grasp(self, name: str, tol_forward: float = 0.03, tol_lateral: float = 0.025) -> bool:
        """Chwyt 'name' udaje sie, gdy jakas szyszka lezy w jego strefie (w ukladzie robota)."""
        self.grasp_attempts += 1
        g = self.grasp_spot(name)
        for cone in self.cones:
            if not cone.alive:
                continue
            dx, dy = self.to_robot(cone.x, cone.y)
            if abs(dx - g.forward_m) <= tol_forward and abs(dy) <= tol_lateral:
                cone.alive = False
                self.collected += 1
                return True
        self.failed_grasps += 1
        return False

    def remaining(self) -> int:
        return sum(1 for c in self.cones if c.alive)


def calibrate_grasps(cfg: Config, world: SimWorld) -> None:
    """
    Symulowany odpowiednik tools/calibrate_target.py: dla kazdego nagranego chwytu
    policz, w ktorym wierszu obrazu lezy szyszka, gdy jest dokladnie w punkcie chwytu.
    Na sprzecie ten sam krok robi czlowiek: klade szyszke w punkcie chwytu i zapisuje (px, py).
    """
    for g in cfg.grasps:
        p = world.project(g.forward_m, 0.0)
        if p is None:
            raise ValueError(f"punkt chwytu {g.name} ({g.forward_m} m) jest poza kadrem kamery")
        px, py, _ = p
        g.target_row = float(py)
        cfg.cx = float(px)


class SimCamera:
    def __init__(self, world: SimWorld):
        self.world = world

    def read(self):
        return self.world.render(), None

    def close(self):
        pass


class SimClock:
    """
    Wirtualny zegar: sleep() przesuwa czas i calkuje ruch bazy. Petla sterowania
    uzywa tylko now() i sleep(), wiec ten sam kod chodzi na sprzecie z prawdziwym zegarem.
    """

    def __init__(self, base=None):
        self.t = 0.0
        self.base = base

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        dt = max(0.0, float(dt))
        if self.base is not None and hasattr(self.base, "advance"):
            self.base.advance(dt)
        self.t += dt


class SimDrive:
    """
    Minimalna baza symulowana (naped roznicowy). pinecone_bot.base.SimBase robi to samo;
    ta kopia istnieje, zeby symulator nie zalezal od sterownikow sprzetowych.
    """

    def __init__(self, cfg: Config, x: float = 0.0, y: float | None = None, theta: float = 0.0):
        self.cfg = cfg
        # domyslnie start w rogu pola: pasy (skret w lewo) pokrywaja wtedy cale pole
        if y is None:
            y = -cfg.sim.field_m / 2 + 0.2
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0
        self.w = 0.0

    def set_speed(self, v_mps: float, w_radps: float) -> None:
        c = self.cfg.control
        self.v = min(max(v_mps, c.v_min), c.v_max)
        self.w = min(max(w_radps, -c.w_max), c.w_max)

    def stop(self) -> None:
        self.v = 0.0
        self.w = 0.0

    def odometry(self):
        return self.x, self.y, self.theta

    def advance(self, dt: float) -> None:
        s = self.cfg.sim
        drift = s.drift_w if abs(self.v) > 1e-6 else 0.0
        self.theta += (self.w * s.turn_gain + drift) * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

    def close(self) -> None:
        pass


class SimXiaoDrive(SimDrive):
    """
    Hover przez Xiao, jak zmierzony (cfg.sim.hover_*): skret to PWM b, nie rad/s.
    Robot stojacy rusza dopiero od |b| >= hover_static_pwm, krecacy sie kreci dalej od hover_kinetic_pwm,
    powyzej predkosc rosnie o hover_rate_per_pwm na jednostke b; silniki z opoznieniem hover_tau_s.
    W czasie jazdy do przodu kola juz sie tocza, wiec maly skret dziala bez martwej strefy.
    set_speed mapuje w -> b tak jak XiaoBase (xiao_steer_min/max z configu); set_raw podaje b wprost.
    Predkosc do przodu bez modelu: speed_pwm w symulacji = mm/s.
    """

    ROLL_RATE_PER_PWM = 0.006   # rad/s na jednostke b w czasie jazdy do przodu

    def __init__(self, cfg: Config, **kw):
        super().__init__(cfg, **kw)
        self.t = 0.0
        self.b = 0
        self.w_act = 0.0
        self.spinning = False
        self.history: list[tuple[float, float]] = [(0.0, self.theta)]

    def pwm_for(self, v: float, w: float) -> tuple[int, int]:
        from .base import XIAO_STEER_LIMIT, xiao_map
        c, b = self.cfg.control, self.cfg.base
        v = min(max(v, c.v_min), c.v_max)
        w = min(max(w, -c.w_max), c.w_max)
        return int(round(v * 1000)), -xiao_map(w, c.w_max, b.xiao_steer_min, b.xiao_steer_max, XIAO_STEER_LIMIT)

    def set_speed(self, v_mps: float, w_radps: float) -> None:
        speed, steer = self.pwm_for(v_mps, w_radps)
        self.set_raw(speed, steer)

    def set_raw(self, speed_pwm: int, steer_pwm: int) -> None:
        self.v = speed_pwm / 1000.0
        self.b = int(steer_pwm)
        self.w = 0.0   # nieuzywane; faktyczny obrot w w_act

    def stop(self) -> None:
        self.set_raw(0, 0)

    def _target_rate(self) -> float:
        s = self.cfg.sim
        mag = abs(self.b)
        if abs(self.v) > 0.02:
            r = mag * self.ROLL_RATE_PER_PWM
        else:
            if not self.spinning and mag >= s.hover_static_pwm:
                self.spinning = True
            if self.spinning and mag < s.hover_kinetic_pwm:
                self.spinning = False
            r = max(0.0, mag - s.hover_kinetic_pwm) * s.hover_rate_per_pwm if self.spinning else 0.0
        return -math.copysign(r, self.b) if self.b else 0.0   # ujemne b = w lewo = kurs rosnie

    def advance(self, dt: float) -> None:
        s = self.cfg.sim
        left = float(dt)
        while left > 1e-9:
            h = min(0.01, left)
            target = self._target_rate()
            self.w_act += (target - self.w_act) * min(1.0, h / max(s.hover_tau_s, 1e-3))
            if target == 0.0 and abs(self.w_act) < 0.02:
                self.w_act = 0.0
                if abs(self.v) <= 0.02:
                    self.spinning = False
            drift = s.drift_w if abs(self.v) > 1e-6 else 0.0
            self.theta += (self.w_act + drift) * h
            self.x += self.v * math.cos(self.theta) * h
            self.y += self.v * math.sin(self.theta) * h
            self.t += h
            left -= h
        self.history.append((self.t, self.theta))
        if len(self.history) > 2000:
            del self.history[:1000]


class SimGyro:
    """Telefon: kurs z opoznieniem gyro_delay_s, odswiezany co 1/poll_hz (jak odpytywanie phyphox)."""

    def __init__(self, drive: SimXiaoDrive, cfg: Config):
        self.drive = drive
        self.delay = cfg.sim.gyro_delay_s
        self.hz = cfg.heading.poll_hz
        self.alive = True

    def yaw(self) -> float | None:
        if not self.alive:
            return None
        tq = self.drive.t - self.delay
        tq = math.floor(tq * self.hz) / self.hz
        hist = self.drive.history
        for t, th in reversed(hist):
            if t <= tq:
                return th
        return hist[0][1]

    def close(self) -> None:
        pass


def build_sim_robot(cfg: Config, cones: list | None = None):
    """
    (base, world, clock, heading) dla symulacji. Bez cfg.sim.hover: idealny naped i idealny kurs.
    Z hover: SimXiaoDrive + SimGyro, a gdy jest kurs i heading.rate_loop - petla obrotu na zyroskopie.
    """
    from .heading import OdometryHeading
    from .turn_loop import wrap_with_turn_loop

    if not cfg.sim.hover:
        drive = SimDrive(cfg)
        heading = OdometryHeading(drive) if cfg.heading.source != "none" else None
        base = drive
    else:
        drive = SimXiaoDrive(cfg)
        heading = SimGyro(drive, cfg) if cfg.heading.source != "none" else None
        base = wrap_with_turn_loop(drive, heading, cfg, clock=lambda: drive.t, thread=False)
    world = SimWorld(cfg, drive.odometry, cones=cones)
    return base, world, SimClock(base), heading


class SimArmSimple:
    """Ramie w symulacji: czeka tyle, ile trwa ruch, i pyta swiat, czy szyszka byla w strefie."""

    def __init__(self, world: SimWorld, clock: SimClock, motion_seconds: float = 4.0):
        self.world = world
        self.clock = clock
        self.motion_seconds = motion_seconds
        self.calls: list[str] = []

    def replay(self, name: str):
        self.calls.append(name)
        self.clock.sleep(self.motion_seconds)
        if name.startswith("grasp"):
            return self.world.try_grasp(name)
        return None

    def home(self) -> None:
        self.calls.append("home")
        self.clock.sleep(1.0)

    def close(self) -> None:
        pass
