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
