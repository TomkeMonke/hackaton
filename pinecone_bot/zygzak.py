"""
Zygzak po mapie: pasy jak kosiarka, z pozycja z mapy RTAB-Map zamiast liczenia z czasu.

Robot staje, robi zdjecie i lokalizuje sie w mapie (pinecone_bot/localize.py) -> (x, y, kurs). Z tego
miejsca plan: lane_count pasow po lane_length_m w kierunku, w ktorym stoi, odstep lane_spacing_m (pierwszy
skret w lewo). Do kazdego punktu: obrot w miejscu na namiar (petla obrotu na zyroskopie), jazda prosto
co najwyzej look_every_m z trzymaniem kursu, stop, zdjecie, poprawka pozycji. Kurs miedzy zdjeciami z
zyroskopu telefonu, droga z czasu; prawdziwa predkosc (speed_scale) uczy sie z kolejnych zdjec.
Nieudane zdjecie -> dalej na liczeniu, a gdy nie ma jeszcze zadnej pozycji: obrot o look_turn_deg i znowu.

Bez szyszek (sama jazda). Deterministycznie, bez ML.

  python -m pinecone_bot.zygzak --dry-run            # kamera i mapa prawdziwe, komendy jazdy tylko drukowane
  python -m pinecone_bot.zygzak --real               # jedzie (wylacznik w rece, kurs z telefonu)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

from .config import Config
from .localize import Pose2D


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def plan_zigzag(start: Pose2D, length: float, spacing: float, count: int,
                left: bool = True) -> list[tuple[float, float]]:
    """Punkty zygzaka w ukladzie mapy (bez punktu startu). Pas 0 z miejsca startu w kierunku start.yaw."""
    side = 1.0 if left else -1.0
    local = []
    for i in range(count):
        far = length if i % 2 == 0 else 0.0
        local.append((far, side * i * spacing))
        if i < count - 1:
            local.append((far, side * (i + 1) * spacing))
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return [(start.x + c * u - s * v, start.y + s * u + c * v) for u, v in local]


class PoseTracker:
    """Pozycja w mapie: ostatnie zdjecie + kurs z zyroskopu + droga z czasu (razy speed_scale)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.x = self.y = 0.0
        self.yaw_offset: float | None = None  # kurs mapy - kurs zyroskopu
        self.speed_scale = 1.0
        self._since_fix = 0.0                 # droga z liczenia od ostatniego zdjecia
        self._fix_xy: tuple[float, float] | None = None

    @property
    def known(self) -> bool:
        return self.yaw_offset is not None

    def yaw(self, gyro: float) -> float:
        return wrap(gyro + (self.yaw_offset or 0.0))

    def advance(self, v_cmd: float, dt: float, gyro: float) -> None:
        d = v_cmd * self.speed_scale * dt
        yaw = self.yaw(gyro)
        self.x += d * math.cos(yaw)
        self.y += d * math.sin(yaw)
        self._since_fix += d

    def fix(self, p: Pose2D, gyro: float) -> tuple[bool, str]:
        """Wpisz lokalizacje z mapy. False, gdy odskok od przewidywania > max_jump_m (odrzucona)."""
        n = self.cfg.nav
        if self.known:
            jump = math.hypot(p.x - self.x, p.y - self.y)
            if jump > n.max_jump_m:
                return False, "odskok %.2f m" % jump
            # prawdziwa predkosc: przejechane wg mapy / wg liczenia (tylko po dluzszym odcinku)
            if self._fix_xy is not None and self._since_fix > 0.3:
                real = math.hypot(p.x - self._fix_xy[0], p.y - self._fix_xy[1])
                ratio = real / (self._since_fix / self.speed_scale)
                new = 0.5 * self.speed_scale + 0.5 * ratio
                self.speed_scale = min(max(new, n.speed_scale_min), n.speed_scale_max)
        self.x, self.y = p.x, p.y
        self.yaw_offset = wrap(p.yaw - gyro)
        self._fix_xy = (p.x, p.y)
        self._since_fix = 0.0
        return True, ""


@dataclass
class ZigzagResult:
    reached: int = 0
    total: int = 0
    looks: int = 0
    fixes: int = 0
    rejected: int = 0
    path: list = field(default_factory=list)   # (x, y) po kazdym zdjeciu


class Zygzak:
    """
    base.set_speed/stop, gyro() -> kurs [rad] albo None, look() -> Pose2D|None (robot juz stoi),
    clock z now()/sleep() (SimClock w symulacji, WallClock na sprzecie).
    """

    def __init__(self, cfg: Config, base, gyro, look, clock, log=print):
        self.cfg, self.base, self.gyro, self.look_fn, self.clock, self.log = cfg, base, gyro, look, clock, log
        self.tr = PoseTracker(cfg)
        self.res = ZigzagResult()
        self.dt = 1.0 / max(cfg.control.loop_hz, 1.0)

    # --- klocki ---
    def _gyro(self) -> float:
        g = self.gyro()
        if g is None:
            raise RuntimeError("brak kursu z telefonu (phyphox na wierzchu, ekran wlaczony?)")
        return g

    def _stop_and_look(self) -> bool:
        self.base.stop()
        self.clock.sleep(self.cfg.nav.look_settle_s)
        self.res.looks += 1
        p = self.look_fn(Pose2D(self.tr.x, self.tr.y, self.tr.yaw(self._gyro())) if self.tr.known else None)
        g = self._gyro()
        if p is None:
            self.log("  zdjecie: brak lokalizacji, jade na liczeniu")
            return False
        ok, why = self.tr.fix(p, g)
        if not ok:
            self.res.rejected += 1
            self.log("  zdjecie: odrzucone (%s)" % why)
            return False
        self.res.fixes += 1
        self.res.path.append((p.x, p.y))
        self.log("  pozycja x=%.2f y=%.2f kurs=%.0f st (inliers %d, predkosc x%.2f)" % (
            p.x, p.y, math.degrees(p.yaw), p.inliers, self.tr.speed_scale))
        return True

    def _turn_to(self, bearing: float) -> None:
        c, h = self.cfg.control, self.cfg.heading
        tol = math.radians(h.tol_deg)
        t0 = self.clock.now()
        while True:
            err = wrap(bearing - self.tr.yaw(self._gyro()))
            if abs(err) <= tol:
                break
            if self.clock.now() - t0 > self.cfg.nav.turn_timeout_s:
                raise RuntimeError("obrot nie dochodzi do celu - sprawdz heading.sign i control.steer_sign")
            w = min(max(h.kp * err, -c.search_w), c.search_w)
            if abs(w) < h.w_min:
                w = math.copysign(h.w_min, err)
            self.base.set_speed(0.0, w * c.steer_sign)
            self.clock.sleep(self.dt)
        self.base.stop()

    def _drive(self, dist: float, bearing: float) -> None:
        """Prosto dist metrow (z liczenia), kurs trzymany na bearing."""
        c, h = self.cfg.control, self.cfg.heading
        done = 0.0
        v = c.search_drive_v
        while done < dist:
            g = self._gyro()
            err = wrap(bearing - self.tr.yaw(g))
            if abs(err) > math.radians(h.align_deg):
                self._turn_to(bearing)
                continue
            w = min(max(h.kp * err, -c.search_w), c.search_w)
            self.base.set_speed(v, w * c.steer_sign)
            self.clock.sleep(self.dt)
            self.tr.advance(v, self.dt, self._gyro())
            done += v * self.tr.speed_scale * self.dt
        self.base.stop()

    def _localize_start(self) -> bool:
        n = self.cfg.nav
        for i in range(n.look_retries + 1):
            if self._stop_and_look():
                return True
            if i < n.look_retries:
                self.log("  nie wiem, gdzie jestem - obrot o %.0f st i jeszcze raz" % n.look_turn_deg)
                g = self._gyro()
                self.tr.yaw_offset = 0.0
                self._turn_to(wrap(g + math.radians(n.look_turn_deg)))
                self.tr.yaw_offset = None
        return False

    # --- calosc ---
    def run(self) -> ZigzagResult:
        c, n = self.cfg.control, self.cfg.nav
        if not self._localize_start():
            self.log("STOP: robota nie ma w mapie (albo mapa/kamera nie ta)")
            return self.res
        start = Pose2D(self.tr.x, self.tr.y, self.tr.yaw(self._gyro()))
        wps = plan_zigzag(start, c.lane_length_m, c.lane_spacing_m, c.lane_count, n.first_turn_left)
        self.res.total = len(wps)
        self.log("zygzak: %d pasow po %.1f m, odstep %.2f m, pierwszy skret w %s, %d punktow" % (
            c.lane_count, c.lane_length_m, c.lane_spacing_m,
            "lewo" if n.first_turn_left else "prawo", len(wps)))
        for k, (wx, wy) in enumerate(wps):
            self.log("punkt %d/%d: (%.2f, %.2f)" % (k + 1, len(wps), wx, wy))
            for _ in range(50):
                dx, dy = wx - self.tr.x, wy - self.tr.y
                dist = math.hypot(dx, dy)
                if dist <= n.reach_tol_m:
                    break
                bearing = math.atan2(dy, dx)
                self._turn_to(bearing)
                self._drive(min(dist, n.look_every_m), bearing)
                self._stop_and_look()
            self.res.reached += 1
        self.base.stop()
        self.log("koniec: %d/%d punktow, zdjec %d, poprawek %d, odrzuconych %d" % (
            self.res.reached, self.res.total, self.res.looks, self.res.fixes, self.res.rejected))
        return self.res


# --- sprzet ---
def make_look(camera, loc, cfg: Config, save_dir: str | None = None):
    """look(prior) dla robota: swieza klatka z kamery (kilka pierwszych po postoju wyrzucane) -> lokalizacja."""
    counter = [0]

    def look(prior: Pose2D | None) -> Pose2D | None:
        frame = None
        for _ in range(3):
            frame, _depth = camera.read()
        if frame is None:
            return None
        p = loc.localize(frame, prior=prior, radius_m=cfg.nav.localize_radius_m)
        if p is None and prior is not None:
            p = loc.localize(frame)  # moze prior byl zly: cala mapa
        if save_dir:
            import os
            import cv2
            counter[0] += 1
            os.makedirs(save_dir, exist_ok=True)
            cv2.imwrite(os.path.join(save_dir, "look_%03d.jpg" % counter[0]), frame)
        return p

    return look


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="komendy jazdy tylko drukowane")
    mode.add_argument("--real", action="store_true")
    ap.add_argument("--config", default=None)
    ap.add_argument("--map", default=None, help="map_features.npz (nadpisuje nav.map_features)")
    ap.add_argument("--lanes", type=int, default=None, help="liczba pasow (nadpisuje control.lane_count)")
    ap.add_argument("--length", type=float, default=None, help="dlugosc pasa m (control.lane_length_m)")
    ap.add_argument("--spacing", type=float, default=None, help="odstep pasow m (control.lane_spacing_m)")
    ap.add_argument("--first-turn", choices=["left", "right"], default=None,
                    help="strona pierwszego skretu: z lewego dolnego rogu 'right', z prawego 'left'")
    ap.add_argument("--save-looks", default="frames/zygzak", help="zapis zdjec z postojow ('' = nie)")
    args = ap.parse_args(argv)

    import os
    from .brain import WallClock
    from .camera import make_camera
    from .heading import make_heading
    from .localize import MapLocalizer
    from .main import make_devices
    from .turn_loop import wrap_with_turn_loop

    cfg = Config.load(args.config)
    if args.lanes is not None:
        cfg.control.lane_count = args.lanes
    if args.length is not None:
        cfg.control.lane_length_m = args.length
    if args.spacing is not None:
        cfg.control.lane_spacing_m = args.spacing
    if args.first_turn is not None:
        cfg.nav.first_turn_left = args.first_turn == "left"
    cfg.heading.source = "phyphox"
    # mapa nagrana z automatyczna ekspozycja; zamrozona na starcie (np. w cieniu) przepala obraz na sloncu
    cfg.camera.lock_auto = False
    path = os.path.expanduser(args.map or cfg.nav.map_features)
    print("mapa:", path)
    loc = MapLocalizer.load(path)
    print("  %d klatek kluczowych" % len(loc.ids))

    camera = make_camera(cfg)
    base, _arm = make_devices(cfg, dry=args.dry_run, no_arm=True)
    heading = make_heading(cfg, base)
    t0 = time.monotonic()
    while heading.yaw() is None and time.monotonic() - t0 < 5.0:
        time.sleep(0.1)
    if heading.yaw() is None:
        print("brak kursu z telefonu - phyphox na wierzchu, 'Allow remote access', ekran wlaczony")
        heading.close()
        camera.close()
        base.close()
        return 2
    if not args.dry_run:
        base = wrap_with_turn_loop(base, heading, cfg)
    z = Zygzak(cfg, base, heading.yaw, make_look(camera, loc, cfg, args.save_looks or None), WallClock())
    try:
        z.run()
    except KeyboardInterrupt:
        print("przerwane")
    except RuntimeError as e:
        print("STOP:", e)
    finally:
        base.stop()
        base.close()
        heading.close()
        camera.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
