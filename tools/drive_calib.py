"""
Kalibracja jazdy bez miarki: droga z glebi RealSense, kat z zyroskopu telefonu (phyphox).

    python tools/drive_calib.py                    # 2x prosto + 2x obrot, raport, nic nie zapisuje
    python tools/drive_calib.py --write            # to samo + wpis do pinecone_config.json

Przygotowanie:
- Kamera patrzy do przodu na sciane / pudlo 1-3 m przed robotem (srodek kadru), ramie nieruchome.
  Przed robotem wolne miejsce na oba przejazdy (domyslnie 0.2 + 0.4 m).
- Telefon plasko na bazie, ekranem do gory, phyphox "Gyroscope (rotation rate)" z "Allow remote access".
- Wylacznik w rece. web_control.py wylaczony (trzyma port Xiao).

Co mierzy (fazy czasowe, open-loop, jak tools/base_test.py; komendy SUROWE, bez control.steer_sign):
- prosto z --speeds: droga = glebia przed - glebia po, znos kursu z zyroskopu;
- obroty z --turn-rates (na zmiane w lewo i w prawo, robot wraca do kierunku): kat z zyroskopu,
  po pierwszym pytanie do operatora, w ktora strone sie obrocil (zyroskop sam nie odrozni
  zlego steer_sign od zlego heading.sign).
Z dwoch punktow PWM -> predkosc dopasowuje prosta pwm = p0 + s*v, stad xiao_pwm_min/max
(i xiao_steer_min/max dla obrotow) przy obecnym control.v_max / w_max.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pinecone_bot.base import XIAO_SPEED_LIMIT, XIAO_STEER_LIMIT, xiao_map  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402

SETTLE_S = 1.0          # po zatrzymaniu: robot sie toczy, zyroskop dobiera probki
MIN_MOVE_M = 0.02       # ponizej tego uznajemy, ze kola nie ruszyly (tarcie statyczne)
MIN_TURN_DEG = 3.0
MAX_WALL_M = 3.5        # dalej glebia D4xx jest za szumna na pomiar cm
MIN_WALL_M = 0.4        # blizej glebia D4xx ma dziury (MinZ ~0.3 m przy 640x480)
ROI_FRAC = 0.2          # srodkowy kwadrat kadru (20% szerokosci i wysokosci)
MIN_VALID_FRAC = 0.3


def wall_distance(depth_frames, roi_frac: float = ROI_FRAC) -> float | None:
    """Mediana glebi [m] ze srodka kadru po wielu klatkach; None, gdy za malo pikseli z glebia."""
    import numpy as np

    meds = []
    for d in depth_frames:
        if d is None:
            continue
        h, w = d.shape[:2]
        rh, rw = max(1, int(h * roi_frac / 2)), max(1, int(w * roi_frac / 2))
        roi = d[h // 2 - rh:h // 2 + rh, w // 2 - rw:w // 2 + rw]
        valid = roi[roi > 0.15]
        if valid.size >= MIN_VALID_FRAC * roi.size:
            meds.append(float(np.median(valid)))
    if len(meds) < max(1, len(depth_frames) // 2):
        return None
    return float(np.median(meds))


def fit_pwm(points: list[tuple[int, float]], full_scale: float) -> tuple[int, int] | None:
    """(pwm, zmierzona predkosc) -> (pwm_min, pwm_max) dla xiao_map przy full_scale.

    Prosta pwm = p0 + s*v najmniejszymi kwadratami. None, gdy za malo punktow albo wynik bez sensu.
    """
    pts = [(abs(p), abs(v)) for p, v in points if abs(v) > 1e-6]
    if len(pts) < 2 or len({v for _, v in pts}) < 2:
        return None
    n = len(pts)
    mv = sum(v for _, v in pts) / n
    mp = sum(p for p, _ in pts) / n
    s = sum((v - mv) * (p - mp) for p, v in pts) / sum((v - mv) ** 2 for _, v in pts)
    p0 = mp - s * mv
    if s <= 0 or p0 < 0:
        return None
    return int(round(p0)), int(round(p0 + s * full_scale))


def signs_from_turn(w_cmd: float, dyaw: float, turned_left: bool, heading_sign: float) -> tuple[float, float]:
    """(control.steer_sign, heading.sign) z jednego obrotu.

    w_cmd > 0 = SUROWA komenda w lewo (bez steer_sign). dyaw = zmiana kursu z zyroskopu
    przy obecnym heading.sign. Obrot w lewo ma dawac dodatni kurs.
    """
    steer_sign = 1.0 if (w_cmd > 0) == turned_left else -1.0
    gyro_ok = (dyaw > 0) == turned_left
    return steer_sign, heading_sign if gyro_ok else -heading_sign


class Calibrator:
    def __init__(self, cfg: Config, base, gyro, read_depth, ask, sleep=time.sleep, log=print):
        self.cfg, self.base, self.gyro = cfg, base, gyro
        self.read_depth, self.ask, self.sleep, self.log = read_depth, ask, sleep, log

    def _yaw(self) -> float:
        for _ in range(30):
            y = self.gyro.yaw()
            if y is not None:
                return y
            self.sleep(0.1)
        raise RuntimeError("brak danych z phyphox (telefon, Allow remote access, ekran wlaczony?)")

    def _wall(self, n: int = 10) -> float:
        d = wall_distance([self.read_depth() for _ in range(n)])
        if d is None or d > MAX_WALL_M:
            raise RuntimeError("brak sciany przed kamera (srodek kadru bez glebi albo dalej niz %.1f m)" % MAX_WALL_M)
        return d

    def _drive(self, v: float, w: float, seconds: float) -> None:
        self.base.set_speed(v, w)
        try:
            self.sleep(seconds)
        finally:
            self.base.stop()
        self.sleep(SETTLE_S)

    def forward(self, v: float, seconds: float) -> dict:
        c, b = self.cfg.control, self.cfg.base
        pwm = xiao_map(v, c.v_max, b.xiao_pwm_min, b.xiao_pwm_max, XIAO_SPEED_LIMIT)
        d0, y0 = self._wall(), self._yaw()
        if d0 - 1.5 * v * seconds < MIN_WALL_M:
            raise RuntimeError("za blisko sciany (%.2f m) na przejazd %.2f m - odsun robota" % (d0, v * seconds))
        self._drive(v, 0.0, seconds)
        d1, y1 = self._wall(), self._yaw()
        dist = d0 - d1
        r = {"v_cmd": v, "pwm": pwm, "seconds": seconds, "wall_before_m": round(d0, 3),
             "wall_after_m": round(d1, 3), "dist_m": round(dist, 3), "v_real": round(dist / seconds, 4),
             "drift_deg": round(math.degrees(y1 - y0), 1)}
        self.log("[prosto v=%.2f pwm=%d] %.3f m -> %.3f m: przejechal %.3f m (%.3f m/s), znos kursu %+.1f st"
                 % (v, pwm, d0, d1, dist, r["v_real"], r["drift_deg"]))
        if dist < -MIN_MOVE_M:
            self.log("  UWAGA: sciana sie ODDALILA - robot cofal (odwrocony kierunek jazdy w Xiao?) albo kamera patrzy do tylu")
        elif abs(dist) < MIN_MOVE_M:
            self.log("  kola nie ruszyly: pwm %d ponizej tarcia statycznego" % pwm)
        return r

    def turn(self, w: float, seconds: float) -> dict:
        c, b = self.cfg.control, self.cfg.base
        pwm = abs(xiao_map(w, c.w_max, b.xiao_steer_min, b.xiao_steer_max, XIAO_STEER_LIMIT))
        y0 = self._yaw()
        self._drive(0.0, w, seconds)
        dyaw = self._yaw() - y0
        r = {"w_cmd": w, "pwm": pwm, "seconds": seconds, "dyaw_deg": round(math.degrees(dyaw), 1),
             "w_real": round(abs(dyaw) / seconds, 4)}
        self.log("[obrot w=%+.2f pwm=%d] kurs %+.1f st (%.3f rad/s)" % (w, pwm, r["dyaw_deg"], r["w_real"]))
        return r

    def run(self, speeds, seconds, turn_rates, turn_seconds) -> dict:
        fwd = [self.forward(v, seconds) for v in speeds]
        turns, steer_sign, heading_sign = [], None, None
        for i, w in enumerate(turn_rates):
            w = abs(w) if i % 2 == 0 else -abs(w)   # lewo, prawo, lewo...: wraca do kierunku startu
            r = self.turn(w, turn_seconds)
            turns.append(r)
            if steer_sign is None and abs(r["dyaw_deg"]) >= MIN_TURN_DEG:
                left = self.ask("W ktora strone obrocil sie robot (patrzac z gory)? [l/p]: ")
                steer_sign, heading_sign = signs_from_turn(w, math.radians(r["dyaw_deg"]), left,
                                                           self.cfg.heading.sign)
        c = self.cfg.control
        moved = [r for r in fwd if r["dist_m"] > MIN_MOVE_M]
        rotated = [r for r in turns if abs(r["dyaw_deg"]) >= MIN_TURN_DEG]
        return {
            "forward": fwd, "turns": turns,
            "steer_sign": steer_sign, "heading_sign": heading_sign,
            "xiao_pwm": fit_pwm([(r["pwm"], r["v_real"]) for r in moved], c.v_max),
            "xiao_steer": fit_pwm([(r["pwm"], r["w_real"]) for r in rotated], c.w_max),
        }


def apply(cfg: Config, res: dict) -> list[str]:
    """Wpisuje wyniki do cfg. Zwraca liste zmian (tylko pola, ktore udalo sie zmierzyc)."""
    changes = []

    def put(obj, name, value, label):
        old = getattr(obj, name)
        if value is not None and old != value:
            setattr(obj, name, value)
            changes.append("%s: %s -> %s" % (label, old, value))

    put(cfg.control, "steer_sign", res["steer_sign"], "control.steer_sign")
    put(cfg.heading, "sign", res["heading_sign"], "heading.sign")
    if res["xiao_pwm"]:
        put(cfg.base, "xiao_pwm_min", res["xiao_pwm"][0], "base.xiao_pwm_min")
        put(cfg.base, "xiao_pwm_max", res["xiao_pwm"][1], "base.xiao_pwm_max")
    if res["xiao_steer"]:
        put(cfg.base, "xiao_steer_min", res["xiao_steer"][0], "base.xiao_steer_min")
        put(cfg.base, "xiao_steer_max", res["xiao_steer"][1], "base.xiao_steer_max")
    return changes


def ask_left(prompt: str) -> bool:
    while True:
        a = input(prompt).strip().lower()
        if a in ("l", "p"):
            return a == "l"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None)
    p.add_argument("--port", default=None, help="port Xiao (domyslnie cfg.base.port)")
    p.add_argument("--speeds", type=float, nargs="+", default=[0.10, 0.20], help="m/s, >= 2 do dopasowania PWM")
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--turn-rates", type=float, nargs="+", default=[0.3, 0.5], help="rad/s, >= 2")
    p.add_argument("--turn-seconds", type=float, default=2.0)
    p.add_argument("--write", action="store_true", help="zapisz wyniki do pliku configu")
    p.add_argument("--yes", action="store_true", help="bez potwierdzenia startu")
    args = p.parse_args(argv)

    cfg = Config.load(args.config)
    cfg.base.driver = "xiao"
    if args.port:
        cfg.base.port = args.port
    need = sum(args.speeds) * args.seconds
    print("Robot pojedzie prosto ok. %.2f m (+ zapas), potem obroty w miejscu. Wylacznik w rece." % need)
    if not args.yes and input("Start? [t/N]: ").strip().lower() != "t":
        return 1

    from pinecone_bot.base import make_base
    from pinecone_bot.camera import RealSenseCamera
    from pinecone_bot.heading import PhyphoxGyro

    camera = gyro = base = None
    try:
        camera = RealSenseCamera(cfg, depth=True)
        gyro = PhyphoxGyro(cfg.heading).start()
        base = make_base(cfg)
        cal = Calibrator(cfg, base, gyro, read_depth=lambda: camera.read()[1], ask=ask_left)
        res = cal.run(args.speeds, args.seconds, args.turn_rates, args.turn_seconds)
    except (RuntimeError, KeyboardInterrupt) as e:
        print("Przerwane: %s" % (e or "Ctrl+C"))
        return 2
    finally:
        if base is not None:
            base.stop()
            base.close()
        if gyro is not None:
            gyro.close()
        if camera is not None:
            camera.close()

    changes = apply(cfg, res)
    print("\nWynik:")
    for line in changes or ["bez zmian wzgledem configu"]:
        print("  " + line)
    if res["xiao_pwm"] is None:
        print("  PWM jazdy nie dopasowane (potrzeba 2 przejazdow, w ktorych robot ruszyl)")
    if res["xiao_steer"] is None:
        print("  PWM obrotu nie dopasowane (potrzeba 2 obrotow > %.0f st)" % MIN_TURN_DEG)
    if args.write and changes:
        print("Zapisane: %s" % cfg.save(args.config))
    elif changes:
        print("Nic nie zapisane (dodaj --write).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
