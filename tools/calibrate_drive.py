"""
Kalibracja jazdy do przodu (Xiao) glebia z kamery: ile m/s daje dane PWM, bez miarki.

Robot stoi przodem do sciany (albo duzego plaskiego przedmiotu) 1.5-3 m od niej, kamera patrzy prosto.
Dla kazdego PWM z listy: odleglosc do sciany (mediana glebi ze srodka kadru z kilku klatek), jazda
--seconds do przodu, znowu odleglosc -> przejechane metry -> m/s. Potem tyle samo do tylu, zeby wrocic
na start. W czasie jazdy do przodu glebia jest sprawdzana co klatke: sciana blizej niz --min-wall -> stop.

Z wynikow (prosta v = k * (PWM - PWM0) przez punkty, w ktorych robot jechal):
  base.xiao_pwm_min = PWM0 (ponizej robot stoi), base.xiao_pwm_max = PWM dla control.v_max.

Panel jazdy (web_control.py) musi byc zamkniety - trzyma port Xiao. STOP: tools/estop_server.py.

  python tools/calibrate_drive.py                      # PWM 80, 100, 130
  python tools/calibrate_drive.py --pwm 90,110,130 --write
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pinecone_bot.config import Config  # noqa: E402

SEND_PERIOD = 0.05
MOVE_SPEED = 0.02   # m/s; wolniej = robot stoi


def wall_distance(depth: np.ndarray | None) -> float | None:
    """Mediana glebi w srodkowym prostokacie kadru (40% szerokosci, 30% wysokosci). None bez pomiaru."""
    if depth is None:
        return None
    h, w = depth.shape[:2]
    patch = depth[int(0.35 * h):int(0.65 * h), int(0.3 * w):int(0.7 * w)]
    valid = patch[(patch > 0.2) & (patch < 8.0)]
    if valid.size < 0.3 * patch.size:
        return None
    return float(np.median(valid))


def measure(read_depth, n: int = 8) -> float | None:
    vals = [d for d in (wall_distance(read_depth()) for _ in range(n)) if d is not None]
    if len(vals) < n // 2:
        return None
    return float(np.median(vals))


@dataclass
class Run:
    pwm: int
    meters: float
    speed: float    # m/s


def drive_run(send, read_depth, sleep, clock, pwm: int, seconds: float, min_wall: float, log=print) -> Run | None:
    d0 = measure(read_depth)
    if d0 is None:
        log(f"  PWM {pwm}: brak glebi na srodku kadru - kamera patrzy na sciane?")
        return None
    if d0 < min_wall + 0.5:
        log(f"  PWM {pwm}: sciana {d0:.2f} m - za blisko, odsun robota")
        return None
    t0 = clock()
    driven_s = 0.0
    stopped_early = False
    try:
        while clock() - t0 < seconds:
            send(pwm)
            d = wall_distance(read_depth())
            if d is not None and d < min_wall:
                stopped_early = True
                break
            sleep(SEND_PERIOD)
            driven_s = clock() - t0
    finally:
        for _ in range(5):
            send(0)
            sleep(0.02)
    sleep(1.0)   # wybieg i drgania
    d1 = measure(read_depth)
    if d1 is None:
        log(f"  PWM {pwm}: brak glebi po jezdzie")
        return None
    meters = d0 - d1
    run = Run(pwm, meters, meters / max(driven_s, 0.1))
    log(f"  PWM {pwm}: {d0:.2f} -> {d1:.2f} m, przejechal {meters * 100:5.1f} cm w {driven_s:.1f} s = "
        f"{run.speed:.3f} m/s" + ("  (STOP: sciana za blisko)" if stopped_early else ""))
    # powrot na start: tyle samo czasu do tylu, ile jechal do przodu
    t0 = clock()
    try:
        while clock() - t0 < driven_s:
            send(-pwm)
            sleep(SEND_PERIOD)
    finally:
        for _ in range(5):
            send(0)
            sleep(0.02)
    sleep(1.0)
    return run


@dataclass
class DriveFit:
    pwm_min: int
    pwm_max: int
    k: float        # m/s na jednostke PWM


def fit_drive(runs: list[Run], v_max: float) -> DriveFit | None:
    """Prosta v = k * (pwm - pwm0) przez przebiegi, w ktorych robot jechal. None: jechal w mniej niz 2."""
    moving = [r for r in runs if r.speed >= MOVE_SPEED]
    if len(moving) < 2:
        return None
    x = np.array([r.pwm for r in moving], dtype=float)
    y = np.array([r.speed for r in moving], dtype=float)
    k, c = np.polyfit(x, y, 1)
    if k <= 0:
        return None
    pwm0 = -c / k
    still = [r.pwm for r in runs if r.speed < MOVE_SPEED]
    if still:
        pwm0 = max(pwm0, max(still))   # tam, gdzie stal, na pewno nie rusza
    pwm_max = pwm0 + v_max / k
    return DriveFit(int(round(pwm0)), int(round(pwm_max)), float(k))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None)
    p.add_argument("--port", default=None, help="port Xiao (domyslnie cfg.base.port)")
    p.add_argument("--pwm", default="80,100,130", help="lista PWM do przodu, po przecinku")
    p.add_argument("--seconds", type=float, default=2.0)
    p.add_argument("--min-wall", type=float, default=0.5, help="blizej sciany niz tyle [m] = stop")
    p.add_argument("--write", action="store_true", help="zapisz xiao_pwm_min/max do pinecone_config.json")
    args = p.parse_args()

    import serial

    from pinecone_bot.camera import make_camera

    cfg = Config.load(args.config)
    port = args.port or cfg.base.port
    cam = make_camera(cfg, None, depth=True)
    ser = serial.Serial(port, cfg.base.baud, timeout=0, write_timeout=0.5)
    time.sleep(0.3)   # Xiao resetuje sie przy otwarciu portu

    def send(a: int) -> None:
        ser.write(f"a{int(a)} b0\n".encode("ascii"))

    def read_depth():
        return cam.read()[1]

    bgr, depth = cam.read()
    try:
        import cv2
        os.makedirs("frames", exist_ok=True)
        cv2.imwrite("frames/calibrate_drive.jpg", bgr)
    except Exception as exc:  # noqa: BLE001 - podglad nie jest konieczny
        print(f"(bez zdjecia widoku: {exc})")
    d = wall_distance(depth)
    print(f"widok kamery: frames/calibrate_drive.jpg, srodek kadru {'-' if d is None else f'{d:.2f} m'}")

    print(f"Xiao {port}. Robot pojedzie do przodu i wroci tylem. Ctrl+C / STOP przerywa.")
    runs = []
    try:
        for pwm in (int(x) for x in args.pwm.split(",")):
            r = drive_run(send, read_depth, time.sleep, time.monotonic, pwm, args.seconds, args.min_wall)
            if r is not None:
                runs.append(r)
    except KeyboardInterrupt:
        print("przerwane")
        return 1
    finally:
        for _ in range(5):
            send(0)
            time.sleep(0.02)
        ser.close()
        cam.close()

    if runs and all(abs(r.meters) < 0.03 for r in runs):
        print("UWAGA: glebia na srodku kadru prawie sie nie zmienila, choc robot jechal - kamera nie patrzy przed "
              "robota (sufit, podloga albo kamera na ramieniu, ktore sie ruszylo). Zobacz frames/calibrate_drive.jpg.")
        return 2
    fit = fit_drive(runs, cfg.control.v_max)
    if fit is None:
        print("Za malo przejazdow z ruchem (potrzebne 2). Podnies --pwm albo sprawdz, czy hover jest wlaczony.")
        return 2
    v_lane = cfg.control.search_drive_v
    print(f"\nrusza od PWM {fit.pwm_min}, v_max {cfg.control.v_max} m/s przy PWM {fit.pwm_max}, "
          f"{fit.k * 100:.2f} cm/s na PWM; pasy ({v_lane} m/s) pojada na PWM "
          f"{int(round(fit.pwm_min + v_lane / fit.k))}")
    if args.write:
        cfg.base.xiao_pwm_min = fit.pwm_min
        cfg.base.xiao_pwm_max = fit.pwm_max
        print(f"zapisane: {cfg.save(args.config)}")
    else:
        print("bez --write nic nie zapisane")
    return 0


if __name__ == "__main__":
    sys.exit(main())
