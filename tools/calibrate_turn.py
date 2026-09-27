"""
Kalibracja obrotu bazy (Xiao) zyroskopem telefonu: od jakiego PWM skretu hover rusza i jaki PWM daje w_max.

Hover ma martwa strefe (ponizej ~140 stoi), a zaraz nad nia przyspiesza bardzo stromo. Bez tych liczb
regulator kursu w pasach wysyla za slabe komendy i obrot nigdy nie konczy sie na kacie docelowym.

Narzedzie kreci robotem w miejscu w lewo coraz mocniej: steer = -start, -start-krok, ... (ujemne b = w lewo
wg XiaoBase), kazdy krok step_s sekund, potem stop. Predkosc obrotu mierzy telefon (phyphox) z pominieciem
rozpedzania. Konczy, gdy predkosc przekroczy --max-rate albo skonczy sie zakres. Z wynikow:
  xiao_steer_min = najmniejsze |b|, przy ktorym robot sie kreci
  xiao_steer_max = |b| dajace control.w_max (interpolacja miedzy krokami)
  steer_sign     = -1, jesli ujemne b krecilo w prawo (kurs malal)

Telefon plasko na robocie, ekranem do gory, phyphox "Zyroskop" z remote access na wierzchu.
Robot na podlodze, wolne 1 m dookola, STOP pod reka (tools/estop_server.py ubija tez ten skrypt).
Panel jazdy (web_control.py) musi byc zamkniety - trzyma port Xiao.

  python tools/calibrate_turn.py                  # tylko pomiar i tabela
  python tools/calibrate_turn.py --write          # pomiar + zapis do pinecone_config.json
  python tools/calibrate_turn.py --response 160 --step-s 2   # opoznienie i rozpedzanie (3 proby)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pinecone_bot.config import Config  # noqa: E402

MOVE_RATE = 0.05     # rad/s; ponizej tego uznajemy, ze robot stoi (szum zyroskopu, drgania)
SEND_PERIOD = 0.05   # s; Xiao ma watchdog 500 ms


@dataclass
class Step:
    pwm: int        # |b| wyslane do Xiao (skret w lewo = -pwm)
    rate: float     # rad/s zmierzone zyroskopem, + = w lewo


@dataclass
class TurnFit:
    steer_min: int
    steer_max: int
    steer_sign: float
    reached_w_max: bool


def fit_turn(steps: list[Step], w_max: float) -> TurnFit | None:
    """None, gdy robot w ogole sie nie ruszyl."""
    moving = [s for s in steps if abs(s.rate) >= MOVE_RATE]
    if not moving:
        return None
    sign = 1.0 if sum(s.rate for s in moving) > 0 else -1.0
    steer_min = moving[0].pwm
    prev = None
    for s in steps:
        r = abs(s.rate)
        if r >= w_max:
            if prev is None or abs(prev.rate) >= w_max:
                return TurnFit(steer_min, s.pwm, sign, True)
            r0 = abs(prev.rate)
            frac = (w_max - r0) / (r - r0)
            return TurnFit(steer_min, int(round(prev.pwm + frac * (s.pwm - prev.pwm))), sign, True)
        prev = s
    return TurnFit(steer_min, steps[-1].pwm, sign, False)


def sweep(send, yaw, sleep, pwms: list[int], step_s: float, spinup_s: float, rest_s: float,
          max_rate: float, log=print) -> list[Step]:
    """
    send(b) wysyla komende skretu (b ze znakiem) i musi byc wolane co SEND_PERIOD; yaw() -> rad albo None;
    sleep(dt) czeka. Po kazdym kroku stop i rest_s przerwy.
    """
    steps: list[Step] = []

    def hold(b: int, seconds: float) -> None:
        t = 0.0
        while t < seconds:
            send(b)
            sleep(SEND_PERIOD)
            t += SEND_PERIOD

    try:
        for pwm in pwms:
            hold(-pwm, spinup_s)
            y0 = yaw()
            hold(-pwm, step_s - spinup_s)
            y1 = yaw()
            hold(0, rest_s)
            if y0 is None or y1 is None:
                raise RuntimeError("brak kursu z telefonu (phyphox na wierzchu? ekran wlaczony?)")
            rate = (y1 - y0) / (step_s - spinup_s)
            steps.append(Step(pwm, rate))
            log(f"  b=-{pwm:3d}: {math.degrees(rate):+7.1f} st/s  ({rate:+.2f} rad/s)")
            if abs(rate) > max_rate:
                break
    finally:
        for _ in range(5):
            send(0)
            sleep(0.02)
    return steps


def record_response(send, yaw, clock, sleep, pwm: int, hold_s: float, coast_s: float = 1.0):
    """Staly skret -pwm przez hold_s, potem stop i coast_s wybiegu. Zwraca [(t, yaw)] od chwili komendy."""
    samples = []
    t0 = clock()
    try:
        while clock() - t0 < hold_s + coast_s:
            send(-pwm if clock() - t0 < hold_s else 0)
            y = yaw()
            if y is not None:
                samples.append((clock() - t0, y))
            sleep(SEND_PERIOD)
    finally:
        for _ in range(5):
            send(0)
            sleep(0.02)
    return samples


@dataclass
class Response:
    delay_s: float | None      # od komendy do pierwszej zmiany kursu > 1 st
    rise_s: float | None       # od poczatku ruchu do 63% predkosci koncowej
    rate: float                # rad/s pod koniec skretu
    coast_deg: float           # tyle obrocil sie po stopie


def analyze_response(samples, hold_s: float) -> Response:
    y0 = samples[0][1]
    moved = [t for t, y in samples if abs(y - y0) > math.radians(1.0)]
    delay = moved[0] if moved else None
    held = [(t, y) for t, y in samples if hold_s - 0.5 <= t <= hold_s]
    rate = (held[-1][1] - held[0][1]) / (held[-1][0] - held[0][0]) if len(held) > 1 else 0.0
    rise = None
    if delay is not None and abs(rate) > 0.05:
        for (ta, ya), (tb, yb) in zip(samples, samples[3:]):
            if ta >= delay and tb <= hold_s and abs((yb - ya) / (tb - ta)) >= 0.63 * abs(rate):
                rise = tb - delay
                break
    at_stop = min(samples, key=lambda s: abs(s[0] - hold_s))[1]
    return Response(delay, rise, rate, math.degrees(samples[-1][1] - at_stop))


def apply_fit(cfg: Config, fit: TurnFit) -> None:
    cfg.base.xiao_steer_min = fit.steer_min
    cfg.base.xiao_steer_max = max(fit.steer_max, fit.steer_min + 1)
    cfg.control.steer_sign = fit.steer_sign


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=None)
    p.add_argument("--port", default=None, help="port Xiao (domyslnie cfg.base.port)")
    p.add_argument("--start", type=int, default=100)
    p.add_argument("--stop", type=int, default=260)
    p.add_argument("--step", type=int, default=10)
    p.add_argument("--step-s", type=float, default=1.5, help="czas jednego kroku [s]")
    p.add_argument("--max-rate", type=float, default=1.2, help="rad/s; szybciej = koniec pomiaru")
    p.add_argument("--write", action="store_true", help="zapisz wynik do pinecone_config.json")
    p.add_argument("--response", type=int, default=None, metavar="PWM",
                   help="zamiast skanu: 3x staly skret -PWM przez --step-s, mierzy opoznienie i rozpedzanie")
    args = p.parse_args()

    import serial

    from pinecone_bot.heading import PhyphoxGyro

    cfg = Config.load(args.config)
    port = args.port or cfg.base.port
    gyro = PhyphoxGyro(cfg.heading).start()
    t0 = time.monotonic()
    while gyro.yaw() is None and time.monotonic() - t0 < 5.0:
        time.sleep(0.1)
    if gyro.yaw() is None:
        print(f"Brak danych z telefonu ({cfg.heading.phyphox_url}): {gyro.last_error}")
        gyro.close()
        return 2
    print(f"telefon OK, Xiao {port}. Robot zacznie krecic sie w lewo. Ctrl+C / STOP przerywa.")
    ser = serial.Serial(port, cfg.base.baud, timeout=0, write_timeout=0.5)
    time.sleep(0.3)   # Xiao resetuje sie przy otwarciu portu

    def send(b: int) -> None:
        ser.write(f"a0 b{int(b)}\n".encode("ascii"))

    if args.response is not None:
        try:
            for i in range(3):
                samples = record_response(send, gyro.yaw, time.monotonic, time.sleep, args.response, args.step_s)
                r = analyze_response(samples, args.step_s)
                fmt = lambda x: "  -  " if x is None else f"{x:4.2f}"  # noqa: E731
                print(f"  proba {i + 1}: opoznienie {fmt(r.delay_s)} s, rozpedzanie {fmt(r.rise_s)} s, "
                      f"predkosc {r.rate:+.2f} rad/s, wybieg po stopie {r.coast_deg:+.1f} st")
                time.sleep(2.0)
        except KeyboardInterrupt:
            print("przerwane")
            return 1
        finally:
            ser.close()
            gyro.close()
        return 0

    try:
        steps = sweep(send, gyro.yaw, time.sleep, list(range(args.start, args.stop + 1, args.step)),
                      step_s=args.step_s, spinup_s=0.5, rest_s=1.0, max_rate=args.max_rate)
    except KeyboardInterrupt:
        print("przerwane")
        return 1
    finally:
        ser.close()
        gyro.close()

    fit = fit_turn(steps, cfg.control.w_max)
    if fit is None:
        print("Robot sie nie ruszyl w calym zakresie. Hover wlaczony? Podnies --stop.")
        return 2
    print(f"\nrusza od b={fit.steer_min}, w_max={cfg.control.w_max} rad/s przy b={fit.steer_max}"
          + ("" if fit.reached_w_max else " (w_max NIE osiagniete - wziete ostatnie b)")
          + f", steer_sign={fit.steer_sign:+.0f}")
    if fit.steer_sign < 0:
        print("UWAGA: ujemne b krecilo w prawo - mapowanie skretu odwrocone, steer_sign = -1")
    if args.write:
        apply_fit(cfg, fit)
        print(f"zapisane: {cfg.save(args.config)}")
    else:
        print("bez --write nic nie zapisane")
    return 0


if __name__ == "__main__":
    sys.exit(main())
