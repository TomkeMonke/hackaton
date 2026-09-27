"""
Petla predkosci obrotu na zyroskopie: brain zadaje w [rad/s], PWM skretu dobiera zyroskop.

Po co: hover przy tym samym PWM kreci sie raz, raz nie. Zmierzone 2026-09-27 (tools/calibrate_turn.py):
robot, ktory stal, ruszal dopiero od |b| ~160 (tarcie statyczne), a juz krecacy sie od ~100; 10 jednostek
wyzej to +0.3..0.7 rad/s. Stala tabela w -> PWM (xiao_steer_min/max) tego nie opisze, a regulator kursu
w pasach wysylal wtedy za slabe komendy i obrot nie dochodzil do celu.

TurnRateLoop (czysta logika, opis stanow w klasie): robot stoi -> |b| rosnie rampa, az ruszy; ruszyl ->
od razu PWM, ktory ostatnio trzymal predkosc; kreci sie -> calka bledu predkosci. Zmierzone w = roznica kursu
z ostatnich rate_window_s. Nigdy wiecej niz rate_pwm_max.

GyroTurnBase: owija baze Xiao (set_speed -> set_raw), tyka petle co 1/rate_hz. Bez kursu (telefon zgasl)
wraca do zwyklego set_speed z mapowaniem z configu. Jazda do przodu zostaje z mapowania v -> PWM.
"""
from __future__ import annotations

import threading
import time
from collections import deque

from .config import Config, HeadingConfig

W_EPS = 1e-3


class TurnRateLoop:
    """
    Trzy stany obrotu w miejscu:
      STOI    - zadany obrot, a zyroskop nie widzi ruchu: |b| rosnie szybka rampa (rate_ramp PWM/s) od progu,
                przy ktorym robot ostatnio ruszyl (u_break - rate_break_margin), a nie od zera;
      RUSZYL  - pierwszy odczyt ruchu: zapamietaj u_break i od razu zejdz do u_run (PWM, ktory ostatnio trzymal
                zadana predkosc), bo po zerwaniu tarcia statycznego ten sam PWM kreci 3-5x za szybko;
      KRECI   - calka bledu predkosci (rate_ki), nie nizej niz rate_pwm_start - 10 (inaczej staje i wraca stick-slip).
    Przy jezdzie do przodu kola sie tocza i nie ma martwej strefy: zwykla calka od zera, bez rampy.
    """

    def __init__(self, cfg: HeadingConfig):
        self.cfg = cfg
        self.u = 0.0            # |b| teraz
        self.dir = 0            # +1 w lewo, -1 w prawo, 0 stoj
        self.moving = False
        self.rolling = False
        self.u_break = float(cfg.rate_pwm_start)   # przy tylu ostatnio ruszyl z miejsca
        self.u_run: float | None = None             # tyle ostatnio trzymalo zadana predkosc
        self._still_since: float | None = None
        self._hist: deque = deque()
        self._last_now: float | None = None
        self.rate: float | None = None   # ostatnia zmierzona predkosc [rad/s], + = w lewo

    def reset(self) -> None:
        self.u = 0.0
        self.dir = 0
        self.moving = False
        self._still_since = None

    def _measure(self, now: float, yaw: float | None) -> float | None:
        if yaw is not None:
            self._hist.append((now, yaw))
        while len(self._hist) > 2 and now - self._hist[0][0] > self.cfg.rate_window_s:
            self._hist.popleft()
        if len(self._hist) < 2:
            return None
        (t0, y0), (t1, y1) = self._hist[0], self._hist[-1]
        if t1 - t0 < 0.5 * self.cfg.rate_window_s:
            return None
        return (y1 - y0) / (t1 - t0)

    def update(self, target_w: float, yaw: float | None, now: float, rolling: bool = False) -> int:
        """Zwraca b dla Xiao (ujemne = w lewo, jak XiaoBase: steer = -w). rolling: baza jedzie do przodu."""
        c = self.cfg
        dt = 0.0 if self._last_now is None else min(max(now - self._last_now, 0.0), 0.2)
        self._last_now = now
        self.rate = self._measure(now, yaw)
        if abs(target_w) < W_EPS:
            self.reset()
            return 0
        d = 1 if target_w > 0 else -1
        target = abs(target_w)
        along = None if self.rate is None else d * self.rate   # predkosc w zadanym kierunku
        if d != self.dir or rolling != self.rolling:
            self.dir = d
            self.rolling = rolling
            self.moving = False
            self._still_since = None
            self.u = 0.0 if rolling else max(float(c.rate_pwm_start), self.u_break - c.rate_break_margin)

        if rolling:
            if along is not None:
                self.u += c.rate_ki * (target - along) * dt
            self.u = min(max(self.u, 0.0), float(c.rate_pwm_max))
            return int(round(-d * self.u))

        if along is not None:
            if not self.moving:
                if along >= c.rate_move:
                    self.moving = True
                    # ruch widac z opoznieniem (telefon + okno pomiaru): rampa zdazyla urosnac o ramp * lag
                    self.u_break = max(float(c.rate_pwm_start), self.u - c.rate_ramp * c.rate_lag_s)
                    self.u = self.u_run if self.u_run is not None else float(c.rate_pwm_start) + 10.0
                else:
                    self.u += c.rate_ramp * dt
            else:
                if along < c.rate_move:
                    self._still_since = now if self._still_since is None else self._still_since
                    if now - self._still_since > 0.3:
                        self.moving = False          # stanal: znowu rampa
                        self._still_since = None
                else:
                    self._still_since = None
                self.u += c.rate_ki * (target - along) * dt
                if along > 2.0 * target + 0.3:
                    # pedzi (np. kopniak po zerwaniu tarcia): od razu na dol, calka jest za wolna przy opoznieniu
                    self.u = min(self.u, self.u_run if self.u_run is not None else float(c.rate_pwm_start))
                self.u = max(self.u, float(c.rate_pwm_start) - 10.0)
                if abs(target - along) < 0.25 * target + 0.05:
                    self.u_run = self.u if self.u_run is None else 0.8 * self.u_run + 0.2 * self.u
        self.u = min(max(self.u, 0.0), float(c.rate_pwm_max))
        return int(round(-d * self.u))


class GyroTurnBase:
    """Baza z petla obrotu na zyroskopie. inner musi miec set_speed, set_raw, pwm_for, stop, close."""

    def __init__(self, inner, heading, cfg: Config, clock=time.monotonic, thread: bool = True):
        self.inner = inner
        self.heading = heading
        self.cfg = cfg
        self.clock = clock
        self.loop = TurnRateLoop(cfg.heading)
        self.v = 0.0
        self.w = 0.0
        self.closed_loop = False     # czy ostatni tick szedl po zyroskopie
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = None
        if thread:
            self._thread = threading.Thread(target=self._run, name="turn-loop", daemon=True)
            self._thread.start()

    # -- interfejs Base ----------------------------------------------------
    def set_speed(self, v_mps: float, w_radps: float) -> None:
        with self._lock:
            self.v, self.w = v_mps, w_radps
        self.tick()

    def stop(self) -> None:
        with self._lock:
            self.v = self.w = 0.0
            self.loop.reset()
            self.inner.stop()

    def odometry(self):
        return self.inner.odometry()

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.inner.close()

    # -- petla -------------------------------------------------------------
    def tick(self, now: float | None = None) -> None:
        with self._lock:
            now = self.clock() if now is None else now
            yaw = self.heading.yaw()
            if yaw is None:
                self.closed_loop = False
                self.loop.reset()
                self.inner.set_speed(self.v, self.w)
                return
            self.closed_loop = True
            rolling = abs(self.v) > 0.02
            b = self.loop.update(self.w, yaw, now, rolling=rolling)
            speed_pwm = self.inner.pwm_for(self.v, 0.0)[0]
            self.inner.set_raw(speed_pwm, b)

    def _run(self) -> None:
        period = 1.0 / max(self.cfg.heading.rate_hz, 1.0)
        while not self._stop_evt.wait(period):
            self.tick()

    def advance(self, dt: float) -> None:
        """Symulacja (bez watku): tyka petle co 1/rate_hz i przesuwa model napedu."""
        period = 1.0 / max(self.cfg.heading.rate_hz, 1.0)
        left = float(dt)
        while left > 1e-9:
            step = min(period, left)
            self.tick()
            self.inner.advance(step)
            left -= step


def wrap_with_turn_loop(base, heading, cfg: Config, **kw):
    """GyroTurnBase, gdy jest kurs, petla wlaczona i baza umie set_raw (Xiao); inaczej baza bez zmian."""
    if heading is None or not cfg.heading.rate_loop or not hasattr(base, "set_raw"):
        return base
    return GyroTurnBase(base, heading, cfg, **kw)

