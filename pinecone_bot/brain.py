"""
Maszyna stanow zbieracza szyszek + regulator P podjazdu.

SEARCH   -> obracaj sie w miejscu; jest detekcja -> APPROACH; dlugo nic -> DONE
APPROACH -> regulator P: obroc tak, by szyszka byla w kolumnie cx, jedz tak, by byla w wierszu target_row;
            settle_frames klatek w tolerancji -> ALIGN; zgubiona > lost_timeout -> SEARCH
ALIGN    -> stop, potwierdz na stojaco (2 klatki) -> GRASP; nie potwierdzone -> APPROACH
GRASP    -> arm.replay(najblizszy nagrany chwyt); sukces -> DROP; pusty chwytak -> RETRY
DROP     -> arm.replay('drop_box'), arm.home() -> SEARCH
RETRY    -> cofnij chwile -> APPROACH; po `retries` porazkach odwroc sie i SEARCH

Cala 'inteligencja' to ten plik. Zadnych modeli, zadnych promptow. Kazda decyzja jest w logu CSV.
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from enum import Enum

from .config import Config
from .detector import Detection


def _wrap(a: float) -> float:
    """Kat do przedzialu (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class State(str, Enum):
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    GRASP = "GRASP"
    DROP = "DROP"
    RETRY = "RETRY"
    DONE = "DONE"


class WallClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, dt: float) -> None:
        if dt > 0:
            time.sleep(dt)


@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    err_x: float = 0.0
    err_y: float = 0.0
    in_tol: bool = False
    target_row: float = 0.0


@dataclass
class Stats:
    collected: int = 0
    grasp_attempts: int = 0
    grasp_failures: int = 0
    frames: int = 0
    transitions: list = field(default_factory=list)


class Controller:
    """Regulator P na pozycji szyszki w obrazie. Czysta funkcja: (px, py) -> (v, w)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg

    def target_row_for(self, py: float) -> float:
        """Celuj w srodkowy chwyt; gdy szyszka jest juz blisko innego nagranego wiersza, celuj w niego."""
        c = self.cfg.control
        g = self.cfg.grasp_for_row(py)
        if abs(g.target_row - py) <= 2 * c.tol_y_px:
            return g.target_row
        mid = self.cfg.grasps[len(self.cfg.grasps) // 2]
        return mid.target_row

    def compute(self, det: Detection) -> Command:
        c = self.cfg.control
        target_row = self.target_row_for(det.py)
        err_x = det.px - self.cfg.cx          # >0: szyszka na prawo od srodka chwytaka
        err_y = target_row - det.py           # >0: szyszka wyzej w obrazie = dalej -> jedz do przodu

        w = 0.0 if abs(err_x) < c.deadband_px else -c.kx * err_x * c.steer_sign
        v = 0.0 if abs(err_y) < c.deadband_px else c.ky * err_y
        # najpierw obrot, potem jazda: przy duzym bledzie kata nie jedz do przodu
        v *= max(0.0, 1.0 - abs(err_x) / 80.0)
        v = min(max(v, c.v_min), c.v_max)
        w = min(max(w, -c.w_max), c.w_max)

        in_tol = abs(err_x) <= c.tol_x_px and any(
            abs(g.target_row - det.py) <= c.tol_y_px for g in self.cfg.grasps
        )
        return Command(v, w, err_x, err_y, in_tol, target_row)


class Brain:
    def __init__(self, cfg: Config, camera, detector, base, arm, clock=None,
                 log_path: str | None = None, on_frame=None, verbose: bool = True, heading=None):
        self.cfg = cfg
        self.heading = heading  # zrodlo kursu (pinecone_bot/heading.py) albo None = pasy z czasu
        self.camera = camera
        self.detector = detector
        self.base = base
        self.arm = arm
        self.clock = clock or WallClock()
        # chwyty bez pliku ruchu wypadaja z listy, inaczej replay() konczy sie FileNotFoundError w polowie misji
        has_motion = getattr(arm, "has_motion", None)
        if callable(has_motion):
            missing = [g.name for g in cfg.grasps if not has_motion(g.name)]
            if missing:
                print(f"UWAGA: brak plikow ruchu dla chwytow {missing}, pomijam je "
                      f"(nagraj tools/record_waypoints.py --name <nazwa>)")
                cfg.grasps = [g for g in cfg.grasps if g.name not in missing]
            if not cfg.grasps:
                raise RuntimeError("zaden skonfigurowany chwyt nie ma pliku ruchu w motions/")
        self.controller = Controller(cfg)
        self.on_frame = on_frame
        self.verbose = verbose
        self.stats = Stats()

        self.state = State.SEARCH
        self._state_since = self.clock.now()
        self._last_seen = self.clock.now()
        self._settle = 0
        self._align_frames = 0
        self._retry_count = 0
        self._explore_t = 0.0
        self._explore_last_t: float | None = None
        # pasy po kursie: lista odcinkow i postep w biezacym (patrz _explore_heading_cmd)
        self._segments = self._lane_segments()
        self._seg_idx = 0
        self._seg_progress = 0.0    # line: metry, spin: radiany
        self._seg_time = 0.0        # sekundy w SEARCH na tym odcinku (bezpiecznik, gdy kurs nie zbiega)
        self._seg_last_yaw: float | None = None
        self._explore_last_v = 0.0
        self._h0: float | None = None   # kurs na starcie wzorca = kierunek pierwszego pasa
        self._yaw_lost_since: float | None = None
        self._last_yaw: float | None = None
        self._last_det: Detection | None = None
        self._last_cmd = Command()

        self._log_fh = None
        self._log = None
        if log_path:
            self._log_fh = open(log_path, "w", newline="", encoding="ascii")
            self._log = csv.writer(self._log_fh)
            self._log.writerow(["t", "state", "n_det", "px", "py", "err_x", "err_y", "v", "w", "collected",
                                "yaw_deg"])

    # --- pomocnicze ------------------------------------------------------
    def _goto(self, new: State, why: str = "") -> None:
        if new != self.state:
            t = self.clock.now()
            self.stats.transitions.append((round(t, 2), self.state.value, new.value, why))
            if self.verbose:
                print(f"[{t:7.2f}s] {self.state.value:8s} -> {new.value:8s} {why}")
            self.state = new
            self._state_since = t
            if new == State.SEARCH:
                self._explore_last_t = None
                self._explore_last_v = 0.0
                self._seg_last_yaw = None   # obrot w APPROACH nie liczy sie do pelnego obrotu
            self._settle = 0
            self._align_frames = 0

    def _elapsed(self) -> float:
        return self.clock.now() - self._state_since

    def _drive(self, v: float, w: float) -> None:
        self.base.set_speed(v, w)
        self._last_cmd.v, self._last_cmd.w = v, w

    def _timed_drive(self, v: float, w: float, seconds: float) -> None:
        """Ruch otwarty przez zadany czas (cofanie, odwrot). Kamera w tym czasie nie steruje."""
        self._drive(v, w)
        self.clock.sleep(seconds)
        self.base.stop()

    def _explore_done(self, now: float) -> bool:
        """lanes: po lane_count pasach. spin_drive: po search_timeout_s bez detekcji."""
        c = self.cfg.control
        if c.search_pattern != "lanes":
            return self._elapsed() > c.search_timeout_s
        if self.heading is not None:
            return self._seg_idx >= len(self._segments)
        w_s = max(c.search_w, 1e-3)
        v_s = max(c.search_drive_v, 1e-3)
        spin_s = 2 * math.pi / w_s
        period = c.lane_length_m / v_s + 2 * (math.pi / 2) / w_s + c.lane_spacing_m / v_s
        return self._explore_t > spin_s + c.lane_count * period

    def _explore_cmd(self, now: float) -> tuple[float, float]:
        """
        Wzorzec szukania, gdy nic nie widac. Czas wzorca (_explore_t) biegnie tylko w SEARCH i nie zeruje sie
        po chwycie, wiec robot wraca do pasow tam, gdzie przerwal (z dokladnoscia do tego, ze stoi w innym
        miejscu po podjezdzie do szyszki; bez odometrii lepiej sie nie da).

        lanes:      pelny obrot na start (kamera widzi 1.5 m, wiec od razu lapie szyszki wokol), potem pasy:
                    prosto lane_length_m, obrot 90 st, prosto lane_spacing_m, obrot 90 st w te sama strone,
                    prosto lane_length_m z powrotem, i kolejny pas w druga strone. Czasy z predkosci zadanych,
                    wiec na trawie wymaga zmierzenia search_drive_v i search_w (tools/base_test.py);
                    z bipropellantem (zamknieta petla predkosci) jest dokladniej niz z Xiao (otwarte PWM).
        spin_drive: pelny obrot, kawalek prosto, od nowa. Prostsze, ale bladzi losowo.
        """
        c = self.cfg.control
        if self.heading is not None and c.search_pattern == "lanes":
            yaw = self.heading.yaw()
            self._last_yaw = yaw
            if yaw is not None:
                self._yaw_lost_since = None
                return self._explore_heading_cmd(now, yaw)
            if self._yaw_lost_since is None:
                self._yaw_lost_since = now
            if now - self._yaw_lost_since <= self.cfg.heading.lost_s:
                # stoj i czekaj na kurs; ten czas nie liczy sie do pasa
                self._explore_last_t = now
                self._explore_last_v = 0.0
                return 0.0, 0.0
            self._heading_fallback(f"brak kursu od {self.cfg.heading.lost_s:.1f} s")
        if self._explore_last_t is not None:
            self._explore_t += max(0.0, now - self._explore_last_t)
        self._explore_last_t = now
        t = self._explore_t
        w_s = max(c.search_w, 1e-3)
        v_s = max(c.search_drive_v, 1e-3)
        spin_s = 2 * math.pi / w_s
        if t < spin_s:
            return 0.0, c.search_w
        t -= spin_s
        if c.search_pattern != "lanes":
            drive_s = c.search_drive_m / v_s
            return (0.0, c.search_w) if (t % (spin_s + drive_s)) < spin_s else (c.search_drive_v, 0.0)
        lane_s = c.lane_length_m / v_s
        turn_s = (math.pi / 2) / w_s
        gap_s = c.lane_spacing_m / v_s
        period = lane_s + turn_s + gap_s + turn_s
        lane_idx = int(t // period)
        phase_t = t - lane_idx * period
        turn_dir = c.search_w if lane_idx % 2 == 0 else -c.search_w   # parzysty pas: skret w lewo, nieparzysty: w prawo
        if phase_t < lane_s:
            return c.search_drive_v, 0.0
        phase_t -= lane_s
        if phase_t < turn_s:
            return 0.0, turn_dir
        phase_t -= turn_s
        if phase_t < gap_s:
            return c.search_drive_v, 0.0
        return 0.0, turn_dir

    def _lane_segments(self) -> list[tuple[str, float, float]]:
        """
        Pasy jako odcinki (rodzaj, ile, kurs docelowy wzgledem kursu startowego):
        spin = pelny obrot (ile w radianach), line = prosto (ile w metrach) z trzymaniem kursu,
        turn = obrot do kursu. Ta sama geometria co pasy z czasu w _explore_cmd.
        """
        c = self.cfg.control
        segs = [("spin", 2 * math.pi, 0.0)]
        for i in range(c.lane_count):
            lane_h, end_h = (0.0, math.pi) if i % 2 == 0 else (math.pi, 0.0)
            segs += [("line", c.lane_length_m, lane_h), ("turn", 0.0, math.pi / 2),
                     ("line", c.lane_spacing_m, math.pi / 2), ("turn", 0.0, end_h)]
        return segs

    def _seg_nominal_s(self, kind: str, amount: float) -> float:
        c = self.cfg.control
        w_s = max(c.search_w, 1e-3)
        v_s = max(c.search_drive_v, 1e-3)
        if kind == "spin":
            return amount / w_s
        if kind == "turn":
            return (math.pi / 2) / w_s
        return amount / v_s

    def _next_segment(self) -> None:
        self._seg_idx += 1
        self._seg_progress = 0.0
        self._seg_time = 0.0
        self._seg_last_yaw = None
        self._explore_last_v = 0.0

    def _turn_w(self, err: float) -> float:
        """Obrot w miejscu do kursu: P, ograniczony do search_w, nie wolniej niz w_min."""
        c, h = self.cfg.control, self.cfg.heading
        w = min(max(h.kp * err, -c.search_w), c.search_w)
        if abs(w) < h.w_min:
            w = math.copysign(h.w_min, err)
        return w * c.steer_sign

    def _heading_fallback(self, why: str) -> None:
        """Kurs niedostepny albo nie zbiega: dalej pasy z czasu, od miejsca, w ktorym jestesmy we wzorcu."""
        t = sum(self._seg_nominal_s(k, a) for k, a, _ in self._segments[:self._seg_idx])
        if self._seg_idx < len(self._segments):
            kind, amount, _ = self._segments[self._seg_idx]
            if kind == "line":
                t += self._seg_progress / max(self.cfg.control.search_drive_v, 1e-3)
            elif kind == "spin":
                t += max(0.0, self._seg_progress) / max(self.cfg.control.search_w, 1e-3)
        print(f"UWAGA: pasy po kursie wylaczone ({why}), dalej pasy z czasu")
        self.stats.transitions.append((round(self.clock.now(), 2), "SEARCH", "SEARCH", f"bez kursu: {why}"))
        self.heading = None
        self._explore_t = t

    def _explore_heading_cmd(self, now: float, yaw: float) -> tuple[float, float]:
        """
        Pasy po kursie (zyroskop albo odometria). Obroty koncza sie, gdy kurs dojdzie do celu, a nie po czasie,
        wiec poslizg kol i rozne silniki nie skrecaja pasow. Na prostej regulator P trzyma kurs pasa.
        Dlugosc prostej dalej liczona z czasu i search_drive_v (zyroskop nie mierzy drogi).
        Po podjezdzie do szyszki robot najpierw obraca sie z powrotem na kurs pasa, potem jedzie dalej.
        """
        c, h = self.cfg.control, self.cfg.heading
        dt = 0.0 if self._explore_last_t is None else max(0.0, now - self._explore_last_t)
        self._explore_last_t = now
        if self._h0 is None:
            self._h0 = yaw
        # postep z poprzedniej komendy (predkosc do przodu dotyczy biezacego odcinka)
        self._seg_progress += dt * self._explore_last_v
        self._seg_time += dt
        self._explore_last_v = 0.0
        tol = math.radians(h.tol_deg)
        while self._seg_idx < len(self._segments):
            kind, amount, target = self._segments[self._seg_idx]
            if self._seg_time > 3 * self._seg_nominal_s(kind, amount) + 10.0:
                self._heading_fallback(f"odcinek {self._seg_idx} ({kind}) trwa za dlugo - sprawdz "
                                       f"heading.sign i control.steer_sign")
                return self._explore_cmd(now)
            if kind == "spin":
                if self._seg_last_yaw is not None:
                    self._seg_progress += _wrap(yaw - self._seg_last_yaw)
                self._seg_last_yaw = yaw
                if self._seg_progress >= amount - tol:
                    self._next_segment()
                    continue
                return 0.0, c.search_w * c.steer_sign
            err = _wrap(self._h0 + target - yaw)
            if kind == "turn":
                if abs(err) <= tol:
                    self._next_segment()
                    continue
                return 0.0, self._turn_w(err)
            if self._seg_progress >= amount:
                self._next_segment()
                continue
            if abs(err) > math.radians(h.align_deg):
                return 0.0, self._turn_w(err)
            w = min(max(h.kp * err, -c.search_w), c.search_w) * c.steer_sign
            self._explore_last_v = c.search_drive_v
            return c.search_drive_v, w
        return 0.0, 0.0

    def _pick_target(self, dets: list[Detection]) -> Detection | None:
        """
        Ktora szyszke gonic. W SEARCH: najblizsza (najnizej w obrazie), a przy remisie ta blizej srodka.
        W APPROACH/ALIGN: ta sama, ktora gonilismy (najblizsza poprzedniej pozycji w pikselach),
        inaczej dwie szyszki w tej samej odleglosci powoduja wieczne krecenie w lewo i w prawo.
        """
        if not dets:
            return None
        full = [d for d in dets if not d.partial]
        if full:
            dets = full
        c = self.cfg.control
        if self.state in (State.APPROACH, State.ALIGN) and self._last_det is not None:
            last = self._last_det
            best = min(dets, key=lambda d: math.hypot(d.px - last.px, d.py - last.py))
            if math.hypot(best.px - last.px, best.py - last.py) <= c.track_max_jump_px:
                return best
        lowest = max(d.py for d in dets)
        candidates = [d for d in dets if lowest - d.py <= 15.0]
        return min(candidates, key=lambda d: abs(d.px - self.cfg.cx))

    def _cone_in_grasp_zone(self, dets: list[Detection]) -> bool:
        c = self.cfg.control
        zone = self.cfg.detector.grasp_zone_rows
        for d in dets:
            if abs(d.px - self.cfg.cx) <= 3 * c.tol_x_px and any(
                abs(g.target_row - d.py) <= zone for g in self.cfg.grasps
            ):
                return True
        return False

    # --- glowna petla ----------------------------------------------------
    def step(self) -> bool:
        """Jedna iteracja. Zwraca False, gdy robot skonczyl (DONE)."""
        if self.state == State.DONE:
            return False
        frame, _depth = self.camera.read()
        dets = self.detector.detect(frame)
        det = self._pick_target(dets)
        now = self.clock.now()
        self.stats.frames += 1
        if self._h0 is None and self.heading is not None:
            # kierunek pierwszego pasa = kurs na starcie, zanim ewentualny podjazd do szyszki obroci robota
            self._h0 = self.heading.yaw()
        if det is not None:
            self._last_seen = now
            self._last_det = det
        cmd = Command()

        if self.state == State.SEARCH:
            c = self.cfg.control
            if det is not None:
                self._goto(State.APPROACH, f"widze szyszke px={det.px:.0f} py={det.py:.0f}")
            elif self._explore_done(now):
                self.base.stop()
                self._goto(State.DONE, "przeszukane, nic nie widac")
            else:
                v, w = self._explore_cmd(now)
                self._drive(v, w)

        elif self.state == State.APPROACH:
            if det is None:
                c = self.cfg.control
                lost_for = now - self._last_seen
                if lost_for > c.lost_timeout_s:
                    self.base.stop()
                    self._retry_count = 0  # nowa szyszka = nowa pula prob
                    self._goto(State.SEARCH, "zgubilem szyszke")
                elif lost_for <= c.lost_hold_s:
                    # migotanie: dokoncz ostatni obrot przez chwile, bez jazdy do przodu na slepo
                    self._drive(0.0, self._last_cmd.w)
                elif self._last_det is not None and self._last_det.py < self.cfg.image_h * c.far_row_frac:
                    # byla daleko (u gory obrazu, na granicy zasiegu): pelznij do przodu, az wejdzie w kadr
                    self._drive(c.far_creep_v, 0.0)
                else:
                    self._drive(0.0, 0.0)
            elif det.partial:
                # szyszka ucieta krawedzia (daleko lub z boku): skrec w jej strone i pelznij, az bedzie cala w kadrze
                c = self.cfg.control
                self._settle = 0
                err_x = det.px - self.cfg.cx
                w = 0.0 if abs(err_x) < c.deadband_px else -c.kx * err_x * c.steer_sign
                w = min(max(w, -c.w_max), c.w_max)
                v = c.far_creep_v * max(0.0, 1.0 - abs(err_x) / 80.0)
                cmd = Command(v, w, err_x, 0.0, False, 0.0)
                self._drive(v, w)
            else:
                cmd = self.controller.compute(det)
                if cmd.in_tol:
                    self._settle += 1
                    self._drive(0.0, 0.0)
                    if self._settle >= self.cfg.control.settle_frames:
                        self.base.stop()
                        self._goto(State.ALIGN, f"ustawiony px={det.px:.0f} py={det.py:.0f}")
                else:
                    self._settle = 0
                    self._drive(cmd.v, cmd.w)

        elif self.state == State.ALIGN:
            self.base.stop()
            if det is None or det.partial:
                self._goto(State.APPROACH, "zniknela przy potwierdzaniu")
            else:
                cmd = self.controller.compute(det)
                if cmd.in_tol:
                    self._align_frames += 1
                    if self._align_frames >= 2:
                        self._goto(State.GRASP, f"potwierdzone, chwyt {self.cfg.grasp_for_row(det.py).name}")
                else:
                    self._goto(State.APPROACH, "po zatrzymaniu poza tolerancja")

        elif self.state == State.GRASP:
            self.base.stop()
            py = det.py if det is not None else (self._last_det.py if self._last_det else self.cfg.grasps[0].target_row)
            grasp = self.cfg.grasp_for_row(py)
            self.stats.grasp_attempts += 1
            result = self.arm.replay(grasp.name)
            if result is None:
                # ramie nie wie; sprawdz kamera, czy szyszka dalej lezy w strefie chwytu
                frame2, _ = self.camera.read()
                result = not self._cone_in_grasp_zone(self.detector.detect(frame2))
            if result:
                self._goto(State.DROP, f"{grasp.name} OK")
            else:
                self.stats.grasp_failures += 1
                self._goto(State.RETRY, f"{grasp.name} pusty chwytak")

        elif self.state == State.DROP:
            self.arm.replay("drop_box")
            self.arm.home()
            self.stats.collected += 1
            self._retry_count = 0
            self._goto(State.SEARCH, f"zebrane {self.stats.collected}")

        elif self.state == State.RETRY:
            self._retry_count += 1
            c = self.cfg.control
            if self._retry_count > c.retries:
                self._retry_count = 0
                # odwroc sie od tej szyszki i szukaj innej
                self._timed_drive(0.0, c.search_w, math.radians(60) / max(c.search_w, 1e-3))
                self._goto(State.SEARCH, "porzucam te szyszke")
            else:
                self._timed_drive(c.retry_back_v, 0.0, c.retry_back_s)
                # przez caly chwyt kamera nie widziala szyszki (ramie w kadrze); bez tego lost_timeout
                # wyrzucilby nas do SEARCH w pierwszej klatce APPROACH
                self._last_seen = self.clock.now()
                self._goto(State.APPROACH, f"ponowna proba {self._retry_count}")

        if self._log is not None:
            self._log.writerow([
                f"{now:.3f}", self.state.value, len(dets),
                f"{det.px:.1f}" if det else "", f"{det.py:.1f}" if det else "",
                f"{cmd.err_x:.1f}", f"{cmd.err_y:.1f}",
                f"{self._last_cmd.v:.3f}", f"{self._last_cmd.w:.3f}", self.stats.collected,
                f"{math.degrees(self._last_yaw):.1f}" if self._last_yaw is not None else "",
            ])
        if self.on_frame is not None:
            self.on_frame(frame, dets, self.state, cmd)
        return self.state != State.DONE

    def run(self, max_seconds: float | None = None) -> Stats:
        period = 1.0 / self.cfg.control.loop_hz
        t0 = self.clock.now()
        try:
            while True:
                t_start = self.clock.now()
                if not self.step():
                    break
                if max_seconds is not None and self.clock.now() - t0 > max_seconds:
                    if self.verbose:
                        print("limit czasu")
                    break
                self.clock.sleep(max(0.0, period - (self.clock.now() - t_start)))
        finally:
            self.base.stop()
            if self._log_fh:
                self._log_fh.close()
        return self.stats
