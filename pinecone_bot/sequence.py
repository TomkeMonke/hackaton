"""
Sekwencje: zhardkodowane zbieranie szyszek jako lista krokow jazdy i ruchow ramienia,
odtwarzana po kolei (serwer: web_control.py, tryb "sequence"; UI: frontend.html).

Krok (slownik, tak jak lezy w sequences/<name>.json):
    {"type": "drive", "speed": 0.3, "steer": 0.0, "seconds": 2.0}
        jazda ze stalym celem (ulamki -1..1 jak WASD, bez limitu predkosci z suwaka),
        po uplywie czasu stop i krotkie ustalenie (settle_s)
    {"type": "arm", "name": "grasp_cam"}
        ruch z motions/ przez panel ramienia (tools/arm_web.py, :8010); krok konczy sie,
        gdy panel ramienia zglosi "gotowe" (busy=None, kolejka pusta)
    {"type": "wait", "seconds": 1.0}

Runner dostaje wstrzykniete funkcje (jazda, ramie, sleep, zegar), wiec testuje sie bez
sprzetu i bez sieci. Jedna sekwencja naraz. STOP: stop() przerywa biezacy krok, zeruje
jazde i wysyla STOP do ramienia. Sekwencja nie ma petli ani warunkow: to celowo prosty
"zhardkodowany" przebieg, ktory czlowiek uklada i sprawdza krok po kroku.
"""
from __future__ import annotations

import json
import logging
import os
import re
import threading
import time
from typing import Callable

log = logging.getLogger(__name__)

STEP_TYPES = ("drive", "arm", "wait")
MAX_STEPS = 60
MAX_DRIVE_SECONDS = 30.0
MAX_WAIT_SECONDS = 60.0
ARM_TIMEOUT_S = 120.0        # tyle max czeka na koniec ruchu ramienia
SEQ_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


# ---------------------------------------------------------------------------
# format krokow
# ---------------------------------------------------------------------------

def _num(raw, name: str, lo: float, hi: float) -> float:
    try:
        val = float(raw)
    except (TypeError, ValueError):
        raise ValueError(f"{name}: to nie liczba ({raw!r})") from None
    if val != val or not lo <= val <= hi:
        raise ValueError(f"{name}: {val} poza zakresem {lo}..{hi}")
    return val


def parse_steps(raw) -> list:
    """Sprawdza i normalizuje liste krokow. ValueError z czytelnym komunikatem."""
    if not isinstance(raw, list):
        raise ValueError("kroki musza byc lista")
    if not raw:
        raise ValueError("pusta sekwencja")
    if len(raw) > MAX_STEPS:
        raise ValueError(f"za duzo krokow (max {MAX_STEPS})")
    steps = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f"krok {i}: nie jest slownikiem")
        kind = item.get("type")
        if kind == "drive":
            steps.append({
                "type": "drive",
                "speed": _num(item.get("speed", 0.0), f"krok {i} speed", -1.0, 1.0),
                "steer": _num(item.get("steer", 0.0), f"krok {i} steer", -1.0, 1.0),
                "seconds": _num(item.get("seconds"), f"krok {i} seconds", 0.0, MAX_DRIVE_SECONDS),
            })
        elif kind == "arm":
            name = str(item.get("name") or "")
            if not SEQ_NAME_RE.match(name):
                raise ValueError(f"krok {i}: zla nazwa ruchu ramienia {name!r}")
            steps.append({"type": "arm", "name": name})
        elif kind == "wait":
            steps.append({"type": "wait", "seconds": _num(item.get("seconds"), f"krok {i} seconds", 0.0, MAX_WAIT_SECONDS)})
        else:
            raise ValueError(f"krok {i}: nieznany typ {kind!r} (jest: {', '.join(STEP_TYPES)})")
    return steps


def step_text(step: dict) -> str:
    if step["type"] == "drive":
        return f"jazda {step['seconds']:g} s (speed {step['speed']:+.2f}, steer {step['steer']:+.2f})"
    if step["type"] == "arm":
        return f"ramie: {step['name']}"
    return f"czekaj {step['seconds']:g} s"


def sequence_path(sequences_dir: str, name: str) -> str:
    return os.path.join(sequences_dir, f"{name}.json")


def load_sequences(sequences_dir: str) -> dict:
    """{name: steps}; pliki z bledami sa pomijane z ostrzezeniem w logu."""
    out = {}
    if not os.path.isdir(sequences_dir):
        return out
    for fname in sorted(os.listdir(sequences_dir)):
        if not fname.endswith(".json"):
            continue
        path = os.path.join(sequences_dir, fname)
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
            out[fname[:-5]] = parse_steps(data["steps"] if isinstance(data, dict) else data)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            log.warning("pomijam %s: %s", path, exc)
    return out


def save_sequence(sequences_dir: str, name: str, steps: list) -> str:
    if not SEQ_NAME_RE.match(name or ""):
        raise ValueError("nazwa: litery, cyfry, '-' i '_' (max 40)")
    steps = parse_steps(steps)
    os.makedirs(sequences_dir, exist_ok=True)
    path = sequence_path(sequences_dir, name)
    with open(path, "w", encoding="ascii") as fh:
        json.dump({"name": name, "steps": steps}, fh, indent=2, ensure_ascii=True)
        fh.write("\n")
    return path


def delete_sequence(sequences_dir: str, name: str) -> bool:
    path = sequence_path(sequences_dir, name)
    if os.path.exists(path):
        os.remove(path)
        return True
    return False


# ---------------------------------------------------------------------------
# odtwarzanie
# ---------------------------------------------------------------------------

class SequenceRunner:
    """Wykonuje kroki po kolei w osobnym watku (start) albo w biezacym (run).

    drive(speed, steer)   ustawia cel jazdy; (0, 0) = stop
    arm_start(name)       -> (ok, msg): zleca ruch ramienia (POST /api/cmd motion)
    arm_state()           -> dict ze snapshotu panelu ramienia (busy, queue, error, result)
                             albo None, gdy panel nie odpowiada
    arm_stop()            STOP ramienia (przy przerwaniu sekwencji)
    """

    def __init__(
        self,
        drive: Callable[[float, float], None],
        arm_start: Callable[[str], tuple],
        arm_state: Callable[[], dict | None],
        arm_stop: Callable[[], None],
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        poll_s: float = 0.25,
        settle_s: float = 0.3,
        arm_timeout_s: float = ARM_TIMEOUT_S,
    ):
        self._drive = drive
        self._arm_start = arm_start
        self._arm_state = arm_state
        self._arm_stop = arm_stop
        self._sleep = sleep
        self._clock = clock
        self.poll_s = poll_s
        self.settle_s = settle_s
        self.arm_timeout_s = arm_timeout_s

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.running = False
        self.name = ""
        self.steps: list = []
        self.index = -1          # biezacy krok (-1 = przed startem / po koncu)
        self.error: str | None = None
        self.finished: str = ""  # "ukonczona" | "przerwana" | "blad: ..."

    # --- sterowanie --------------------------------------------------------

    def start(self, name: str, steps: list) -> bool:
        """Watek z sekwencja. False, gdy inna jeszcze trwa."""
        with self._lock:
            if self.running:
                return False
            self.running = True
        self._thread = threading.Thread(target=self.run, args=(name, steps, True), name="sequence", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()

    def status(self) -> dict:
        with self._lock:
            step = self.steps[self.index] if 0 <= self.index < len(self.steps) else None
            return {
                "running": self.running,
                "name": self.name,
                "index": self.index,
                "total": len(self.steps),
                "step": step_text(step) if step else None,
                "error": self.error,
                "finished": self.finished,
            }

    # --- wykonanie ---------------------------------------------------------

    def run(self, name: str, steps: list, _started: bool = False) -> bool:
        """Blokujaco. True = wszystkie kroki wykonane."""
        with self._lock:
            if not _started and self.running:
                return False
            self.running = True
            self.name = name
            self.steps = list(steps)
            self.index = -1
            self.error = None
            self.finished = ""
        self._stop.clear()
        ok = False
        try:
            for i, step in enumerate(self.steps):
                with self._lock:
                    self.index = i
                log.info("sekwencja %s krok %d/%d: %s", name, i + 1, len(self.steps), step_text(step))
                self._do_step(step)
            ok = True
            with self._lock:
                self.finished = "ukonczona"
        except _Aborted as exc:
            with self._lock:
                self.finished = str(exc) or "przerwana"
            log.info("sekwencja %s: %s", name, self.finished)
        except Exception as exc:  # noqa: BLE001 - blad jednego kroku nie moze ubic serwera
            log.exception("sekwencja %s: blad", name)
            with self._lock:
                self.error = f"{step_text(self.steps[self.index])}: {exc}"
                self.finished = "blad"
        finally:
            self._drive(0.0, 0.0)
            if not ok:
                try:
                    self._arm_stop()
                except Exception as exc:  # noqa: BLE001
                    log.warning("STOP ramienia nieudany: %s", exc)
            with self._lock:
                self.running = False
                self.index = -1
        return ok

    def _check_stop(self) -> None:
        if self._stop.is_set():
            raise _Aborted("przerwana")

    def _pause(self, seconds: float) -> None:
        """Spi po kawalku, zeby STOP dzialal w ciagu ~0.1 s."""
        end = self._clock() + seconds
        while True:
            self._check_stop()
            left = end - self._clock()
            if left <= 0:
                return
            self._sleep(min(0.1, left))

    def _do_step(self, step: dict) -> None:
        self._check_stop()
        kind = step["type"]
        if kind == "wait":
            self._pause(step["seconds"])
        elif kind == "drive":
            self._drive(step["speed"], step["steer"])
            try:
                self._pause(step["seconds"])
            finally:
                self._drive(0.0, 0.0)
            self._pause(self.settle_s)
        elif kind == "arm":
            ok, msg = self._arm_start(step["name"])
            if not ok:
                raise RuntimeError(f"panel ramienia odrzucil: {msg}")
            deadline = self._clock() + self.arm_timeout_s
            while True:
                self._pause(self.poll_s)
                st = self._arm_state()
                if st is None:
                    raise RuntimeError("panel ramienia nie odpowiada")
                if st.get("busy") is None and not st.get("queue"):
                    if st.get("error"):
                        raise RuntimeError(f"ramie: {st['error']}")
                    if str(st.get("result") or "").startswith("przerwane"):
                        raise _Aborted("przerwana (STOP ramienia)")
                    return
                if self._clock() > deadline:
                    raise RuntimeError(f"ramie nie skonczylo ruchu w {self.arm_timeout_s:g} s")
        else:
            raise ValueError(f"nieznany typ kroku {kind!r}")


class _Aborted(Exception):
    """Sekwencja przerwana przez stop()."""
