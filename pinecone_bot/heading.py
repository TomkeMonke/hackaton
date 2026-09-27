"""
Zrodla kursu robota (kat obrotu wokol osi pionowej) dla pasow w SEARCH.

Interfejs: yaw() -> radiany (dodatni = obrot w lewo / CCW, ciagly, bez zawijania) albo None, gdy kurs
nieznany (brak danych, telefon zgasl, siec padla). Brain uzywa tylko roznic kursu, zero jest dowolne.

PhyphoxGyro: telefon przyklejony plasko do bazy, aplikacja phyphox, eksperyment "Gyroscope (rotation
rate)", w menu "Allow remote access". phyphox wystawia serwer HTTP; odpytujemy go o nowe probki
predkosci katowej wokol osi Z telefonu (prostopadlej do ekranu) i calkujemy je w kurs.
Protokol (dokumentacja phyphox, "remote interface"):
  GET /get?gyr_time                            -> ostatnia wartosc bufora
  GET /get?gyrZ=T|gyr_time&gyr_time=T          -> wszystkie probki z gyr_time > T
  GET /control?cmd=start                       -> start pomiaru
Odpowiedz: {"buffer": {"gyrZ": {"buffer": [...]}, "gyr_time": {"buffer": [...]}}, "status": {"measuring": ...}}
"""
from __future__ import annotations

import json
import threading
import time
import urllib.request

from .config import Config, HeadingConfig

MAX_GAP_S = 0.5   # przerwa miedzy probkami wieksza niz to = pomiar byl wstrzymany, nie calkuj przez nia


def _http_get_json(url: str, timeout: float = 0.5) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as resp:  # noqa: S310 - adres z configu
        return json.loads(resp.read().decode("utf-8"))


class PhyphoxGyro:
    def __init__(self, cfg: HeadingConfig, fetch=None, clock=time.monotonic):
        self.cfg = cfg
        self.url = cfg.phyphox_url.rstrip("/")
        self._fetch = fetch or _http_get_json
        self._clock = clock
        self._lock = threading.Lock()
        self._yaw = 0.0
        self._last_t: float | None = None     # czas (zegar phyphox) ostatniej probki
        self._last_w = 0.0
        self._last_new: float | None = None   # zegar lokalny ostatniej odpowiedzi z nowymi probkami
        self._last_start_req = -1e9
        self.last_error: Exception | None = None
        self.samples = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- odpytywanie -------------------------------------------------------
    def poll(self) -> int:
        """Jedno zapytanie. Zwraca liczbe nowych probek (0 przy bledzie albo braku danych)."""
        z, t = self.cfg.gyro_buffer, self.cfg.time_buffer
        if self._last_t is None:
            query = f"{t}&{z}"   # tylko ostatnia probka: calkujemy od teraz
        else:
            th = repr(self._last_t)
            query = f"{z}={th}|{t}&{t}={th}"
        try:
            data = self._fetch(f"{self.url}/get?{query}")
            ts = data["buffer"][t]["buffer"]
            ws = data["buffer"][z]["buffer"]
            measuring = bool(data.get("status", {}).get("measuring", True))
        except Exception as exc:  # noqa: BLE001 - siec, zly JSON, zly bufor: kurs po prostu nieznany
            self.last_error = exc
            return 0
        if not measuring:
            self._request_start()
        n = self._integrate(ts, ws)
        if n:
            with self._lock:
                self._last_new = self._clock()
        return n

    def _request_start(self) -> None:
        now = self._clock()
        if now - self._last_start_req < 2.0:
            return
        self._last_start_req = now
        try:
            self._fetch(f"{self.url}/control?cmd=start")
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    def _integrate(self, ts: list, ws: list) -> int:
        """Calkowanie trapezami. Probki z null (NaN w phyphox) pomijane."""
        n = 0
        with self._lock:
            for ti, wi in zip(ts, ws):
                if ti is None or wi is None:
                    continue
                ti, wi = float(ti), float(wi) * self.cfg.sign
                if self._last_t is not None and ti <= self._last_t:
                    if ti < self._last_t - 1.0:   # zegar phyphox cofnal sie: pomiar zrestartowany
                        self._last_t, self._last_w = ti, wi
                    continue
                if self._last_t is not None and ti - self._last_t <= MAX_GAP_S:
                    self._yaw += 0.5 * (wi + self._last_w) * (ti - self._last_t)
                self._last_t, self._last_w = ti, wi
                n += 1
                self.samples += 1
        return n

    # -- watek -------------------------------------------------------------
    def start(self) -> "PhyphoxGyro":
        self._thread = threading.Thread(target=self._run, name="phyphox", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        period = 1.0 / max(self.cfg.poll_hz, 1.0)
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.poll()
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    # -- interfejs ---------------------------------------------------------
    def yaw(self) -> float | None:
        with self._lock:
            if self._last_new is None or self._clock() - self._last_new > self.cfg.stale_s:
                return None
            return self._yaw


class OdometryHeading:
    """Kurs z base.odometry() (bipropellant z hallotronow; w symulacji prawdziwy kat robota)."""

    def __init__(self, base):
        self.base = base

    def yaw(self) -> float | None:
        odo = self.base.odometry()
        return None if odo is None else float(odo[2])

    def close(self) -> None:
        pass


def make_heading(cfg: Config, base=None):
    """None, gdy cfg.heading.source == 'none' (pasy z czasu)."""
    src = cfg.heading.source
    if src == "none":
        return None
    if src == "odometry":
        return OdometryHeading(base)
    if src == "phyphox":
        return PhyphoxGyro(cfg.heading).start()
    raise ValueError(f"nieznane zrodlo kursu: {src!r} (none | phyphox | odometry)")
