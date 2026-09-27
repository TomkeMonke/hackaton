"""
Podglad kamery z detekcja szyszek dla panelu webowego (tools/vision_web.py).

VisionFeed w osobnym watku czyta klatki, puszcza detektor HSV i trzyma ostatni wynik:
  - summary(): ile szyszek widac (cale + uciete krawedzia), lista detekcji, odleglosc z glebi,
    fps, historia liczby szyszek (wykres w panelu), rekord sesji,
  - jpeg(view): klatka w jednym z widokow: "overlay" (ramki detekcji), "raw", "mask" (prog HSV),
    "depth" (glebia w kolorach). Kodowanie JPEG leniwe i z pamiecia podreczna na numer klatki.

Zrodla klatek: RealSense (make_camera), plik/katalog, "sim" (symulator) albo URL obrazka JPEG
(HttpJpegCamera) - np. podglad lerobot, gdy kamere trzyma inny proces.
Bez sieci i bez sprzetu w tym module: testy podaja wlasna kamere.
"""
from __future__ import annotations

import math
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np

from .detector import Detection, HsvConeDetector

VIEWS = ("overlay", "raw", "mask", "depth")
HISTORY_S = 60.0


class HttpJpegCamera:
    """Kamera z adresu, ktory oddaje pojedynczy JPEG (np. http://pi:8081/cam/front.jpg)."""

    def __init__(self, url: str, timeout: float = 2.0):
        self.url = url
        self.timeout = timeout

    def read(self):
        with urllib.request.urlopen(self.url, timeout=self.timeout) as r:  # noqa: S310 (adres od operatora)
            data = r.read()
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError(f"{self.url}: to nie jest JPEG")
        return bgr, None

    def close(self) -> None:
        pass


def depth_at(depth_m: np.ndarray | None, px: float, py: float, r: int = 4) -> float | None:
    """Mediana glebi [m] w kwadracie 2r+1 wokol (px, py); None, gdy brak pomiaru (same zera)."""
    if depth_m is None:
        return None
    h, w = depth_m.shape[:2]
    x, y = int(round(px)), int(round(py))
    if not (0 <= x < w and 0 <= y < h):
        return None
    patch = depth_m[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1]
    good = patch[patch > 0]
    if good.size == 0:
        return None
    return float(np.median(good))


def depth_colormap(depth_m: np.ndarray, max_m: float = 1.5) -> np.ndarray:
    """Glebia -> obraz BGR (blisko = cieplo). Brak pomiaru na czarno."""
    d = np.clip(depth_m / max_m, 0.0, 1.0)
    img = cv2.applyColorMap((255 - d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[depth_m <= 0] = 0
    return img


class VisionFeed:
    def __init__(self, camera, detector: HsvConeDetector, hz: float = 10.0, jpeg_quality: int = 70,
                 source: str = "", clock=time.monotonic, wall=time.time):
        self.camera = camera
        self.detector = detector
        self.hz = hz
        self.jpeg_quality = int(jpeg_quality)
        self.source = source
        self._clock = clock
        self._wall = wall
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.seq = 0
        self._bgr: np.ndarray | None = None
        self._depth: np.ndarray | None = None
        self._mask: np.ndarray | None = None
        self._dets: list[Detection] = []
        self._dists: list[float | None] = []
        self._jpeg_cache: dict[str, tuple[int, bytes]] = {}
        self._frame_times: deque[float] = deque(maxlen=30)
        self._history: deque[tuple[float, int]] = deque()
        self.max_seen = 0
        self.frames = 0
        self.errors = 0
        self.error: str | None = None
        self.last_t: float | None = None

    # --- petla -----------------------------------------------------------

    def step(self) -> bool:
        """Jedna klatka: odczyt, detekcja, zapis wyniku. False, gdy kamera rzucila wyjatek."""
        try:
            bgr, depth = self.camera.read()
            if bgr is None:
                raise RuntimeError("kamera oddala pusta klatke")
            dets = self.detector.detect(bgr)
            mask = self.detector.last_mask
        except Exception as exc:  # noqa: BLE001 - kazdy blad kamery ma byc widoczny w panelu, nie zabic watku
            with self._cond:
                self.errors += 1
                self.error = f"{type(exc).__name__}: {exc}"
            return False
        now = self._clock()
        if depth is not None and depth.shape[:2] != bgr.shape[:2]:
            depth = cv2.resize(depth, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        dists = [depth_at(depth, d.px, d.py) for d in dets]
        with self._cond:
            self.seq += 1
            self.frames += 1
            self.error = None
            self._bgr, self._depth, self._mask = bgr, depth, mask
            self._dets, self._dists = dets, dists
            self._jpeg_cache.clear()
            self._frame_times.append(now)
            self.last_t = now
            self._history.append((now, len(dets)))
            while self._history and now - self._history[0][0] > HISTORY_S:
                self._history.popleft()
            self.max_seen = max(self.max_seen, sum(1 for d in dets if not d.partial))
            self._cond.notify_all()
        return True

    def run(self) -> None:
        period = 1.0 / self.hz if self.hz > 0 else 0.0
        while not self._stop.is_set():
            t0 = time.monotonic()
            if not self.step():
                self._stop.wait(1.0)  # kamera odpadla: nie mielimy CPU, probujemy co sekunde
                continue
            dt = time.monotonic() - t0
            if period > dt:
                self._stop.wait(period - dt)

    def start(self) -> None:
        self._thread = threading.Thread(target=self.run, name="vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        try:
            self.camera.close()
        except Exception:  # noqa: BLE001
            pass

    # --- odczyt dla serwera ---------------------------------------------

    def wait_frame(self, after_seq: int, timeout: float = 2.0) -> int:
        """Czeka na klatke nowsza niz after_seq; oddaje aktualny numer (moze byc stary po timeoucie)."""
        with self._cond:
            self._cond.wait_for(lambda: self.seq > after_seq or self._stop.is_set(), timeout=timeout)
            return self.seq

    def fps(self) -> float:
        with self._cond:
            t = list(self._frame_times)
        if len(t) < 2 or t[-1] <= t[0]:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0])

    def summary(self) -> dict:
        fps = self.fps()
        with self._cond:
            dets, dists = list(self._dets), list(self._dists)
            h, w = self._bgr.shape[:2] if self._bgr is not None else (0, 0)
            now = self._clock()
            hist = [[round(t - now, 1), n] for t, n in self._history]
            full = [d for d in dets if not d.partial]
            known = [x for x, d in zip(dists, dets) if x is not None and not d.partial]
            return {
                "count": len(full),
                "partial": len(dets) - len(full),
                "total": len(dets),
                "max_seen": self.max_seen,
                "nearest_m": round(min(known), 3) if known else None,
                "detections": [
                    {"px": round(d.px, 1), "py": round(d.py, 1), "area": round(d.area),
                     "bbox": list(d.bbox), "partial": d.partial,
                     "dist_m": None if x is None else round(x, 3)}
                    for d, x in zip(dets, dists)
                ],
                "frame": [w, h],
                "has_depth": self._depth is not None,
                "fps": round(fps, 1),
                "seq": self.seq,
                "frames": self.frames,
                "errors": self.errors,
                "error": self.error,
                "age_s": None if self.last_t is None else round(now - self.last_t, 2),
                "history": hist,
                "source": self.source,
                "ts": round(self._wall(), 3),
            }

    def render(self, view: str) -> np.ndarray | None:
        """Obraz BGR dla widoku (bez kodowania). None, gdy jeszcze nie bylo klatki."""
        with self._cond:
            bgr, depth, mask = self._bgr, self._depth, self._mask
            dets, dists = list(self._dets), list(self._dists)
        if bgr is None:
            return None
        if view == "raw":
            return bgr
        if view == "mask":
            if mask is None:
                return np.zeros_like(bgr)
            vis = (bgr * 0.25).astype(np.uint8)
            vis[mask > 0] = bgr[mask > 0]
            return vis
        if view == "depth":
            if depth is None:
                vis = (bgr * 0.3).astype(np.uint8)
                cv2.putText(vis, "brak glebi", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                return vis
            return depth_colormap(depth)
        vis = HsvConeDetector.draw_debug(bgr, dets)
        for d, x in zip(dets, dists):
            if x is not None:
                bx, by, bw, bh = d.bbox
                cv2.putText(vis, f"{x * 100:.0f} cm", (bx, by + bh + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return vis

    def jpeg(self, view: str = "overlay") -> tuple[int, bytes | None]:
        """(numer klatki, JPEG) dla widoku; nieznany widok = overlay."""
        if view not in VIEWS:
            view = "overlay"
        with self._cond:
            seq = self.seq
            cached = self._jpeg_cache.get(view)
            if cached is not None and cached[0] == seq:
                return cached
        img = self.render(view)
        if img is None:
            return seq, None
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        data = buf.tobytes() if ok else None
        with self._cond:
            if data is not None and self.seq == seq:
                self._jpeg_cache[view] = (seq, data)
        return seq, data

    def raw_frame(self) -> np.ndarray | None:
        with self._cond:
            return None if self._bgr is None else self._bgr.copy()


def sim_camera(cfg, period_s: float = 20.0):
    """Kamera z symulatora do pokazu bez sprzetu: robot stoi i powoli rozglada sie na boki."""
    from .sim import SimCamera, SimWorld

    # szyszki w polu widzenia kamery z cfg.sim (0.3..1.4 m przed robotem), zeby demo cos liczylo
    cones = [(0.45, -0.12), (0.6, 0.15), (0.8, -0.25), (1.0, 0.05), (1.2, 0.3), (0.7, 0.5), (0.9, -0.55)]

    t0 = time.monotonic()

    def pose():
        return 0.0, 0.0, 0.6 * math.sin(2 * math.pi * (time.monotonic() - t0) / period_s)

    return SimCamera(SimWorld(cfg, pose, cones=cones))
