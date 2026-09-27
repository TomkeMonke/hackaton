"""
Zrodla obrazu dla pinecone_bot.

    RealSenseCamera  - D415 przez pyrealsense2 (import leniwy, tylko na Pi)
    FileCamera       - obraz / katalog obrazow / plik wideo, do strojenia na sucho
    make_camera(cfg, source) - wybiera jedno z powyzszych

Kontrakt (Protocol Camera):
    read()  -> (bgr HxWx3 uint8, glebia w metrach float32 HxW albo None)
    close() -> zwalnia urzadzenie

Robot ustawia sie do szyszki WYLACZNIE po obrazie kolorowym (kolumna cfg.cx,
wiersz grasp.target_row). Glebia jest opcjonalna - na przeszkody, pozniej.

Dlaczego glebia 424x240, a kolor 640x480: minimalna odleglosc pomiaru D415
zalezy od rozdzielczosci strumienia glebi. Przy 640x480 to ok. 31 cm (zmierzone
w scan_cones.py), przy 424x240 spada do ok. 16 cm. Po rs.align glebia i tak
jest przeprobkowana do rozdzielczosci koloru, wiec piksel (u, v) na obrazie
kolorowym i depth[v, u] to ten sam promien.
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Protocol

import cv2
import numpy as np

from .config import Config

IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")
VIDEO_EXTS = (".mp4", ".avi", ".mkv", ".mov", ".webm")

DEPTH_W, DEPTH_H = 424, 240   # nizsza rozdzielczosc glebi = mniejszy min. dystans D415


class Camera(Protocol):
    def read(self) -> tuple[np.ndarray, np.ndarray | None]: ...
    def close(self) -> None: ...


@dataclass
class Intrinsics:
    """Parametry wewnetrzne strumienia KOLORU (glebia jest do niego wyrownana)."""
    fx: float
    fy: float
    ppx: float
    ppy: float
    width: int
    height: int


class RealSenseCamera:
    """
    D415: kolor cfg.image_w x cfg.image_h BGR8 @ fps, glebia 424x240 z16 wyrownana
    do koloru (tylko gdy depth=True). Konwencje jak w rs_snapshot.py / scan_cones.py:
    sprawdzenie, czy urzadzenie jest widoczne, kilka klatek na rozgrzanie
    auto-ekspozycji, pipeline.stop() w close().

    read() przy timeoucie wait_for_frames nie rzuca, tylko oddaje ostatnia dobra
    klatke i zlicza to w self.timeouts (petla sterowania nie ma sie wywracac
    przez jedna zgubiona klatke; jesli licznik rosnie, to kabel albo USB).
    """

    def __init__(self, cfg: Config, depth: bool = True, fps: int = 30,
                 warmup_frames: int | None = None, timeout_ms: int = 2000,
                 laser_power: float | None = None):
        import pyrealsense2 as rs  # leniwie: na laptopie bez kamery modul moze nie istniec
        self._rs = rs
        self.with_depth = depth
        self.timeout_ms = int(timeout_ms)
        self.timeouts = 0
        self.frames_read = 0
        self._last: tuple[np.ndarray, np.ndarray | None] | None = None

        if len(rs.context().query_devices()) == 0:
            raise RuntimeError("Nie widac kamery RealSense. Sprawdz kabel i realsense-viewer.")

        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, cfg.image_w, cfg.image_h, rs.format.bgr8, fps)
        if depth:
            config.enable_stream(rs.stream.depth, DEPTH_W, DEPTH_H, rs.format.z16, fps)
        try:
            profile = self._pipeline.start(config)
        except RuntimeError as exc:
            streams = f"kolor {cfg.image_w}x{cfg.image_h}@{fps}"
            if depth:
                streams += f" + glebia {DEPTH_W}x{DEPTH_H}"
            raise RuntimeError(
                f"Nie udalo sie wystartowac {streams}: {exc}\n"
                "Kamere moze trzymac inny proces - urzadzenie jest na wylacznosc."
            ) from exc

        self._align = rs.align(rs.stream.color) if depth else None
        self.depth_scale: float | None = None
        if depth:
            sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = float(sensor.get_depth_scale())
            if laser_power is not None and sensor.supports(rs.option.laser_power):
                rng = sensor.get_option_range(rs.option.laser_power)
                sensor.set_option(rs.option.laser_power, max(rng.min, min(rng.max, laser_power)))

        vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = vsp.get_intrinsics()
        self.intrinsics = Intrinsics(float(intr.fx), float(intr.fy),
                                     float(intr.ppx), float(intr.ppy),
                                     int(intr.width), int(intr.height))

        # Kamera pamieta opcje po poprzednim procesie: jesli ktos wczesniej zamrozil ekspozycje
        # (lock_auto), bez tego zostaje reczna wartosc i na sloncu obraz jest bialy.
        if not cfg.camera.lock_auto:
            self._enable_auto(profile)

        # Rozgrzanie auto-ekspozycji i auto white balance, jak w rs_snapshot.py. Bez tego
        # pierwsze klatki sa ciemne albo maja zle kolory i prog HSV na nich nie trafia.
        if warmup_frames is None:
            warmup_frames = cfg.camera.warmup_frames
        last = None
        for _ in range(max(0, int(warmup_frames))):
            try:
                last = self._pipeline.wait_for_frames(self.timeout_ms)
            except RuntimeError:
                break

        self.locked: dict[str, float] = {}
        if cfg.camera.lock_auto:
            self.locked = self._lock_auto(profile, last)

    def _color_sensor(self, profile):
        rs = self._rs
        for s in profile.get_device().query_sensors():
            if s.supports(rs.option.enable_auto_white_balance) or s.supports(rs.option.white_balance):
                return s
        return None

    def _enable_auto(self, profile) -> None:
        """Wlacz AWB i auto-ekspozycje koloru, jesli poprzedni proces je wylaczyl."""
        rs = self._rs
        sensor = self._color_sensor(profile)
        if sensor is None:
            return
        for opt in (rs.option.enable_auto_white_balance, rs.option.enable_auto_exposure):
            if sensor.supports(opt) and sensor.get_option(opt) != 1:
                sensor.set_option(opt, 1)

    def _lock_auto(self, profile, frames) -> dict[str, float]:
        """
        Zamroz AWB i auto-ekspozycje koloru na wartosciach z ostatniej klatki rozgrzewki.
        Samo wylaczenie auto wraca do recznej wartosci domyslnej, dlatego najpierw
        czytamy biezaca wartosc z metadanych klatki i wpisujemy ja recznie.
        Zwraca, co ustawiono (do logu); czego kamera nie wspiera, pomija.
        """
        rs = self._rs
        color = frames.get_color_frame() if frames is not None else None
        sensor = self._color_sensor(profile)
        if sensor is None:
            return {}
        pairs = [
            (rs.option.enable_auto_white_balance, rs.option.white_balance,
             rs.frame_metadata_value.white_balance, "white_balance"),
            (rs.option.enable_auto_exposure, rs.option.exposure,
             rs.frame_metadata_value.actual_exposure, "exposure"),
        ]
        done: dict[str, float] = {}
        for auto_opt, value_opt, meta, name in pairs:
            if not sensor.supports(auto_opt):
                continue
            value = None
            if color and color.supports_frame_metadata(meta):
                value = float(color.get_frame_metadata(meta))
            elif sensor.supports(value_opt):
                value = float(sensor.get_option(value_opt))
            sensor.set_option(auto_opt, 0)
            if value is not None and sensor.supports(value_opt):
                rng = sensor.get_option_range(value_opt)
                value = max(rng.min, min(rng.max, value))
                sensor.set_option(value_opt, value)
                done[name] = value
        print(f"[camera] zablokowane auto: {done}", file=sys.stderr)
        return done

    def read(self) -> tuple[np.ndarray, np.ndarray | None]:
        try:
            frames = self._pipeline.wait_for_frames(self.timeout_ms)
        except RuntimeError as exc:
            self.timeouts += 1
            if self._last is None:
                raise RuntimeError(f"Brak klatki z RealSense (timeout): {exc}") from exc
            print(f"[camera] timeout #{self.timeouts}, oddaje poprzednia klatke", file=sys.stderr)
            return self._last

        if self._align is not None:
            frames = self._align.process(frames)
        color = frames.get_color_frame()
        if not color:
            self.timeouts += 1
            if self._last is None:
                raise RuntimeError("RealSense oddal zestaw klatek bez koloru")
            return self._last

        # .copy(): tablica z get_data() wskazuje na bufor klatki, ktory librealsense
        # zwalnia, gdy obiekt frame zniknie. Kopia jest tania (0.9 MB).
        bgr = np.asanyarray(color.get_data()).copy()
        depth_m: np.ndarray | None = None
        if self.with_depth:
            d = frames.get_depth_frame()
            if d:
                # 0 = brak pomiaru (za blisko, brak tekstury). Zostaje 0.0, nie NaN,
                # zeby porownania w kodzie przeszkod byly proste (depth > 0).
                depth_m = np.asanyarray(d.get_data()).astype(np.float32) * self.depth_scale
        self._last = (bgr, depth_m)
        self.frames_read += 1
        return self._last

    def close(self) -> None:
        try:
            self._pipeline.stop()
        except RuntimeError:
            pass  # stop() przed start() albo kamera juz odlaczona


class FileCamera:
    """
    Zrodlo z dysku: pojedynczy obraz (oddawany w kolko), katalog obrazow
    (posortowany po nazwie, zapetlony) albo plik wideo (zapetlony).
    read() zwraca (bgr, None) - glebi z plikow nie ma.

    loop=False: po ostatniej klatce read() rzuca EOFError (do skryptow, ktore
    maja przejsc przez zbior raz).
    """

    def __init__(self, source: str, loop: bool = True):
        self.source = source
        self.loop = loop
        self.index = 0          # numer nastepnej klatki do oddania
        self.current_name = ""  # nazwa pliku ostatnio oddanej klatki (do podpisow)
        self._paths: list[str] = []
        self._cap: cv2.VideoCapture | None = None
        self._single: np.ndarray | None = None

        if os.path.isdir(source):
            self._paths = sorted(
                os.path.join(source, f) for f in os.listdir(source)
                if f.lower().endswith(IMAGE_EXTS)
            )
            if not self._paths:
                raise FileNotFoundError(f"Brak obrazow ({', '.join(IMAGE_EXTS)}) w {source}")
        elif os.path.isfile(source):
            ext = os.path.splitext(source)[1].lower()
            if ext in IMAGE_EXTS:
                img = cv2.imread(source, cv2.IMREAD_COLOR)
                if img is None:
                    raise IOError(f"Nie da sie wczytac obrazu {source}")
                self._single = img
                self.current_name = os.path.basename(source)
            else:
                self._cap = cv2.VideoCapture(source)
                if not self._cap.isOpened():
                    raise IOError(f"Nie da sie otworzyc wideo {source}")
                self.current_name = os.path.basename(source)
        else:
            raise FileNotFoundError(f"Nie ma takiego pliku ani katalogu: {source}")

    def __len__(self) -> int:
        if self._paths:
            return len(self._paths)
        if self._cap is not None:
            return max(0, int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT)))
        return 1

    def read(self) -> tuple[np.ndarray, np.ndarray | None]:
        if self._single is not None:
            self.index += 1
            return self._single.copy(), None

        if self._paths:
            if self.index >= len(self._paths):
                if not self.loop:
                    raise EOFError(f"Koniec katalogu {self.source}")
                self.index = 0
            path = self._paths[self.index]
            img = cv2.imread(path, cv2.IMREAD_COLOR)
            if img is None:
                raise IOError(f"Nie da sie wczytac obrazu {path}")
            self.current_name = os.path.basename(path)
            self.index += 1
            return img, None

        assert self._cap is not None
        ok, frame = self._cap.read()
        if not ok:
            if not self.loop:
                raise EOFError(f"Koniec wideo {self.source}")
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.index = 0
            ok, frame = self._cap.read()
            if not ok:
                raise IOError(f"Wideo {self.source} nie oddaje klatek")
        self.index += 1
        return frame, None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None


def make_camera(cfg: Config, source: str | None = None, depth: bool = True) -> Camera:
    """
    source None  -> RealSenseCamera(cfg, depth=depth)
    source str   -> FileCamera(source)  (obraz / katalog / wideo)
    source "sim" -> NIE tutaj; symulator dostarcza wlasny obiekt kamery.
    """
    if source is None:
        return RealSenseCamera(cfg, depth=depth)
    if source == "sim":
        raise ValueError("make_camera: 'sim' obsluguje symulator, nie ten modul")
    return FileCamera(source)


def paced_frames(cam: Camera, hz: float):
    """Generator klatek z ograniczeniem tempa (do podgladu plikow, zeby nie mignely)."""
    period = 1.0 / hz if hz > 0 else 0.0
    while True:
        t0 = time.monotonic()
        yield cam.read()
        dt = time.monotonic() - t0
        if period > dt:
            time.sleep(period - dt)
