"""
Nagranie pomieszczenia z kamery RealSense (D435) prosto w formacie dla RTAB-Map, na Pi.

Zapisuje to samo, co `bag_to_rtabmap.py` robi z bagu, tylko bez bagu (bag z Pi zapchalby pendrive,
a stary bag mial zepsute ekstrinsyki):

  <out>/rgb/000001.jpg       kolor
  <out>/depth/000001.png     glebia 16-bit w mm, wyrownana do koloru (rs.align na zywo)
  <out>/calib/rs_color.yaml  kalibracja koloru (format RTAB-Map)
  <out>/stamps.txt           znaczniki czasu w sekundach
  <out>/README_rtabmap.txt   jak z tego zrobic mape

Co sekunde drukuje stan: ile klatek, pokrycie glebia, predkosc obrotu (z przesuniecia obrazu)
i OSTRZEZENIA - za szybki obrot i dziury w nagraniu zepsuly poprzednia mape (spojne 54%).

  python tools/record_rgbd.py                         # 640x480, zapis 15 Hz, ~/mapy/<data>_rtabmap
  python tools/record_rgbd.py --hz 10 --out ~/mapy/sala --seconds 300
Ctrl+C konczy i dopisuje stamps.txt. Kamere trzyma jeden proces: najpierw zamknij rs_mjpeg_server.py.
"""
from __future__ import annotations

import argparse
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

MAX_YAW_DEG_S = 30.0     # szybciej -> odometria RTAB-Map gubi klatki
MIN_COVERAGE = 0.40      # mniej pikseli z glebia -> za blisko sciany albo patrzy w pustke
MAX_GAP_S = 0.3          # dziura miedzy zapisanymi klatkami
DEPTH_LO_MM, DEPTH_HI_MM = 300, 6000
SMALL_W = 160            # szerokosc obrazu do liczenia obrotu

CALIB_TEMPLATE = """%YAML:1.0
---
camera_name: {name}
image_width: {w}
image_height: {h}
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {fx:.6f}, 0., {cx:.6f}, 0., {fy:.6f}, {cy:.6f}, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0., 0., 0., 0., 0. ]
distortion_model: plumb_bob
rectification_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]
projection_matrix: !!opencv-matrix
   rows: 3
   cols: 4
   dt: d
   data: [ {fx:.6f}, 0., {cx:.6f}, 0., 0., {fy:.6f}, {cy:.6f}, 0., 0., 0., 1., 0. ]
"""

README = """Zestaw RGB-D dla RTAB-Map (tools/record_rgbd.py)
=================================================

{n} par kolor+glebia, {dur:.1f} s, {hz:.1f} Hz, {w}x{h}, kamera {cam}.
Glebia wyrownana do koloru, w mm (RGBDImages\\scale=1).
Ostrzezenia w trakcie: za szybki obrot {fast} s, dziury {gaps}, niskie pokrycie glebia {low} s.

  rgb:   rgb/
  depth: depth/
  calib: calib/rs_color.yaml
  stamps.txt

Mapowanie bez GUI (RTAB-Map 0.23.8 win64, jak w docs/LOG.md 2026-09-25):
  1. rtabmap-dataRecorder -hide rtabmap_source.ini raw.db
     ini: [Camera] type=0, rgbd\\driver=7, calibrationName=rs_color, calibrationPath=calib,
          RGBDImages\\path_rgb=rgb, RGBDImages\\path_depth=depth, RGBDImages\\scale=1,
          Images\\stamps=stamps.txt
  2. rtabmap-reprocess -odom --RGBD/LinearUpdate 0 --RGBD/AngularUpdate 0
       --Odom/ResetCountdown 1 --Vis/MinInliers 12 --Vis/CorType 1
       --Vis/MaxFeatures 2000 --Vis/MaxDepth 4 raw.db map.db
     (Vis/MaxDepth 4: na zewnatrz dalekie drzewa i niebo daja glebie-smieci do 30 m)
  3. rtabmap-detectMoreLoopClosures --inter -r 100 -a 180 -i 3 map.db
  4. rtabmap-export --cloud --poses --voxel 0.01 map.db
"""


def depth_coverage(depth_mm: np.ndarray, lo: int = DEPTH_LO_MM, hi: int = DEPTH_HI_MM) -> float:
    """Czesc pikseli z glebia w zakresie [lo, hi] mm."""
    if depth_mm.size == 0:
        return 0.0
    return float(np.count_nonzero((depth_mm >= lo) & (depth_mm <= hi))) / depth_mm.size


def small_gray(bgr: np.ndarray) -> np.ndarray:
    """Maly obraz szarosci (float32) do liczenia przesuniecia miedzy klatkami."""
    h, w = bgr.shape[:2]
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    return cv2.resize(g, (SMALL_W, max(1, round(h * SMALL_W / w))),
                      interpolation=cv2.INTER_AREA).astype(np.float32)


def yaw_rate_deg_s(prev: np.ndarray, cur: np.ndarray, dt: float, fx_small: float) -> float:
    """Predkosc obrotu (st/s) z poziomego przesuniecia obrazu (korelacja fazowa). Znak pomijany."""
    if dt <= 0:
        return 0.0
    (dx, _dy), _resp = cv2.phaseCorrelate(prev, cur)
    return float(np.degrees(np.arctan2(abs(dx), fx_small))) / dt


def warnings_for(yaw: float, coverage: float, gap: float) -> list[str]:
    out = []
    if yaw > MAX_YAW_DEG_S:
        out.append("ZA SZYBKI OBROT %.0f st/s (max %.0f)" % (yaw, MAX_YAW_DEG_S))
    if coverage < MIN_COVERAGE:
        out.append("MALO GLEBI %.0f%% (za blisko albo pustka)" % (100 * coverage))
    if gap > MAX_GAP_S:
        out.append("DZIURA %.2f s bez klatek" % gap)
    return out


def write_calibration(path: str, w: int, h: int, fx: float, fy: float, cx: float, cy: float,
                      name: str = "rs_color") -> None:
    # recznie, nie cv2.FileStorage: OpenCV 5 pisze "%YAML 1.2", RTAB-Map chce "%YAML:1.0"
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(CALIB_TEMPLATE.format(name=name, w=w, h=h, fx=fx, fy=fy, cx=cx, cy=cy))


def write_meta(out: str, stamps: list[float], w: int, h: int, cam: str,
               fast_s: float = 0.0, gaps: int = 0, low_s: float = 0.0) -> None:
    with open(os.path.join(out, "stamps.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write("".join("%.6f\n" % s for s in stamps))
    dur = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    with open(os.path.join(out, "README_rtabmap.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(README.format(n=len(stamps), dur=dur, hz=(len(stamps) - 1) / dur if dur else 0.0,
                              w=w, h=h, cam=cam, fast=round(fast_s, 1), gaps=gaps,
                              low=round(low_s, 1)))


class FrameWriter:
    """Zapis klatek w osobnym watku: pendrive potrafi stanac na chwile, kamera nie moze czekac."""

    def __init__(self, out: str, maxsize: int = 90, jpeg_quality: int = 95):
        self.rgb_dir = os.path.join(out, "rgb")
        self.depth_dir = os.path.join(out, "depth")
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)
        self.jpeg_quality = jpeg_quality
        self.q: queue.Queue = queue.Queue(maxsize=maxsize)
        self.error: Exception | None = None
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def put(self, index: int, color: np.ndarray, depth_mm: np.ndarray) -> bool:
        """False = kolejka pelna (dysk nie nadaza), klatka odrzucona."""
        try:
            self.q.put_nowait((index, color, depth_mm))
            return True
        except queue.Full:
            return False

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                return
            index, color, depth_mm = item
            base = "%06d" % index
            try:
                cv2.imwrite(os.path.join(self.rgb_dir, base + ".jpg"), color,
                            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                cv2.imwrite(os.path.join(self.depth_dir, base + ".png"), depth_mm)
            except Exception as e:  # noqa: BLE001 - zglaszane w petli glownej
                self.error = e

    def close(self) -> None:
        self.q.put(None)
        self._t.join()


def start_camera(width: int, height: int, fps: int, laser_power: float | None):
    import pyrealsense2 as rs  # leniwie: testy bez kamery

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    try:
        prof = pipe.start(cfg)
    except RuntimeError as e:
        raise RuntimeError("%s\nKamere moze trzymac inny proces (rs_mjpeg_server.py?): "
                           "pkill -f rs_mjpeg_server.py" % e) from e
    dev = prof.get_device()
    depth_sensor = dev.first_depth_sensor()
    if laser_power is not None and depth_sensor.supports(rs.option.laser_power):
        rng = depth_sensor.get_option_range(rs.option.laser_power)
        depth_sensor.set_option(rs.option.laser_power, max(rng.min, min(rng.max, laser_power)))
    scale_mm = depth_sensor.get_depth_scale() * 1000.0
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    name = dev.get_info(rs.camera_info.name)
    return pipe, rs.align(rs.stream.color), scale_mm, intr, name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="katalog wyjsciowy (domyslnie ~/mapy/<data>_rtabmap)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30, help="fps kamery")
    ap.add_argument("--hz", type=float, default=15.0, help="ile klatek na sekunde zapisywac")
    ap.add_argument("--seconds", type=float, default=0.0, help="koniec po tylu sekundach (0 = Ctrl+C)")
    ap.add_argument("--laser-power", type=float, default=None, help="moc projektora IR (D435: 0-360)")
    args = ap.parse_args()

    out = os.path.expanduser(args.out or os.path.join(
        "~", "mapy", time.strftime("%Y%m%d_%H%M%S") + "_rtabmap"))
    if os.path.isdir(os.path.join(out, "rgb")) and os.listdir(os.path.join(out, "rgb")):
        print("katalog %s ma juz klatki - podaj inny --out" % out)
        return 2
    os.makedirs(os.path.join(out, "calib"), exist_ok=True)

    pipe, align, scale_mm, intr, cam = start_camera(args.width, args.height, args.fps,
                                                    args.laser_power)
    write_calibration(os.path.join(out, "calib", "rs_color.yaml"),
                      intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
    fx_small = intr.fx * SMALL_W / intr.width
    print("%s %dx%d fx=%.1f, zapis %.0f Hz -> %s" % (cam, intr.width, intr.height, intr.fx,
                                                    args.hz, out))
    print("Ruszaj WOLNO (obrot < %.0f st/s), na koniec wroc na start. Ctrl+C konczy."
          % MAX_YAW_DEG_S)

    writer = FrameWriter(out)
    stamps: list[float] = []
    period = 1.0 / args.hz
    prev_small, prev_t = None, None
    last_saved_t = None
    fast_s = low_s = 0.0
    gaps = dropped = 0
    t_start = time.time()
    next_report = t_start + 1.0
    yaw = cov = gap = 0.0
    try:
        while True:
            fs = align.process(pipe.wait_for_frames(5000))
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if not c or not d:
                continue
            t = c.get_timestamp() / 1000.0
            if last_saved_t is not None and t - last_saved_t < period * 0.9:
                continue
            color = np.asanyarray(c.get_data()).copy()
            depth_mm = np.asanyarray(d.get_data())
            if abs(scale_mm - 1.0) > 1e-6:
                depth_mm = np.clip(depth_mm * scale_mm, 0, 65535)
            depth_mm = depth_mm.astype(np.uint16).copy()

            small = small_gray(color)
            dt = (t - prev_t) if prev_t is not None else 0.0
            yaw = yaw_rate_deg_s(prev_small, small, dt, fx_small) if prev_small is not None else 0.0
            cov = depth_coverage(depth_mm)
            gap = (t - last_saved_t) if last_saved_t is not None else 0.0
            prev_small, prev_t = small, t

            if writer.put(len(stamps) + 1, color, depth_mm):
                stamps.append(t)
                last_saved_t = t
            else:
                dropped += 1
            if yaw > MAX_YAW_DEG_S:
                fast_s += dt
            if cov < MIN_COVERAGE:
                low_s += dt
            if gap > MAX_GAP_S:
                gaps += 1
            warn = warnings_for(yaw, cov, gap)
            if writer.error:
                raise RuntimeError("zapis klatki: %s" % writer.error)

            now = time.time()
            if warn or now >= next_report:
                el = now - t_start
                print("%6.1f s  klatek %5d  glebia %3.0f%%  obrot %4.0f st/s  kolejka %2d%s%s" % (
                    el, len(stamps), 100 * cov, yaw, writer.q.qsize(),
                    "  odrzucone %d" % dropped if dropped else "",
                    ("  <<< " + "; ".join(warn)) if warn else ""), flush=True)
                next_report = now + 1.0
            if args.seconds and now - t_start >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()
        print("\nzapisuje reszte kolejki (%d)..." % writer.q.qsize(), flush=True)
        writer.close()
        if stamps:
            write_meta(out, stamps, intr.width, intr.height, cam, fast_s, gaps, low_s)

    dur = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    print("gotowe: %d klatek, %.1f s, %s" % (len(stamps), dur, out))
    print("za szybki obrot: %.1f s, dziury: %d, malo glebi: %.1f s, odrzucone (dysk): %d"
          % (fast_s, gaps, low_s, dropped))
    if fast_s > 0.1 * max(dur, 1e-6) or gaps > 3:
        print("UWAGA: duzo ostrzezen - mapa moze sie rozpasc na kawalki jak poprzednio. "
              "Rozwaz nagranie od nowa, wolniej.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
