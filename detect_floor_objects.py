"""
Detekcja dowolnych obiektow lezacych na podlodze, po samej glebi (bez koloru).

Pipeline:
    wyrownana glebia -> chmura punktow (wektorowa deprojekcja)
    -> przyciecie do obszaru roboczego (ROI)
    -> dopasowanie plaszczyzny podlogi (RANSAC + douczenie SVD na inlierach)
    -> maska punktow wystajacych nad plaszczyzne
    -> spojne obszary (cv2.connectedComponents) = obiekty
    -> dla kazdego: centroid XYZ, wysokosc, orientacja i szerokosc chwytu

Dlaczego bez open3d i DBSCAN: glebia z D415 jest ZORGANIZOWANA - to obraz, w
ktorym sasiedztwo pikseli jest juz sasiedztwem w przestrzeni. Etykietowanie
spojnych obszarow na masce 640x480 kosztuje ulamek milisekundy, podczas gdy
DBSCAN po ~300 tys. luznych punktow to setki milisekund na klatke. Przy okazji
nie dokladamy zaleznosci - numpy i cv2 juz sa w repo.

Uklad wspolrzednych jak w detect_object.py: optyczny uklad KOLORU, metry,
X w prawo, Y w dol, Z w glab. Glebia jest wyrownana do koloru, wiec intrinsics
biore z wyrownanej ramki glebi.

Uzycie:
    python detect_floor_objects.py                      # kalibracja podlogi + detekcja
    python detect_floor_objects.py --save-plane floor.json
    python detect_floor_objects.py --load-plane floor.json
    python detect_floor_objects.py --no-preview --seconds 10 --log obiekty.csv

Klawisze w podgladzie:
    q / ESC   wyjscie
    f         dopasuj podloge od nowa (zrob to przy pustej podlodze)
    m         przelacz widok maski obiektow
    s         zapisz klatke do floor_snapshot.png

Import z petli pick-and-place:
    from detect_floor_objects import FloorObjectDetector
    detector = FloorObjectDetector()
    detector.fit_floor(frames)            # raz, przy pustej podlodze
    obiekty = detector.detect(frames)     # lista slownikow, patrz DETECTION_KEYS
"""

from __future__ import annotations

import argparse
import csv
import json
import time

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 640, 480, 30

# Obszar roboczy ramienia - punkty poza nim nie wchodza ani do RANSAC, ani do detekcji.
MIN_DISTANCE = 0.3  # m; blizej D415 i tak nie mierzy
MAX_DISTANCE = 2.0  # m
MAX_SIDE = 1.0  # m; |X| od osi optycznej

PLANE_THRESHOLD = 0.01  # m; inlier plaszczyzny = punkt blizej niz 1 cm
RANSAC_ITERS = 300
RANSAC_SAMPLE = 6000  # ile punktow losujemy do szukania plaszczyzny

MIN_HEIGHT = 0.015  # m nad podloga, ponizej to szum i faktura podlogi
MAX_HEIGHT = 0.40  # m; wyzej to juz nie jest obiekt na podlodze
MIN_AREA_PX = 300  # mniejsze plamy odrzucamy

PRINT_EVERY = 0.5

DETECTION_KEYS = (
    "centroid",  # (x, y, z) w metrach, uklad kamery
    "height_m",  # wysokosc najwyzszego punktu nad plaszczyzna podlogi
    "width_m",  # krotszy bok prostokata = os chwytania
    "length_m",  # dluzszy bok
    "angle_deg",  # orientacja dluzszej osi w plaszczyznie podlogi
    "area_px",
    "pixels",  # (u, v) srodka masy w obrazie
)


# --- geometria ---------------------------------------------------------------


def pixel_grid(width: int, height: int, intrinsics):
    """Siatka (u - ppx)/fx i (v - ppy)/fy, liczona raz na rozmiar obrazu."""
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    return (uu - intrinsics.ppx) / intrinsics.fx, (vv - intrinsics.ppy) / intrinsics.fy


def deproject(depth_image: np.ndarray, grid, depth_scale: float) -> np.ndarray:
    """
    Cala ramka glebi -> HxWx3 metrow, wektorowo.

    To ten sam model co rs2_deproject_pixel_to_point, tylko bez petli po
    pikselach. Wspolczynniki dystorsji wyrownanego strumienia sa zerowe
    (sprawdzone na tej kamerze), wiec model liniowy jest tu dokladny.
    """
    x_norm, y_norm = grid
    z = depth_image.astype(np.float32) * depth_scale
    return np.dstack((x_norm * z, y_norm * z, z))


def fit_plane_ransac(points: np.ndarray, iters: int, threshold: float, rng):
    """
    Zwraca (normal, d) plaszczyzny n.p + d = 0, dopasowanej do najwiekszego
    zbioru punktow. Po RANSAC jest douczenie SVD na wszystkich inlierach -
    sama trojka losowych punktow daje plaszczyzne szarpiaca sie miedzy klatkami.
    """
    if len(points) < 3:
        return None

    best_normal, best_d, best_count = None, 0.0, 0
    for _ in range(iters):
        idx = rng.integers(0, len(points), size=3)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:  # punkty wspolliniowe
            continue
        normal = normal / norm
        d = -float(normal @ p0)
        count = int(np.count_nonzero(np.abs(points @ normal + d) < threshold))
        if count > best_count:
            best_normal, best_d, best_count = normal, d, count

    if best_normal is None:
        return None

    inliers = points[np.abs(points @ best_normal + best_d) < threshold]
    if len(inliers) >= 3:
        centroid = inliers.mean(axis=0)
        _, _, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
        best_normal = vh[2] / np.linalg.norm(vh[2])
        best_d = -float(best_normal @ centroid)

    # Znak tak, zeby punkty MIEDZY kamera a podloga mialy dodatnia odleglosc:
    # kamera jest w (0,0,0), wiec jej odleglosc to samo d.
    if best_d < 0:
        best_normal, best_d = -best_normal, -best_d
    return best_normal, best_d


def plane_basis(normal: np.ndarray):
    """Dwa wektory rozpinajace plaszczyzne, do rzutowania obiektow na podloge."""
    helper = np.array([1.0, 0.0, 0.0])
    if abs(normal @ helper) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(normal, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    return e1, e2 / np.linalg.norm(e2)


# --- detektor ----------------------------------------------------------------


class FloorObjectDetector:
    def __init__(
        self,
        min_distance=MIN_DISTANCE,
        max_distance=MAX_DISTANCE,
        max_side=MAX_SIDE,
        plane_threshold=PLANE_THRESHOLD,
        min_height=MIN_HEIGHT,
        max_height=MAX_HEIGHT,
        min_area_px=MIN_AREA_PX,
        seed=0,
    ):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.max_side = max_side
        self.plane_threshold = plane_threshold
        self.min_height = min_height
        self.max_height = max_height
        self.min_area_px = min_area_px
        self.rng = np.random.default_rng(seed)

        self.normal = None
        self.offset = None
        self.grid = None
        self.depth_scale = None
        self.last_mask = None

    # -- podloga --

    @property
    def has_floor(self) -> bool:
        return self.normal is not None

    def plane_dict(self) -> dict:
        return {"normal": self.normal.tolist(), "offset": self.offset}

    def load_plane(self, path: str) -> None:
        with open(path) as f:
            data = json.load(f)
        self.normal = np.array(data["normal"], dtype=np.float64)
        self.offset = float(data["offset"])

    def save_plane(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.plane_dict(), f, indent=2)

    def fit_floor(self, frames, iters=RANSAC_ITERS) -> bool:
        """Dopasowuje plaszczyzne podlogi do biezacej klatki. Rob to na pustej podlodze."""
        xyz, valid = self._cloud(frames)
        points = xyz[valid]
        if len(points) > RANSAC_SAMPLE:
            points = points[self.rng.choice(len(points), RANSAC_SAMPLE, replace=False)]
        result = fit_plane_ransac(points, iters, self.plane_threshold, self.rng)
        if result is None:
            return False
        self.normal, self.offset = result[0], result[1]
        return True

    # -- detekcja --

    def _cloud(self, frames):
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        if self.grid is None:
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            self.grid = pixel_grid(depth_image.shape[1], depth_image.shape[0], intrinsics)
        if self.depth_scale is None:
            raise RuntimeError("depth_scale nie ustawiony - uzyj set_depth_scale()")

        xyz = deproject(depth_image, self.grid, self.depth_scale)
        z = xyz[:, :, 2]
        valid = (
            (z > self.min_distance)
            & (z < self.max_distance)
            & (np.abs(xyz[:, :, 0]) < self.max_side)
        )
        return xyz, valid

    def set_depth_scale(self, depth_scale: float) -> None:
        self.depth_scale = depth_scale

    def detect(self, frames) -> list[dict]:
        """Lista obiektow nad podloga. Klucze: DETECTION_KEYS."""
        if not self.has_floor:
            raise RuntimeError("najpierw fit_floor() albo load_plane()")

        xyz, valid = self._cloud(frames)
        distance = xyz @ self.normal + self.offset

        above = valid & (distance > self.min_height) & (distance < self.max_height)
        mask = above.astype(np.uint8)
        # Zamkniecie sklei dziury w obiekcie, otwarcie zetnie pojedyncze piksele.
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        self.last_mask = mask

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        e1, e2 = plane_basis(self.normal)

        objects = []
        for label in range(1, count):  # 0 to tlo
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area_px:
                continue
            # Grupujemy po masce PO morfologii, ale geometrie liczymy wylacznie
            # z pikseli, ktore naprawde byly nad podloga. Domkniecie zalepia
            # dziury takze pikselami bez glebi (z = 0), a takie deprojektuja sie
            # do (0,0,0) - czyli pozorna odleglosc rowna offsetowi plaszczyzny.
            # Bez tego przeciecia centroid, wysokosc i szerokosc chwytu klamia.
            member = (labels == label) & above
            points = xyz[member]
            if len(points) < 3:
                continue

            # Rzut na plaszczyzne podlogi -> prostokat o najmniejszym polu.
            flat = np.stack((points @ e1, points @ e2), axis=1).astype(np.float32)
            (_, _), (side_a, side_b), angle = cv2.minAreaRect(flat)
            width, length = sorted((float(side_a), float(side_b)))

            objects.append(
                {
                    "centroid": tuple(np.median(points, axis=0).tolist()),
                    "height_m": float(distance[member].max()),
                    "width_m": width,
                    "length_m": length,
                    "angle_deg": float(angle),
                    "area_px": area,
                    "pixels": (int(centroids[label][0]), int(centroids[label][1])),
                }
            )

        objects.sort(key=lambda o: o["centroid"][2])  # najblizszy pierwszy
        return objects


_default_detector: FloorObjectDetector | None = None


def detect_objects(frames, depth_scale: float | None = None) -> list[dict]:
    """
    Wygodne wejscie dla petli pick-and-place.

    Przy pierwszym wywolaniu dopasowuje podloge do biezacej klatki, wiec
    zawolaj je raz na pustej podlodze. Wiecej kontroli daje FloorObjectDetector.
    """
    global _default_detector
    if _default_detector is None:
        _default_detector = FloorObjectDetector()
        if depth_scale is not None:
            _default_detector.set_depth_scale(depth_scale)
        _default_detector.fit_floor(frames)
    return _default_detector.detect(frames)


# --- CLI ---------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Detekcja obiektow na podlodze po glebi")
    p.add_argument("--width", type=int, default=WIDTH)
    p.add_argument("--height", type=int, default=HEIGHT)
    p.add_argument("--fps", type=int, default=FPS)
    p.add_argument("--min-distance", type=float, default=MIN_DISTANCE)
    p.add_argument("--max-distance", type=float, default=MAX_DISTANCE)
    p.add_argument("--max-side", type=float, default=MAX_SIDE)
    p.add_argument("--min-height", type=float, default=MIN_HEIGHT, help="m nad podloga")
    p.add_argument("--max-height", type=float, default=MAX_HEIGHT)
    p.add_argument("--min-area", type=int, default=MIN_AREA_PX)
    p.add_argument("--plane-threshold", type=float, default=PLANE_THRESHOLD)
    p.add_argument("--load-plane", help="wczytaj plaszczyzne z pliku JSON")
    p.add_argument("--save-plane", help="zapisz dopasowana plaszczyzne do JSON")
    p.add_argument("--log", help="zapis wykrytych obiektow do CSV")
    p.add_argument("--no-preview", action="store_true")
    p.add_argument("--seconds", type=float, help="zakoncz po N sekundach")
    return p.parse_args()


def start_pipeline(width: int, height: int, fps: int):
    if len(rs.context().query_devices()) == 0:
        raise SystemExit("Nie widac kamery RealSense. Sprawdz kabel i realsense-viewer.")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        raise SystemExit(
            f"Nie udalo sie wystartowac {width}x{height}@{fps}: {exc}\n"
            "Kamere moze trzymac inny proces - urzadzenie jest na wylacznosc."
        ) from exc
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    return pipeline, rs.align(rs.stream.color), depth_scale


def draw(view, objects, mask, show_mask):
    if show_mask:
        view = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
    for i, obj in enumerate(objects):
        u, v = obj["pixels"]
        x, y, z = obj["centroid"]
        cv2.circle(view, (u, v), 6, (0, 0, 255), -1)
        cv2.putText(
            view,
            f"#{i} {z:.2f}m h={obj['height_m'] * 100:.0f}cm w={obj['width_m'] * 100:.0f}cm",
            (max(0, u - 90), max(20, v - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )
    cv2.putText(
        view,
        f"obiektow: {len(objects)}   f = dopasuj podloge",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )
    return view


def main() -> None:
    args = parse_args()
    preview = not args.no_preview

    pipeline, align, depth_scale = start_pipeline(args.width, args.height, args.fps)
    print(f"Strumien {args.width}x{args.height}@{args.fps}, depth_scale={depth_scale}")

    detector = FloorObjectDetector(
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        max_side=args.max_side,
        plane_threshold=args.plane_threshold,
        min_height=args.min_height,
        max_height=args.max_height,
        min_area_px=args.min_area,
    )
    detector.set_depth_scale(depth_scale)

    log_file = log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            ["t", "obj", "x_m", "y_m", "z_m", "height_m", "width_m", "length_m",
             "angle_deg", "area_px"]
        )

    show_mask = False
    last_print = 0.0
    t0 = time.time()
    frames_seen = 0

    try:
        if args.load_plane:
            detector.load_plane(args.load_plane)
            print(f"Wczytano plaszczyzne z {args.load_plane}")
        else:
            print("Dopasowuje podloge - w kadrze ma byc PUSTA podloga...")
            for _ in range(10):  # kilka klatek na rozgrzanie auto-ekspozycji
                frames = align.process(pipeline.wait_for_frames())
            if not detector.fit_floor(frames):
                raise SystemExit("Nie udalo sie dopasowac plaszczyzny - za malo punktow w ROI.")
        n = detector.normal
        print(
            f"Podloga: normalna ({n[0]:+.3f} {n[1]:+.3f} {n[2]:+.3f}), "
            f"kamera {detector.offset:.3f} m nad nia"
        )
        if args.save_plane:
            detector.save_plane(args.save_plane)
            print(f"Zapisano plaszczyzne do {args.save_plane}")

        while True:
            frames = align.process(pipeline.wait_for_frames())
            if not frames.get_depth_frame() or not frames.get_color_frame():
                continue
            frames_seen += 1

            objects = detector.detect(frames)
            now = time.time()

            if objects and now - last_print >= PRINT_EVERY:
                print(f"--- {len(objects)} obiektow ---")
                for i, obj in enumerate(objects):
                    x, y, z = obj["centroid"]
                    print(
                        f"  #{i} XYZ = {x:+.3f} {y:+.3f} {z:+.3f} m  "
                        f"wys {obj['height_m'] * 100:5.1f} cm  "
                        f"chwyt {obj['width_m'] * 100:5.1f} cm  "
                        f"kat {obj['angle_deg']:+6.1f} st"
                    )
                last_print = now

            if log_writer:
                for i, obj in enumerate(objects):
                    x, y, z = obj["centroid"]
                    log_writer.writerow(
                        [f"{now - t0:.4f}", i, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                         f"{obj['height_m']:.4f}", f"{obj['width_m']:.4f}",
                         f"{obj['length_m']:.4f}", f"{obj['angle_deg']:.1f}",
                         obj["area_px"]]
                    )

            if preview:
                color_image = np.asanyarray(frames.get_color_frame().get_data())
                view = draw(color_image.copy(), objects, detector.last_mask, show_mask)
                cv2.imshow("D415 - obiekty na podlodze", view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("f"):
                    if detector.fit_floor(frames):
                        print("Podloga dopasowana od nowa.")
                if key == ord("m"):
                    show_mask = not show_mask
                if key == ord("s"):
                    cv2.imwrite("floor_snapshot.png", view)
                    print("zapisano floor_snapshot.png")

            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        if preview:
            cv2.destroyAllWindows()
        if log_file:
            log_file.close()
            print(f"Zapisano log: {args.log}")
        elapsed = time.time() - t0
        print(f"Koniec. {frames_seen} klatek w {elapsed:.1f}s ({frames_seen / max(elapsed, 1e-6):.1f} fps)")


if __name__ == "__main__":
    main()
