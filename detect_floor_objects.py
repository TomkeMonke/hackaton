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

# Obszar roboczy - punkty poza nim nie wchodza ani do RANSAC, ani do detekcji.
# 0.31 m to zmierzone minimum glebi D415 w trybie 640x480; blizej sa same zera,
# co jest tez powodem, dla ktorego cel trzeba zapamietac ZANIM sie do niego
# podjedzie - na dystansie chwytania kamera juz go nie widzi.
MIN_DISTANCE = 0.3
MAX_DISTANCE = 2.0  # m
MAX_SIDE = 1.0  # m; |X| od osi optycznej

PLANE_THRESHOLD = 0.01  # m; inlier plaszczyzny = punkt blizej niz 1 cm
RANSAC_ITERS = 300
RANSAC_SAMPLE = 6000  # ile punktow losujemy do szukania plaszczyzny

MIN_HEIGHT = 0.015  # m nad podloga, ponizej to szum i faktura podlogi
MAX_HEIGHT = 0.40  # m; wyzej to juz nie jest obiekt na podlodze
MIN_AREA_PX = 300  # mniejsze plamy odrzucamy

PRINT_EVERY = 0.5

# Bramki wymiarowe celu. Kolor szyszki (braz) pokrywa sie w HSV ze sciolka,
# drewnem i ziemia, wiec progiem koloru sie jej nie wylowi. Za to rozmiar jest
# znakiem szczegolnym: szyszka lezaca na podlodze to kilkucentymetrowy walec,
# a nie noga krzesla ani but.
TARGET_PRESETS = {
    # bez filtrowania wymiarow - wszystko, co wystaje nad podloge
    "any": {},
    # Wartosci ZMIERZONE na sztucznej trawie 2026-09-25, kamera poziomo 10 cm
    # nad ziemia, szyszki 0.5-0.8 m przed obiektywem: wychodzily 3-4 cm dlugosci,
    # 1-2 cm szerokosci i tylko ~2 cm wysokosci nad plaszczyzna.
    #
    # Wysokosc jest mniejsza niz sama szyszka, bo plaszczyzna klada sie na
    # WIERZCHOLKACH zdzbel sztucznej trawy, a szyszka czesciowo w nie wtapia.
    # Pierwsza wersja miala min_height 2 cm i przez to scinala je do paska tak
    # cienkiego, ze otwarcie morfologiczne kasowalo go do zera - detekcja nie
    # zglaszala nawet odrzucenia, bo klaster w ogole nie powstawal.
    # Prog szerokosci jest nisko swiadomie: ten sam obiekt mierzy sie tym wezej,
    # im dalej lezy (mniej pikseli, wiecej rozmycia glebi). Szyszka z 0.5 m
    # wychodzila 1.1 cm, ta sama klasa obiektu z 0.76 m juz 0.6 cm.
    "szyszka": {
        "min_width": 0.005,
        "max_width": 0.06,
        "min_length": 0.02,
        # 10 cm, nie 16. Pierwsza wersja miala 16 cm z ksiazkowych wymiarow
        # szyszki swierkowej i przepuszczala obiekt mierzacy 15.2 x 4.3 cm,
        # lezacy na torze obok. Zmierzone szyszki maja 3-5 cm dlugosci, wiec
        # gorny prog moze byc duzo blizej rzeczywistosci.
        "max_length": 0.10,
        # 0.8 cm nad plaszczyzna. Nizej niz sie wydaje, bo plaszczyzna lezy na
        # wierzcholkach zdzbel, a szyszka z 0.76 m dawala plasterek tak cienki,
        # ze przy progu 1.2 cm znikala. Sprawdzone na tej murawie: przy 0.8 cm
        # sama trawa nie generuje ani jednej falszywej detekcji.
        "min_height": 0.008,
        "max_height": 0.09,
        "min_fill": 0.40,
        "min_area_px": 50,
    },
}

# Progi koloru ZMIERZONE na torze 2026-09-25 (sample_colors.py), przy
# automatycznym balansie bieli o zmierzchu. UWAGA: to nie jest "braz na
# zielonym". W tym swietle murawa rejestruje sie jako sinozielona (H ~90 w skali
# OpenCV, czyli cyjan), a szyszki jako ciemnofioletowe (H ~148). Odcien sam w
# sobie ich nie rozdziela - ogony rozkladow zachodza na siebie. Rozdziela je
# JASNOSC: szyszki V ~94, murawa V ~152.
#
# Prog na jasnosc jest z natury kruchy - slonce albo cien go przesuwa. Dlatego
# tryb "both" trzyma obok niego sprawdzenie geometryczne, a przy zmianie swiatla
# nalezy przepuscic sample_colors.py jeszcze raz i podmienic te liczby.
COLOR_GATE = {
    "v_max": 125,  # ciemniejsze niz murawa (jej 5. percentyl to 127)
    "s_max": 90,
}

DETECTION_KEYS = (
    "centroid",  # (x, y, z) w metrach, uklad kamery
    "distance_m",  # odleglosc od kamery w linii prostej
    "forward_m",  # ile do przodu po ziemi - tyle ma przejechac platforma
    "lateral_m",  # ile w bok; dodatnie = w prawo
    "ground_distance_m",  # odleglosc po ziemi, bez skladowej pionowej
    "bearing_deg",  # kat do celu; dodatni = w prawo
    "height_m",  # wysokosc najwyzszego punktu nad plaszczyzna podlogi
    "width_m",  # krotszy bok prostokata = os chwytania
    "length_m",  # dluzszy bok
    "angle_deg",  # orientacja dluzszej osi w plaszczyznie podlogi
    "fill",  # pole maski / pole prostokata; walek ma duzo, patyk malo
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


def ground_frame(normal: np.ndarray):
    """
    Uklad zwiazany z ziemia: (przod, prawo, gora).

    "Gora" to normalna plaszczyzny. "Przod" to os optyczna kamery (Z) rzutowana
    na plaszczyzne ziemi - czyli kierunek, w ktorym platforma pojedzie. Dzieki
    temu, ze bierze sie to z DOPASOWANEJ plaszczyzny, a nie z zalozenia "kamera
    stoi rowno", lekkie przechylenie uchwytu samo sie kompensuje.

    Gdy kamera patrzy prosto w plaszczyzne (np. w dol z masztu), rzut osi Z
    znika - wtedy za przod bierzemy kolejna os, zeby uklad byl zawsze okreslony.
    """
    up = normal / np.linalg.norm(normal)
    for axis in ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)):
        candidate = np.array(axis) - (np.array(axis) @ up) * up
        norm = np.linalg.norm(candidate)
        if norm > 1e-3:
            forward = candidate / norm
            break
    right = np.cross(forward, up)
    return forward, right / np.linalg.norm(right), up


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
        min_width=None,
        max_width=None,
        min_length=None,
        max_length=None,
        min_fill=None,
        reject_tall=True,
        mode="depth",
        color_gate=None,
        open_kernel=5,
        filters=None,
        seed=0,
    ):
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.max_side = max_side
        self.plane_threshold = plane_threshold
        self.min_height = min_height
        self.max_height = max_height
        self.min_area_px = min_area_px
        # Bramka wymiarowa celu; None = wymiar nie filtruje.
        self.min_width = min_width
        self.max_width = max_width
        self.min_length = min_length
        self.max_length = max_length
        self.min_fill = min_fill
        # Pasmo wysokosci wycina obiekt POZIOMO: noga stolu ma w pasmie 2-9 cm
        # przekroj ~3x3 cm, czyli wymiarami nie do odroznienia od szyszki.
        # Roznica jest taka, ze noga idzie dalej w gore - klaster dotyka gornej
        # krawedzi pasma. Szyszka konczy sie pod nia.
        self.reject_tall = reject_tall
        # depth = co wystaje nad ziemie, color = co jest ciemniejsze od murawy,
        # both = jedno i drugie naraz.
        self.mode = mode
        self.color_gate = color_gate or COLOR_GATE
        self.open_kernel = open_kernel
        # Lista filtrow rs stosowana do ramki glebi przed deprojekcja.
        self.filters = filters or []
        self.rng = np.random.default_rng(seed)
        # Ile klastrow odpadlo i na czym - bez tego strojenie progow to zgadywanie.
        self.rejected = {"area": 0, "width": 0, "length": 0, "fill": 0, "tall": 0}

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
        for rs_filter in self.filters:
            depth_frame = rs_filter.process(depth_frame)
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

    @staticmethod
    def _within(value, low, high) -> bool:
        if low is not None and value < low:
            return False
        if high is not None and value > high:
            return False
        return True

    @classmethod
    def for_target(cls, target: str, **overrides):
        """Detektor z bramka wymiarowa presetu, np. for_target('szyszka')."""
        if target not in TARGET_PRESETS:
            raise ValueError(f"nieznany cel '{target}', dostepne: {sorted(TARGET_PRESETS)}")
        params = dict(TARGET_PRESETS[target])
        params.update({k: v for k, v in overrides.items() if v is not None})
        return cls(**params)

    def detect(self, frames) -> list[dict]:
        """Lista obiektow nad podloga. Klucze: DETECTION_KEYS."""
        if not self.has_floor:
            raise RuntimeError("najpierw fit_floor() albo load_plane()")

        xyz, valid = self._cloud(frames)
        distance = xyz @ self.normal + self.offset
        above = valid & (distance > self.min_height) & (distance < self.max_height)

        if self.mode != "depth":
            dark = self.color_mask(frames)
            if self.mode == "color":
                # Sam kolor, ale nadal tylko w zasiegu skanu - inaczej kazdy
                # ciemny piksel plotu czy drzewa staje sie kandydatem.
                mask = valid & dark
            else:
                mask = above & dark
        else:
            mask = above

        return self.objects_from_cloud(xyz, distance, mask)

    def color_mask(self, frames) -> np.ndarray:
        """Piksele ciemniejsze i mniej nasycone niz murawa - patrz COLOR_GATE."""
        color = np.asanyarray(frames.get_color_frame().get_data())
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        return (hsv[:, :, 2] <= self.color_gate["v_max"]) & (
            hsv[:, :, 1] <= self.color_gate["s_max"]
        )

    def objects_from_cloud(self, xyz, distance, above) -> list[dict]:
        """
        Sama geometria: maska -> klastry -> przefiltrowane obiekty.

        Wydzielone z detect(), zeby dalo sie sprawdzic filtry na wymyslonej
        scenie, bez kamery i bez czekania, az cos odpowiedniego wejdzie w kadr.
        """
        mask = above.astype(np.uint8)
        # Zamkniecie sklei dziury w obiekcie, otwarcie zetnie pojedyncze piksele.
        #
        # Jadro otwarcia 5x5 bywa za duze dla naszych celow (20-50 pikseli) i
        # potrafi zjesc szyszce wierzcholek: ta sama szyszka raportowala 2.2 cm
        # wysokosci przy progu 0.6 cm i tylko 1.0 cm przy 0.8 cm - nie dlatego,
        # ze zmierzono ja inaczej, tylko dlatego, ze z cienszego paska erozja
        # zostawiala sam dol. Jadro 3 (--open-kernel 3) to odzyskuje, kosztem
        # wiekszej liczby drobnych, migoczacych wykryc. Domyslne 5 jest
        # ostrozniejsze i to ono bylo sprawdzone na torze.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((self.open_kernel, self.open_kernel), np.uint8)
        )
        self.last_mask = mask

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        forward, right, _ = ground_frame(self.normal)

        objects = []
        self.rejected = {"area": 0, "width": 0, "length": 0, "fill": 0, "tall": 0}
        for label in range(1, count):  # 0 to tlo
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_area_px:
                self.rejected["area"] += 1
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
            flat = np.stack((points @ right, points @ forward), axis=1).astype(np.float32)
            (_, _), (side_a, side_b), angle = cv2.minAreaRect(flat)
            width, length = sorted((float(side_a), float(side_b)))

            # Bramka wymiarowa - liczona w metrach, wiec dziala tak samo
            # niezaleznie od tego, jak daleko obiekt lezy.
            if not self._within(width, self.min_width, self.max_width):
                self.rejected["width"] += 1
                continue
            if not self._within(length, self.min_length, self.max_length):
                self.rejected["length"] += 1
                continue

            # Wypelnienie prostokata, liczone w PIKSELACH: zwarta bryla jak
            # szyszka wypelnia swoj prostokat w duzej czesci, a rozstrzelony
            # klaster (krawedz dywanu, szum na polysku) prawie wcale. W pikselach,
            # bo wtedy nie trzeba zakladac nic o pochyleniu powierzchni ani o tym,
            # jaki kawalek swiata przypada na piksel z tej odleglosci.
            rows, cols = np.nonzero(member)
            pixel_points = np.stack((cols, rows), axis=1).astype(np.float32)
            (_, _), (rect_w, rect_h), _ = cv2.minAreaRect(pixel_points)
            rect_px = rect_w * rect_h
            fill = min(1.0, len(points) / rect_px) if rect_px > 1e-6 else 0.0
            if self.min_fill is not None and fill < self.min_fill:
                self.rejected["fill"] += 1
                continue

            # Klaster siegajacy gornej krawedzi pasma to kawalek czegos, co idzie
            # dalej w gore - noga, sciana, but. Szyszka miesci sie w pasmie cala.
            top = float(distance[member].max())
            if self.reject_tall and top >= self.max_height - 0.005:
                self.rejected["tall"] += 1
                continue

            centroid = np.median(points, axis=0)
            # To, czego potrzebuje dojazd: ile do przodu, ile w bok, pod jakim
            # katem. Liczone w ukladzie ziemi, wiec niezalezne od tego, czy
            # uchwyt kamery jest idealnie wypoziomowany.
            ahead = float(centroid @ forward)
            side = float(centroid @ right)
            objects.append(
                {
                    "centroid": tuple(centroid.tolist()),
                    "distance_m": float(np.linalg.norm(centroid)),
                    "forward_m": ahead,
                    "lateral_m": side,
                    "ground_distance_m": float(np.hypot(ahead, side)),
                    "bearing_deg": float(np.degrees(np.arctan2(side, ahead))),
                    "height_m": top,
                    "width_m": width,
                    "length_m": length,
                    "angle_deg": float(angle),
                    "fill": float(fill),
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
    p.add_argument("--min-area", type=int, default=None)
    p.add_argument(
        "--target",
        choices=sorted(TARGET_PRESETS),
        default="szyszka",
        help="bramka wymiarowa celu; 'any' = wszystko nad podloga",
    )
    p.add_argument("--min-width", type=float, help="m, nadpisuje preset")
    p.add_argument("--max-width", type=float)
    p.add_argument("--min-length", type=float)
    p.add_argument("--max-length", type=float)
    p.add_argument("--min-fill", type=float, help="0..1, zwartosc bryly")
    p.add_argument(
        "--mode",
        choices=["depth", "color", "both"],
        default="depth",
        help="co ma znajdowac obiekty: geometria, kolor, albo jedno i drugie",
    )
    p.add_argument("--v-max", type=int, help="prog jasnosci dla trybow color/both")
    p.add_argument(
        "--laser-power",
        type=float,
        help="moc projektora IR 0-360; podniesienie pomaga na jednolitej murawie",
    )
    p.add_argument(
        "--filters",
        action="store_true",
        help="filtry glebi (przestrzenny + czasowy); czasowy pomaga NA POSTOJU, "
        "w ruchu rozmazuje",
    )
    p.add_argument(
        "--open-kernel",
        type=int,
        default=5,
        help="bok jadra otwarcia; 3 odzyskuje scinane wierzcholki malych obiektow",
    )
    p.add_argument(
        "--keep-tall",
        action="store_true",
        help="nie odrzucaj obiektow siegajacych gornej krawedzi pasma wysokosci",
    )
    p.add_argument("--plane-threshold", type=float, default=PLANE_THRESHOLD)
    p.add_argument("--load-plane", help="wczytaj plaszczyzne z pliku JSON")
    p.add_argument("--save-plane", help="zapisz dopasowana plaszczyzne do JSON")
    p.add_argument("--log", help="zapis wykrytych obiektow do CSV")
    p.add_argument(
        "--snapshot",
        help="zapisz jedna klatke z opisanymi detekcjami (dziala tez bez podgladu)",
    )
    p.add_argument("--no-preview", action="store_true")
    p.add_argument("--seconds", type=float, help="zakoncz po N sekundach")
    return p.parse_args()


def make_filters():
    """
    Filtry doczyszczajace glebie. Wlaczane jawnie, nie domyslnie.

    Temporalny jest tu najmocniejszy, bo skan robimy NA POSTOJU: kolejne klatki
    pokazuja te sama scene, wiec filtr uzupelnia dziury pomiarami z sasiednich
    klatek. Przy platformie w ruchu jest odwrotnie - rozmazuje, bo usrednia
    rozne sceny.

    Przestrzenny wygladza z zachowaniem krawedzi.

    CELOWO NIE MA hole_filling_filter. On nie uzupelnia dziur pomiarami, tylko
    ZMYSLA glebie z sasiadow - a ten pipeline decyduje "obiekt czy nie obiekt"
    dokladnie na podstawie tego, czy cos wystaje nad plaszczyzne. Zmyslone
    piksele produkowalyby zmyslone szyszki.

    ZMIERZONE 2026-09-25, zanim ktos wlaczy to domyslnie:

    Pokrycie glebia rosnie i to jest pewne - mierzone PARAMI na tych samych
    klatkach (zeby dryf sceny sie zniosl): 22.30% -> 24.44% w pasie 0.3-1.0 m,
    poprawa w 30 klatkach na 30.

    Liczba wykrytych szyszek na tym nie zyskuje, a czasem traci. Ta sama scena,
    po cztery skany: BEZ filtrow 5 szyszek w 4 przebiegach na 4; Z filtrami
    spadek do 4 w 2 przebiegach na 4 - za kazdym razem gubiona byla najblizsza
    (0.372 m). Efekt jest PRZERYWANY, nie deterministyczny, ale idzie tylko w
    jedna strone i nigdy nie zaobserwowano, zeby filtry dolozyly wykrycie.

    Stad domyslnie wylaczone: wygladzanie zjada male obiekty, tak samo jak za
    duze jadro otwarcia. Wiecej pikseli glebi nie znaczy wiecej szyszek, a to
    drugie jest tym, po co tu jestesmy.
    """
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)

    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)

    return [spatial, temporal]


def start_pipeline(width: int, height: int, fps: int, laser_power: float | None = None):
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
    sensor = profile.get_device().first_depth_sensor()
    if laser_power is not None and sensor.supports(rs.option.laser_power):
        # Projektor IR dokłada teksture na jednolite powierzchnie. Sztuczna
        # trawa o zmierzchu prawie nie ma kontrastu wlasnego i stereo gubi
        # punkty zaczepienia. Zmierzone: moc 150 (domyslna) dawala 31.0%
        # pokrycia glebia na murawie, moc 360 dala 35.8%. Martwego pasa przy
        # lewej krawedzi to nie rusza - tam brakuje nie swiatla, tylko drugiego
        # punktu widzenia.
        rng = sensor.get_option_range(rs.option.laser_power)
        value = max(rng.min, min(rng.max, laser_power))
        sensor.set_option(rs.option.laser_power, value)
        print(f"Moc projektora IR: {value:.0f} (zakres {rng.min:.0f}-{rng.max:.0f})")
    depth_scale = sensor.get_depth_scale()
    return pipeline, rs.align(rs.stream.color), depth_scale


def draw(view, objects, mask, show_mask):
    if show_mask:
        view = cv2.cvtColor(mask * 255, cv2.COLOR_GRAY2BGR)
    for i, obj in enumerate(objects):
        u, v = obj["pixels"]
        x, y, z = obj["centroid"]
        cv2.circle(view, (u, v), 6, (0, 0, 255), -1)
        label = (
            f"#{i} {obj['distance_m']:.2f}m "
            f"{obj['length_m'] * 100:.0f}x{obj['width_m'] * 100:.0f}cm "
            f"h={obj['height_m'] * 100:.0f}cm"
        )
        # Dosuniecie napisu do kadru - przy obiekcie na krawedzi tekst inaczej
        # wychodzi poza obraz i urywa sie dokladnie na szerokosci chwytu.
        (text_w, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        text_x = min(max(0, u - text_w // 2), max(0, view.shape[1] - text_w))
        cv2.putText(
            view,
            label,
            (text_x, max(20, v - 12)),
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

    pipeline, align, depth_scale = start_pipeline(
        args.width, args.height, args.fps, args.laser_power
    )
    print(f"Strumien {args.width}x{args.height}@{args.fps}, depth_scale={depth_scale}")

    # Preset celu daje wartosci domyslne, jawne flagi je nadpisuja.
    params = dict(TARGET_PRESETS[args.target])
    params.setdefault("min_height", args.min_height)
    params.setdefault("max_height", args.max_height)
    params.setdefault("min_area_px", MIN_AREA_PX)
    for name, value in (
        ("min_width", args.min_width),
        ("max_width", args.max_width),
        ("min_length", args.min_length),
        ("max_length", args.max_length),
        ("min_fill", args.min_fill),
        ("min_area_px", args.min_area),
    ):
        if value is not None:
            params[name] = value

    detector = FloorObjectDetector(
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        max_side=args.max_side,
        plane_threshold=args.plane_threshold,
        reject_tall=not args.keep_tall,
        mode=args.mode,
        color_gate=(
            {**COLOR_GATE, "v_max": args.v_max} if args.v_max is not None else None
        ),
        open_kernel=args.open_kernel,
        filters=make_filters() if args.filters else None,
        **params,
    )
    detector.set_depth_scale(depth_scale)
    if args.target != "any":
        print(
            f"Cel '{args.target}': szerokosc {params.get('min_width')}-"
            f"{params.get('max_width')} m, dlugosc {params.get('min_length')}-"
            f"{params.get('max_length')} m, wysokosc {params.get('min_height')}-"
            f"{params.get('max_height')} m, wypelnienie >= {params.get('min_fill')}"
        )

    log_file = log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(
            ["t", "obj", "distance_m", "x_m", "y_m", "z_m", "height_m", "width_m",
             "length_m", "angle_deg", "fill", "area_px"]
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
            try:
                frames = align.process(pipeline.wait_for_frames())
            except RuntimeError as exc:
                # Wyciagniete kabla nie jest bledem programu - powiedz to wprost
                # i wyjdz. Bez tego przechwycenia wyjatek leci przez `finally`,
                # tam pipeline.stop() rzuca drugim wyjatkiem i uzytkownik dostaje
                # traceback konczacy sie na "stop() cannot be called before
                # start()", czyli komunikat nie majacy nic wspolnego z przyczyna.
                print(f"\nKamera przestala odpowiadac: {exc}")
                print("Sprawdz kabel USB i czy urzadzenia nie przejal inny proces.")
                break
            if not frames.get_depth_frame() or not frames.get_color_frame():
                continue
            frames_seen += 1

            objects = detector.detect(frames)
            now = time.time()

            if now - last_print >= PRINT_EVERY:
                rej = detector.rejected
                print(
                    f"--- {len(objects)} obiektow "
                    f"(odrzucone: pole {rej['area']}, szer {rej['width']}, "
                    f"dl {rej['length']}, wyp {rej['fill']}, wystaje {rej['tall']}) ---"
                )
                for i, obj in enumerate(objects):
                    x, y, z = obj["centroid"]
                    print(
                        f"  #{i} dystans {obj['distance_m']:.3f} m  "
                        f"XYZ = {x:+.3f} {y:+.3f} {z:+.3f}  "
                        f"{obj['length_m'] * 100:4.1f}x{obj['width_m'] * 100:4.1f} cm  "
                        f"wys {obj['height_m'] * 100:4.1f} cm  "
                        f"wyp {obj['fill']:.2f}  kat {obj['angle_deg']:+6.1f} st"
                    )
                last_print = now

            if log_writer:
                for i, obj in enumerate(objects):
                    x, y, z = obj["centroid"]
                    log_writer.writerow(
                        [f"{now - t0:.4f}", i, f"{obj['distance_m']:.4f}",
                         f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                         f"{obj['height_m']:.4f}", f"{obj['width_m']:.4f}",
                         f"{obj['length_m']:.4f}", f"{obj['angle_deg']:.1f}",
                         f"{obj['fill']:.3f}", obj["area_px"]]
                    )

            if args.snapshot and frames_seen > 5:
                color_image = np.asanyarray(frames.get_color_frame().get_data())
                cv2.imwrite(
                    args.snapshot,
                    draw(color_image.copy(), objects, detector.last_mask, False),
                )
                print(f"zapisano {args.snapshot} ({len(objects)} detekcji)")
                args.snapshot = None  # tylko pierwsza pasujaca klatka

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
        # Po odpieciu kamery pipeline jest juz zatrzymany i stop() rzuca
        # wyjatkiem, ktory przykrylby prawdziwa przyczyne.
        try:
            pipeline.stop()
        except RuntimeError:
            pass
        if preview:
            cv2.destroyAllWindows()
        if log_file:
            log_file.close()
            print(f"Zapisano log: {args.log}")
        elapsed = time.time() - t0
        print(f"Koniec. {frames_seen} klatek w {elapsed:.1f}s ({frames_seen / max(elapsed, 1e-6):.1f} fps)")


if __name__ == "__main__":
    main()
