"""
Chwyt szyszki z ZAPAMIETANEJ pozycji: cel.json -> katy przegubow SO-101.

D415 przestaje mierzyc ponizej ~0.27 m, wiec na dystansie chwytania szyszki
juz nie widac. Pozycje zdejmuje sie raz, z dystansu (scan_cones.py --json
cel.json), a dojazd i chwyt musza pojsc z pamieci - bez zadnego dalszego
pomiaru. Ten skrypt robi te druga czesc: przelicza cel do ukladu bazy ramienia,
liczy IK i wypisuje sekwencje chwytu krok po kroku.

Domyslnie NIC nie rusza. --dry-run jest wlaczony z automatu i skrypt tylko
wypisuje katy. Ruch wymaga jawnego --port; bez portu lerobot nie jest nawet
importowany.

Czego tu NIE MA i o czym trzeba pamietac czytajac wypis:
  * transformata kamera -> baza ramienia jest OSZACOWANA z montazu, nie
    zmierzona (arm_camera_transform.json, klucz "zmierzone": false),
  * przelozenie katow URDF na jednostki lerobota jest zalozone 1:1,
  * maksymalne rozwarcie chwytaka jest zalozone, nie zmierzone.
Kazde z tych trzech miejsc jest wypisywane jako ZALOZENIE przy kazdym planie.

Uzycie:
    python approach_and_grasp.py                        # plan z cel.json, nic nie wysyla
    python approach_and_grasp.py --target cel.json --index 1
    python approach_and_grasp.py --no-drive             # chwyt z celu bez dojazdu platformy
    python approach_and_grasp.py --standoff 0.15        # gdzie ma stanac szyszka po dojazdzie
    python approach_and_grasp.py --port COM10           # DOPIERO TO rusza ramieniem
"""

from __future__ import annotations

import argparse
import json
import math
import os
import warnings

import numpy as np

# ---------------------------------------------------------------- stale

REPO_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(REPO_DIR, "so101_urdf", "so101_new_calib.urdf")
TRANSFORM_PATH = os.path.join(REPO_DIR, "arm_camera_transform.json")
TARGET_PATH = os.path.join(REPO_DIR, "cel.json")

# Przeguby w kolejnosci lancucha, zgodnie z arm_control.py i URDF.
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_JOINT = "gripper"

# Ostatni link URDF w lancuchu IK. gripper_frame_link to punkt miedzy szczekami,
# a nie obudowa serwa - i tym punktem celujemy w szyszke.
CHAIN_BASE_LINK = "base_link"

# Poza wyjsciowa. Te same wartosci co HOME_POSE w arm_control.py, zeby nie
# miec dwoch roznych "pozycji spoczynkowej". UWAGA: tam to tez placeholder do
# poprawienia po kalibracji - w zerach ramie stoi wyprostowane poziomo (chwytak
# 0.39 m przed baza), co nie jest szczegolnie bezpieczna poza transportowa.
START_POSE_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}
START_GRIPPER_CMD = 50.0

# Geometria chwytu
GRASP_HEIGHT_FRACTION = 0.5  # na jakiej wysokosci szyszki lapiemy (0.5 = w polowie)
MIN_GRASP_HEIGHT_M = 0.008  # nizej nie schodzimy - szczeka zaczepi o ziemie
APPROACH_CLEARANCE_M = 0.06  # ile nad punktem chwytu zawisamy przed opuszczeniem
LIFT_M = 0.08  # ile podnosimy po zacisnieciu
# Gdzie przed kamera ma wyladowac szyszka po dojazdzie. TA SAMA LICZBA co
# STANDOFF_M w drive_to_target.py - zapas liczony od kamery. Jak ktos zmieni tam,
# musi zmienic i tu, inaczej plan chwytu opisuje inne miejsce niz to, gdzie
# platforma stanie.
DEFAULT_STANDOFF_M = 0.12

# Chwytak. Model liniowy rozwarcie -> komenda 0..100; szczeka chodzi po luku,
# wiec to uproszczenie. Maksimum czytane z konfiguracji, tu tylko awaryjne.
GRIPPER_MAX_OPEN_M = 0.05  # zmierzone 2026-09-25: szczeki rozwieraja sie luzno na 5 cm
GRIPPER_OPEN_MARGIN_M = 0.015  # o tyle szerzej niz szyszka otwieramy szczeki
GRIPPER_MIN_MARGIN_M = 0.004  # mniejszego zapasu nie uznajemy za chwyt
GRIPPER_SQUEEZE_M = 0.004  # o tyle "pod wymiar" zaciskamy, zeby docisnac

# Podejscie: 90 st = pionowo z gory. Jesli pionowo nie wychodzi, probujemy
# przechylonych - z mniejszym katem ramie siega dalej.
APPROACH_PITCH_CANDIDATES_DEG = (90.0, 75.0, 60.0, 45.0)

# Progi akceptacji rozwiazania IK
IK_POSITION_TOLERANCE_M = 0.008
IK_AXIS_TOLERANCE_DEG = 8.0
JAW_YAW_TOLERANCE_DEG = 5.0
ROLL_SOLVE_ITERATIONS = 4

# Przelozenie kata z wizji na obrot chwytaka.
# detect_floor_objects.py liczy minAreaRect w ukladzie (w prawo, w przod), czyli
# kierunek o kacie A ma skladowe (lateral, forward) = (cos A, sin A). W ukladzie
# ziemi uzywanym tutaj (X w przod, Y w lewo) daje to yaw = atan2(-cos A, sin A)
# = A - 90. Szczeki zaciskaja sie na KROTSZEJ osi, czyli linia miedzy szczekami
# jest prostopadla do dluzszej osi: +90.
LONG_AXIS_YAW_FROM_DETECTOR_DEG = -90.0
JAW_YAW_FROM_LONG_AXIS_DEG = 90.0

# Domyslna transformata, gdy nie ma pliku konfiguracyjnego. Te same liczby co w
# arm_camera_transform.json - OSZACOWANIE Z MONTAZU, NIE POMIAR.
DEFAULT_TRANSFORM = {
    "zmierzone": False,
    "translacja_m": {"x": 0.08, "y": 0.0, "z": -0.06},
    "rotacja_deg": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "przeguby_lerobot": {name: {"znak": 1.0, "offset_deg": 0.0} for name in JOINT_NAMES},
    "chwytak": {"max_rozwarcie_m": GRIPPER_MAX_OPEN_M},
}

MOUNT_TILT_WARN_DEG = 5.0  # powyzej tego przestaje wystarczac uproszczony yaw


class PlanError(Exception):
    """Plan nie do wykonania. Lepiej stanac niz chwytac w powietrze."""


# ---------------------------------------------------------------- pomocnicze


def wrap180(deg: float) -> float:
    """Kat do przedzialu (-180, 180]."""
    return (float(deg) + 180.0) % 360.0 - 180.0


def wrap90(deg: float) -> float:
    """Kat do (-90, 90]. Linia szczek jest symetryczna co 180 st."""
    return (float(deg) + 90.0) % 180.0 - 90.0


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), stopnie."""
    r, p, y = (math.radians(v) for v in (roll_deg, pitch_deg, yaw_deg))
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


# ---------------------------------------------------------------- transformata


class Transform:
    """Kamera (rzut na ziemie) -> baza ramienia. Wczytana z pliku, nie zgadywana w kodzie."""

    def __init__(self, data: dict, source: str):
        self.source = source
        self.measured = bool(data.get("zmierzone", False))
        t = data.get("translacja_m", {})
        self.translation = np.array(
            [float(t.get("x", 0.0)), float(t.get("y", 0.0)), float(t.get("z", 0.0))]
        )
        r = data.get("rotacja_deg", {})
        self.roll = float(r.get("roll", 0.0))
        self.pitch = float(r.get("pitch", 0.0))
        self.yaw = float(r.get("yaw", 0.0))
        self.rotation = rotation_matrix(self.roll, self.pitch, self.yaw)

        joints = data.get("przeguby_lerobot", {})
        self.joint_sign = {}
        self.joint_offset = {}
        for name in JOINT_NAMES:
            entry = joints.get(name, {}) if isinstance(joints, dict) else {}
            if not isinstance(entry, dict):
                entry = {}
            self.joint_sign[name] = float(entry.get("znak", 1.0))
            self.joint_offset[name] = float(entry.get("offset_deg", 0.0))

        gripper = data.get("chwytak", {})
        if not isinstance(gripper, dict):
            gripper = {}
        self.gripper_max_open_m = float(gripper.get("max_rozwarcie_m", GRIPPER_MAX_OPEN_M))

    def to_base(self, point_ground: np.ndarray) -> np.ndarray:
        return self.rotation @ np.asarray(point_ground, dtype=float) + self.translation

    def yaw_to_base(self, yaw_deg: float) -> float:
        """Obrot w plaszczyznie ziemi -> ten sam obrot w bazie.

        Wazne tylko dla poziomego montazu; przy niezerowym roll/pitch rzut kata
        na plaszczyzne bazy nie jest zwyklym dodaniem yaw, wiec wtedy ostrzegamy.
        """
        return wrap180(yaw_deg + self.yaw)

    def mount_is_level(self) -> bool:
        return abs(self.roll) <= MOUNT_TILT_WARN_DEG and abs(self.pitch) <= MOUNT_TILT_WARN_DEG

    def describe(self) -> list[str]:
        lines = [
            f"Transformata kamera -> baza: {self.source}",
            f"  translacja [m] x {self.translation[0]:+.3f}  y {self.translation[1]:+.3f}  "
            f"z {self.translation[2]:+.3f}",
            f"  rotacja [st] roll {self.roll:+.1f}  pitch {self.pitch:+.1f}  yaw {self.yaw:+.1f}",
        ]
        if not self.measured:
            lines.append(
                "  ZALOZENIE: te liczby to OSZACOWANIE Z MONTAZU, NIE POMIAR "
                "(zmierzone=false). Kalibracja kamera-ramie to osobny task; do tego"
            )
            lines.append(
                "  czasu bledy rzedu centymetrow sa normalne i chwyt trzeba "
                "sprawdzac na oko."
            )
        if not self.mount_is_level():
            lines.append(
                f"  UWAGA: roll/pitch wieksze niz {MOUNT_TILT_WARN_DEG:.0f} st - obrot "
                "chwytaka liczony jest tu uproszczonym yaw i bedzie przekrzywiony."
            )
        return lines


def load_transform(path: str | None) -> Transform:
    if path is None:
        return Transform(DEFAULT_TRANSFORM, "domyslna z kodu (brak pliku)")
    if not os.path.exists(path):
        print(
            f"UWAGA: brak pliku transformaty {path} - biore domyslne oszacowanie z kodu."
        )
        return Transform(DEFAULT_TRANSFORM, f"domyslna z kodu (nie ma {os.path.basename(path)})")
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    return Transform(data, path)


# ---------------------------------------------------------------- cel z wizji


def load_target(path: str, index: int) -> tuple[dict, dict]:
    """Zwraca (cel, naglowek_pliku). Blad, gdy pliku nie ma albo cel pusty."""
    if not os.path.exists(path):
        raise PlanError(
            f"nie ma pliku celu {path}. Najpierw skan: python scan_cones.py --json cel.json"
        )
    with open(path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    targets = data.get("targets") or []
    if not targets:
        raise PlanError(f"plik {path} nie ma zadnego celu - skan nic nie znalazl")
    if index < 0 or index >= len(targets):
        raise PlanError(
            f"nie ma celu o indeksie {index}; plik {path} ma {len(targets)} "
            f"(indeksy 0..{len(targets) - 1})"
        )
    return targets[index], data


def target_required_keys(target: dict) -> None:
    missing = [k for k in ("forward_m", "lateral_m", "width_m") if k not in target]
    if missing:
        raise PlanError(f"cel nie ma wymaganych pol: {', '.join(missing)}")


# ---------------------------------------------------------------- dojazd


def plan_drive(target: dict, standoff_m: float, header: dict) -> dict:
    """Ile obrocic i ile przejechac, zeby szyszka wyladowala w punkcie chwytu.

    To tylko liczby kontrolne. Dojazd wykonuje drive_to_target.py i on jest tu
    wlascicielem tematu - geometria jest trzymana celowo taka sama jak tam:
    obrot wokol srodka osi, zapas liczony OD KAMERY, czyli od punktu, z ktorego
    powstal pomiar.
    """
    offset = float(header.get("camera_forward_offset_m", 0.0) or 0.0)
    forward = float(target["forward_m"])
    lateral = float(target["lateral_m"])
    ground = float(target.get("ground_distance_m", math.hypot(forward, lateral)))
    bearing_file = target.get("bearing_deg")

    if offset:
        # Kamera wystaje przed srodek obrotu, wiec kat i dystans przeliczamy do
        # ukladu srodka obrotu - inaczej po obrocie cel nie lezy na osi jazdy.
        forward_c = forward + offset
        distance_c = math.hypot(forward_c, lateral)
        turn = math.degrees(math.atan2(lateral, forward_c))
    else:
        distance_c = ground
        turn = (
            float(bearing_file)
            if bearing_file is not None
            else math.degrees(math.atan2(lateral, forward))
        )

    notes = []
    travel = distance_c - offset - float(standoff_m)
    if travel < 0:
        notes.append(
            f"szyszka jest blizej ({distance_c - offset:.3f} m) niz zapas "
            f"({standoff_m:.3f} m) - to jazda DO TYLU o {abs(travel):.3f} m; "
            "drive_to_target.py obcina to do zera i robi sama korekte kata"
        )
    if offset:
        notes.append(
            f"kamera wystaje {offset:.3f} m przed srodek obrotu - kat {turn:+.1f} st "
            f"jest liczony od srodka obrotu, nie od namiaru z pliku "
            f"({float(bearing_file or 0.0):+.1f} st)"
        )

    return {
        "obrot_deg": turn,
        "przod_m": travel,
        "po_dojazdzie": {"forward_m": float(standoff_m), "lateral_m": 0.0},
        "uwagi": notes,
    }


# ---------------------------------------------------------------- model ramienia


class ArmModel:
    """Lancuch IK z URDF. Calosc liczona w radianach i metrach, uklad base_link."""

    def __init__(self, urdf_path: str = URDF_PATH):
        if not os.path.exists(urdf_path):
            raise PlanError(f"nie ma URDF {urdf_path}")
        try:
            from ikpy.chain import Chain
        except ImportError as exc:  # pragma: no cover - zalezy od srodowiska
            raise PlanError(
                "brak biblioteki ikpy, a na niej stoi cale IK. Instalacja: "
                "python -m pip install ikpy"
            ) from exc

        # ikpy krzyczy o 'axis' w fixed joincie i o aktywnym linku fixed - oba
        # nieszkodliwe i oba siedza w URDF, ktorego tu nie ruszamy.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self.chain = Chain.from_urdf_file(
                urdf_path,
                base_elements=[CHAIN_BASE_LINK],
                active_links_mask=self._mask(urdf_path),
            )
        self.link_names = [link.name for link in self.chain.links]
        self.joint_index = {name: self.link_names.index(name) for name in JOINT_NAMES}
        self.bounds = {
            name: self.chain.links[self.joint_index[name]].bounds for name in JOINT_NAMES
        }
        # Wierzcholek lancucha (gripper_frame_link) wzgledem osi shoulder_pan.
        self.pan_origin = np.asarray(
            self.chain.links[self.joint_index["shoulder_pan"]].origin_translation, dtype=float
        )
        self.reach_bound_m = self._reach_bound()

    @staticmethod
    def _mask(urdf_path: str) -> list[bool]:
        """Maska aktywnych linkow: wrist_roll zostaje ZAMROZONY.

        wrist_roll ustawiamy sami (z kata szyszki), bo IK na pozycji + osi
        podejscia i tak nie ma z czego go wyznaczyc - obrot wokol osi podejscia
        jest wolny i solver wybralby cokolwiek.
        """
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            from ikpy.chain import Chain

            probe = Chain.from_urdf_file(urdf_path, base_elements=[CHAIN_BASE_LINK])
        names = [link.name for link in probe.links]
        return [name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex") for name in names]

    def _reach_bound(self) -> float:
        """GORNA granica zasiegu z URDF: suma dlugosci czlonow od shoulder_lift.

        To granica teoretyczna (ramie wyprostowane w linie), nie realna obwiednia.
        Realna jest mniejsza i zalezy od wysokosci celu oraz od kata podejscia -
        tego nie zgadujemy, sprawdza to residuum IK dla konkretnego punktu.
        """
        start = self.joint_index["shoulder_lift"]
        total = 0.0
        for link in self.chain.links[start:]:
            translation = getattr(link, "origin_translation", None)
            if translation is not None:
                total += float(np.linalg.norm(np.asarray(translation, dtype=float)))
        return total

    def zero_vector(self) -> np.ndarray:
        return np.zeros(len(self.chain.links))

    def to_chain_vector(self, angles_rad: dict) -> np.ndarray:
        vector = self.zero_vector()
        for name, value in angles_rad.items():
            vector[self.joint_index[name]] = value
        return vector

    def from_chain_vector(self, vector) -> dict:
        return {name: float(vector[self.joint_index[name]]) for name in JOINT_NAMES}

    def fk(self, angles_rad: dict) -> np.ndarray:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            return self.chain.forward_kinematics(self.to_chain_vector(angles_rad))

    @staticmethod
    def jaw_yaw_deg(frame: np.ndarray) -> float:
        """Kierunek linii miedzy szczekami, rzutowany na plaszczyzne ziemi.

        Os X narzedzia lezy w plaszczyznie rozwierania szczek (zawias szczeki to
        os Y gripper_link), wiec jej rzut to linia chwytu.
        """
        axis = frame[:3, 0]
        return math.degrees(math.atan2(axis[1], axis[0]))

    @staticmethod
    def approach_axis_deg_error(frame: np.ndarray, axis: np.ndarray) -> float:
        tool_z = frame[:3, 2]
        wanted = np.asarray(axis, dtype=float)
        wanted = wanted / np.linalg.norm(wanted)
        dot = float(np.clip(tool_z @ wanted, -1.0, 1.0))
        return math.degrees(math.acos(dot))

    def check_reach_bound(self, point_base: np.ndarray, label: str, reach_limit_m: float) -> None:
        distance = float(np.linalg.norm(np.asarray(point_base, dtype=float) - self.pan_origin))
        if distance > reach_limit_m:
            raise PlanError(
                f"{label}: cel POZA ZASIEGIEM ramienia. Punkt lezy {distance * 100:.1f} cm "
                f"od osi shoulder_pan, a gorna granica z URDF to {reach_limit_m * 100:.1f} cm "
                "(ramie wyprostowane w linie, wiec realnie jest jeszcze mniej). "
                "Podjedz platforma blizej - tego nie da sie nadrobic samym ramieniem."
            )

    def solve(
        self,
        point_base: np.ndarray,
        approach_axis: np.ndarray,
        jaw_yaw_deg: float | None,
        seed: dict | None = None,
    ) -> dict:
        """IK na punkt + os podejscia, z wrist_roll dobranym pod kat szyszki.

        Zwraca dict z katami [rad], bledem pozycji, bledem osi i osiagnietym
        obrotem szczek. NIE rzuca bledem - ocena nalezy do wyzej.
        """
        seed_angles = dict(seed) if seed else {name: 0.0 for name in JOINT_NAMES}
        seed_angles.setdefault("wrist_roll", 0.0)
        roll = seed_angles["wrist_roll"]
        solution = None

        for _ in range(ROLL_SOLVE_ITERATIONS):
            solution = self._ik_once(point_base, approach_axis, roll, seed_angles)
            frame = self.fk(solution)
            if jaw_yaw_deg is None:
                break
            achieved = self.jaw_yaw_deg(frame)
            error = wrap90(jaw_yaw_deg - achieved)
            if abs(error) <= JAW_YAW_TOLERANCE_DEG:
                break
            # Nachylenie yaw(wrist_roll) mierzymy z FK, zeby nie zakladac znaku.
            probe = dict(solution)
            probe["wrist_roll"] = roll + math.radians(10.0)
            slope = wrap180(self.jaw_yaw_deg(self.fk(probe)) - achieved) / 10.0
            if abs(slope) < 1e-3:
                break
            roll = self._clamp("wrist_roll", roll + math.radians(error / slope))
            seed_angles = dict(solution)
            seed_angles["wrist_roll"] = roll

        frame = self.fk(solution)
        achieved_yaw = self.jaw_yaw_deg(frame)
        return {
            "angles_rad": solution,
            "frame": frame,
            "position_error_m": float(np.linalg.norm(frame[:3, 3] - np.asarray(point_base))),
            "axis_error_deg": self.approach_axis_deg_error(frame, approach_axis),
            "jaw_yaw_deg": achieved_yaw,
            "jaw_yaw_error_deg": (
                None if jaw_yaw_deg is None else wrap90(jaw_yaw_deg - achieved_yaw)
            ),
        }

    def _ik_once(
        self, point_base: np.ndarray, approach_axis: np.ndarray, roll: float, seed: dict
    ) -> dict:
        initial = self.to_chain_vector({**seed, "wrist_roll": roll})
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            raw = self.chain.inverse_kinematics(
                target_position=np.asarray(point_base, dtype=float),
                target_orientation=np.asarray(approach_axis, dtype=float),
                orientation_mode="Z",
                initial_position=initial,
            )
        angles = self.from_chain_vector(raw)
        angles["wrist_roll"] = roll  # zamrozony, ale ikpy zwraca go i tak
        return angles

    def _clamp(self, name: str, value: float) -> float:
        low, high = self.bounds[name]
        if low is not None and value < low:
            # Linia szczek jest symetryczna co 180 st - moze przeciwna strona wejdzie.
            flipped = value + math.pi
            if high is None or flipped <= high:
                return flipped
            return low
        if high is not None and value > high:
            flipped = value - math.pi
            if low is None or flipped >= low:
                return flipped
            return high
        return value

    def limit_violations(self, angles_rad: dict) -> list[str]:
        out = []
        for name, value in angles_rad.items():
            low, high = self.bounds[name]
            if low is not None and value < low - 1e-6:
                out.append(f"{name} {math.degrees(value):.1f} st < limit {math.degrees(low):.1f} st")
            if high is not None and value > high + 1e-6:
                out.append(f"{name} {math.degrees(value):.1f} st > limit {math.degrees(high):.1f} st")
        return out


# ---------------------------------------------------------------- chwytak


def gripper_command(opening_m: float, max_open_m: float) -> float:
    """Rozwarcie [m] -> komenda lerobota 0..100 (0 = zamkniety).

    Model liniowy. Szczeka SO-101 chodzi po luku, wiec prawdziwa zaleznosc jest
    sinusoidalna - do poprawienia po jednym pomiarze rozwarcia suwmiarka.
    """
    if max_open_m <= 0:
        raise PlanError("max_rozwarcie_m w konfiguracji chwytaka musi byc dodatnie")
    return float(np.clip(opening_m / max_open_m, 0.0, 1.0) * 100.0)


def plan_gripper(width_m: float, max_open_m: float) -> dict:
    """Ile otworzyc przed chwytem i ile zacisnac. Za szeroka szyszka = blad."""
    if width_m <= 0:
        raise PlanError(f"szerokosc chwytu musi byc dodatnia, a jest {width_m}")
    if width_m + GRIPPER_MIN_MARGIN_M > max_open_m:
        raise PlanError(
            f"szyszka jest ZA SZEROKA dla chwytaka: os chwytania {width_m * 100:.1f} cm, "
            f"a szczeki otwieraja sie na {max_open_m * 100:.1f} cm (z zapasem minimum "
            f"{GRIPPER_MIN_MARGIN_M * 100:.1f} cm). Nie probuj - albo inny cel, albo "
            "zmierz i popraw max_rozwarcie_m w konfiguracji."
        )

    notes = []
    open_m = width_m + GRIPPER_OPEN_MARGIN_M
    if open_m > max_open_m:
        open_m = max_open_m
        notes.append(
            f"zapas przyciety: szczeki otwieraja sie tylko na {max_open_m * 100:.1f} cm, "
            f"czyli {(open_m - width_m) * 1000:.0f} mm nad wymiar szyszki"
        )
    close_m = max(0.0, width_m - GRIPPER_SQUEEZE_M)
    return {
        "open_m": open_m,
        "close_m": close_m,
        "open_cmd": gripper_command(open_m, max_open_m),
        "close_cmd": gripper_command(close_m, max_open_m),
        "max_open_m": max_open_m,
        "uwagi": notes,
    }


# ---------------------------------------------------------------- plan chwytu


def grasp_point_ground(target: dict) -> tuple[np.ndarray, float]:
    """Punkt chwytu w ukladzie ziemi (X przod, Y lewo, Z w gore) + wysokosc chwytu."""
    height = float(target.get("height_m", 0.0) or 0.0)
    grasp_height = max(MIN_GRASP_HEIGHT_M, height * GRASP_HEIGHT_FRACTION)
    # cel.json ma lateral dodatni w PRAWO - tu os Y idzie w lewo.
    point = np.array(
        [float(target["forward_m"]), -float(target["lateral_m"]), grasp_height], dtype=float
    )
    return point, grasp_height


def desired_jaw_yaw_deg(target: dict, transform: Transform, extra_offset_deg: float) -> float:
    angle = float(target.get("angle_deg", 0.0) or 0.0)
    long_axis_ground = angle + LONG_AXIS_YAW_FROM_DETECTOR_DEG
    jaw_ground = long_axis_ground + JAW_YAW_FROM_LONG_AXIS_DEG + extra_offset_deg
    return wrap90(transform.yaw_to_base(jaw_ground))


def plan_grasp(
    target: dict,
    transform: Transform,
    arm: ArmModel,
    reach_limit_m: float | None = None,
    approach_pitches_deg=APPROACH_PITCH_CANDIDATES_DEG,
    jaw_yaw_offset_deg: float = 0.0,
    clearance_m: float = APPROACH_CLEARANCE_M,
    lift_m: float = LIFT_M,
) -> dict:
    """Pelna sekwencja chwytu. Rzuca PlanError, gdy celu nie da sie dosiegnac."""
    target_required_keys(target)
    reach_limit = arm.reach_bound_m if reach_limit_m is None else float(reach_limit_m)

    gripper = plan_gripper(float(target["width_m"]), transform.gripper_max_open_m)
    ground_point, grasp_height = grasp_point_ground(target)
    grasp_base = transform.to_base(ground_point)
    jaw_yaw = desired_jaw_yaw_deg(target, transform, jaw_yaw_offset_deg)

    # Tania bramka przed IK: nawet wyprostowane ramie tam nie dosiega.
    arm.check_reach_bound(grasp_base, "punkt chwytu", reach_limit)

    attempts = []
    for pitch in approach_pitches_deg:
        rad = math.radians(pitch)
        axis = np.array([math.cos(rad), 0.0, -math.sin(rad)])
        axis = axis / np.linalg.norm(axis)
        # Zawisniecie i podniesienie licza sie PO OSI PODEJSCIA, nie w pionie -
        # inaczej przy przechylonym podejsciu szczeka wchodzi w szyszke z boku.
        points = {
            "nad szyszka": grasp_base - axis * clearance_m,
            "opuszczenie": grasp_base,
            "podniesienie": grasp_base - axis * lift_m,
        }

        solutions = {}
        failures = []
        seed = None
        for label, point in points.items():
            solution = arm.solve(point, axis, jaw_yaw, seed=seed)
            solutions[label] = solution
            seed = solution["angles_rad"]
            if solution["position_error_m"] > IK_POSITION_TOLERANCE_M:
                failures.append(
                    f"{label}: IK nie doszlo do punktu, zostalo "
                    f"{solution['position_error_m'] * 1000:.0f} mm (prog "
                    f"{IK_POSITION_TOLERANCE_M * 1000:.0f} mm)"
                )
            elif solution["axis_error_deg"] > IK_AXIS_TOLERANCE_DEG:
                failures.append(
                    f"{label}: os podejscia o {solution['axis_error_deg']:.1f} st obok "
                    f"(prog {IK_AXIS_TOLERANCE_DEG:.0f} st)"
                )
            violations = arm.limit_violations(solution["angles_rad"])
            if violations:
                failures.append(f"{label}: poza limitem przegubu - {'; '.join(violations)}")

        attempts.append({"pitch_deg": pitch, "failures": failures})
        if not failures:
            return _assemble_plan(
                target=target,
                transform=transform,
                arm=arm,
                gripper=gripper,
                solutions=solutions,
                points=points,
                pitch_deg=pitch,
                axis=axis,
                grasp_base=grasp_base,
                ground_point=ground_point,
                grasp_height=grasp_height,
                jaw_yaw=jaw_yaw,
                attempts=attempts,
                clearance_m=clearance_m,
                lift_m=lift_m,
            )

    detail = "\n".join(
        f"    podejscie {a['pitch_deg']:.0f} st: " + "; ".join(a["failures"]) for a in attempts
    )
    raise PlanError(
        "nie ma rozwiazania IK dla tego celu - ODRZUCONY, nic nie zostalo obciete.\n"
        f"  punkt chwytu w bazie: x {grasp_base[0]:+.3f}  y {grasp_base[1]:+.3f}  "
        f"z {grasp_base[2]:+.3f} m\n"
        f"  probowane katy podejscia: {', '.join(f'{p:.0f}' for p in approach_pitches_deg)} st\n"
        f"{detail}\n"
        "  Najczestsza przyczyna: cel za daleko albo za nisko dla pionowego chwytu. "
        "Podjedz blizej (--pick-forward mniej) albo przemysl montaz ramienia."
    )


def _assemble_plan(**kw) -> dict:
    """Sklejenie krokow w kolejnosci wykonania. Kazdy krok = jedna komenda."""
    arm: ArmModel = kw["arm"]
    transform: Transform = kw["transform"]
    gripper = kw["gripper"]
    solutions = kw["solutions"]
    points = kw["points"]

    def step(name, description, angles_rad, gripper_cmd, gripper_open_m, point_base):
        return {
            "nazwa": name,
            "opis": description,
            "katy_deg": {n: math.degrees(v) for n, v in angles_rad.items()},
            "lerobot": lerobot_action(angles_rad, gripper_cmd, transform),
            "chwytak_cmd": gripper_cmd,
            "chwytak_rozwarcie_m": gripper_open_m,
            "punkt_bazy_m": None if point_base is None else np.asarray(point_base).tolist(),
        }

    start_rad = {n: math.radians(v) for n, v in START_POSE_DEG.items()}
    above = solutions["nad szyszka"]
    down = solutions["opuszczenie"]
    up = solutions["podniesienie"]
    start_open_m = START_GRIPPER_CMD / 100.0 * gripper["max_open_m"]

    steps = [
        step(
            "pozycja wyjsciowa",
            "poza startowa z arm_control.HOME_POSE (placeholder do poprawy po kalibracji)",
            start_rad,
            START_GRIPPER_CMD,
            start_open_m,
            None,
        ),
        step(
            "nad szyszka",
            f"zawisniecie {kw['clearance_m'] * 100:.0f} cm nad punktem chwytu, po osi podejscia",
            above["angles_rad"],
            START_GRIPPER_CMD,
            start_open_m,
            points["nad szyszka"],
        ),
        step(
            "otwarcie chwytaka",
            f"szczeki na {gripper['open_m'] * 1000:.0f} mm, czyli "
            f"{(gripper['open_m'] - float(kw['target']['width_m'])) * 1000:.0f} mm "
            "nad os chwytania szyszki",
            above["angles_rad"],
            gripper["open_cmd"],
            gripper["open_m"],
            points["nad szyszka"],
        ),
        step(
            "opuszczenie na szyszke",
            f"zejscie na wysokosc chwytu {kw['grasp_height'] * 1000:.0f} mm nad ziemia",
            down["angles_rad"],
            gripper["open_cmd"],
            gripper["open_m"],
            points["opuszczenie"],
        ),
        step(
            "zacisniecie",
            f"szczeki na {gripper['close_m'] * 1000:.0f} mm, {GRIPPER_SQUEEZE_M * 1000:.0f} mm "
            "pod wymiar - docisk",
            down["angles_rad"],
            gripper["close_cmd"],
            gripper["close_m"],
            points["opuszczenie"],
        ),
        step(
            "podniesienie",
            f"wycofanie {kw['lift_m'] * 100:.0f} cm po osi podejscia, z szyszka w szczekach",
            up["angles_rad"],
            gripper["close_cmd"],
            gripper["close_m"],
            points["podniesienie"],
        ),
    ]

    warnings_out = list(gripper["uwagi"])
    for label, solution in solutions.items():
        error = solution["jaw_yaw_error_deg"]
        if error is not None and abs(error) > JAW_YAW_TOLERANCE_DEG:
            warnings_out.append(
                f"{label}: obrot szczek nie wyszedl - brakuje {error:+.1f} st "
                "(limit wrist_roll albo poza zakresem). Szyszka jest prawie obrotowa, "
                "wiec to raczej nie zerwie chwytu, ale wiedz o tym."
            )
    if not transform.measured:
        warnings_out.append(
            "transformata kamera -> baza jest OSZACOWANA, nie zmierzona - te katy sa "
            "tak dobre jak to oszacowanie"
        )
    if any(
        abs(transform.joint_sign[n] - 1.0) > 1e-9 or abs(transform.joint_offset[n]) > 1e-9
        for n in JOINT_NAMES
    ):
        warnings_out.append("katy lerobota licza sie ze znakiem/offsetem z konfiguracji")
    else:
        warnings_out.append(
            "ZALOZENIE: jednostki lerobota = stopnie URDF 1:1 (znak +1, offset 0). "
            "NIESPRAWDZONE - przed pierwszym ruchem porownaj z 'arm_control.py status'"
        )
    warnings_out.append(
        f"ZALOZENIE: maksymalne rozwarcie szczek {gripper['max_open_m'] * 1000:.0f} mm i "
        "liniowe przelozenie na komende 0..100 - nie zmierzone"
    )

    return {
        "kroki": steps,
        "chwytak": gripper,
        "podejscie_deg": kw["pitch_deg"],
        "os_podejscia": np.asarray(kw["axis"]).tolist(),
        "punkt_chwytu_baza_m": np.asarray(kw["grasp_base"]).tolist(),
        "punkt_chwytu_ziemia_m": np.asarray(kw["ground_point"]).tolist(),
        "wysokosc_chwytu_m": kw["grasp_height"],
        "obrot_szczek_deg": kw["jaw_yaw"],
        "residua": {
            label: {
                "pozycja_mm": solution["position_error_m"] * 1000.0,
                "os_deg": solution["axis_error_deg"],
                "obrot_szczek_deg": solution["jaw_yaw_error_deg"],
            }
            for label, solution in solutions.items()
        },
        "proby_podejscia": kw["attempts"],
        "ostrzezenia": warnings_out,
    }


def lerobot_action(angles_rad: dict, gripper_cmd: float, transform: Transform) -> dict:
    """Katy URDF [rad] -> slownik dla arm_control.move_to (jednostki lerobota)."""
    action = {}
    for name in JOINT_NAMES:
        deg = math.degrees(angles_rad.get(name, 0.0))
        action[name] = transform.joint_sign[name] * deg + transform.joint_offset[name]
    action[GRIPPER_JOINT] = float(gripper_cmd)
    return action


# ---------------------------------------------------------------- wypis


def print_header(target: dict, header: dict, index: int, path: str, transform: Transform, arm: ArmModel) -> None:
    print("=" * 72)
    print("PLAN CHWYTU SZYSZKI - z zapamietanej pozycji, kamera na tym dystansie nie widzi")
    print("=" * 72)
    print(f"Cel #{index} z {path} (skan: {header.get('scanned_at', 'brak znacznika czasu')})")
    print(
        f"  do przodu {target['forward_m']:.3f} m, w bok {target['lateral_m']:+.3f} m "
        f"(dodatnie = w prawo), po ziemi "
        f"{target.get('ground_distance_m', float('nan')):.3f} m"
    )
    print(
        f"  os chwytania {float(target['width_m']) * 100:.1f} cm, dlugosc "
        f"{float(target.get('length_m', 0.0)) * 100:.1f} cm, wysokosc "
        f"{float(target.get('height_m', 0.0)) * 100:.1f} cm, kat "
        f"{float(target.get('angle_deg', 0.0)):+.1f} st"
    )
    seen = target.get("seen_in")
    if seen:
        print(
            f"  widziana w {seen} klatkach, rozrzut "
            f"{target.get('spread_forward_mm', float('nan')):.1f} mm w przod"
        )
    print()
    for line in transform.describe():
        print(line)
    print(
        f"Zasieg z URDF: gorna granica {arm.reach_bound_m * 100:.1f} cm od osi shoulder_pan "
        "(suma dlugosci czlonow, ramie w linii prostej)."
    )
    print(
        "  Realna obwiednia jest mniejsza i zalezy od kata podejscia - rozstrzyga "
        "residuum IK, nie ta liczba."
    )
    print()


def print_drive(drive: dict) -> None:
    print("-" * 72)
    print("DOJAZD (kontrolnie; wykonuje go drive_to_target.py, nie ten skrypt)")
    print(f"  1. obroc na miejscu o {drive['obrot_deg']:+.1f} st")
    print(f"  2. jedz w przod {drive['przod_m']:.3f} m")
    after = drive["po_dojazdzie"]
    print(
        f"  po dojazdzie szyszka powinna byc {after['forward_m']:.3f} m przed kamera, "
        f"{after['lateral_m']:+.3f} m w bok"
    )
    for note in drive["uwagi"]:
        print(f"  UWAGA: {note}")
    print(
        "  Blad dojazdu wchodzi 1:1 w chwyt - kamera na tym dystansie nie zweryfikuje "
        "pozycji szyszki."
    )
    print()


def print_plan(plan: dict, dry_run: bool) -> None:
    print("-" * 72)
    point = plan["punkt_chwytu_baza_m"]
    print(
        f"CHWYT: podejscie {plan['podejscie_deg']:.0f} st od poziomu, punkt chwytu w bazie "
        f"x {point[0]:+.3f}  y {point[1]:+.3f}  z {point[2]:+.3f} m"
    )
    print(
        f"  wysokosc chwytu {plan['wysokosc_chwytu_m'] * 1000:.0f} mm nad ziemia, "
        f"obrot linii szczek {plan['obrot_szczek_deg']:+.1f} st"
    )
    print()
    total = len(plan["kroki"])
    for number, step in enumerate(plan["kroki"], start=1):
        print(f"KROK {number}/{total}  {step['nazwa'].upper()}")
        print(f"  {step['opis']}")
        if step["punkt_bazy_m"] is not None:
            p = step["punkt_bazy_m"]
            print(f"  punkt w bazie [m]: x {p[0]:+.3f}  y {p[1]:+.3f}  z {p[2]:+.3f}")
        angles = "  ".join(f"{name} {step['katy_deg'][name]:+7.1f}" for name in JOINT_NAMES)
        print(f"  przeguby URDF [st]: {angles}")
        print(
            f"  chwytak: komenda {step['chwytak_cmd']:5.1f} "
            f"(ok. {step['chwytak_rozwarcie_m'] * 1000:.0f} mm rozwarcia)"
        )
        sent = "  ".join(
            f"{name}={step['lerobot'][name]:+.1f}" for name in JOINT_NAMES + [GRIPPER_JOINT]
        )
        print(f"  do wyslania: {sent}")
        print()

    print("Residua IK (na ile IK trafilo w punkt):")
    for label, residual in plan["residua"].items():
        jaw = residual["obrot_szczek_deg"]
        jaw_text = "brak" if jaw is None else f"{jaw:+.1f} st"
        print(
            f"  {label:16s} pozycja {residual['pozycja_mm']:5.1f} mm, os "
            f"{residual['os_deg']:4.1f} st, obrot szczek {jaw_text}"
        )
    if len(plan["proby_podejscia"]) > 1:
        print(
            "  Pionowe podejscie nie wyszlo - uzyto "
            f"{plan['podejscie_deg']:.0f} st. Odrzucone: "
            + "; ".join(
                f"{a['pitch_deg']:.0f} st ({len(a['failures'])} problemow)"
                for a in plan["proby_podejscia"][:-1]
            )
        )
    print()
    print("ZALOZENIA I OSTRZEZENIA:")
    for note in plan["ostrzezenia"]:
        print(f"  - {note}")
    print()
    if dry_run:
        print(
            "DRY-RUN: nic nie zostalo wyslane. Ruch dopiero po podaniu --port "
            "(np. --port COM10)."
        )


# ---------------------------------------------------------------- wykonanie


def execute_plan(plan: dict, port: str, arm_id: str, steps_per_move: int, settle_s: float) -> None:
    """Wyslanie planu na ramie. Wchodzi tu tylko przy jawnym --port."""
    try:
        import arm_control
    except Exception as exc:  # lerobot ciagnie torcha, na laptopie moze go nie byc
        raise PlanError(
            "nie da sie zaimportowac arm_control (a z nim lerobota): "
            f"{type(exc).__name__}: {exc}. "
            "Plan powyzej jest policzony - do ruchu potrzebny jest lerobot "
            "w tym samym srodowisku."
        ) from exc

    import time

    print(f"RUCH: port {port}, id {arm_id}. Ctrl+C przerywa.")
    robot = arm_control.make_arm(port=port, arm_id=arm_id)
    robot.connect(calibrate=False)
    try:
        for number, step in enumerate(plan["kroki"], start=1):
            print(f"  krok {number}/{len(plan['kroki'])}: {step['nazwa']}")
            arm_control.move_to(robot, step["lerobot"], steps=steps_per_move)
            time.sleep(settle_s)
    finally:
        robot.disconnect()
    print("Sekwencja wykonana.")


# ---------------------------------------------------------------- CLI


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plan chwytu szyszki z cel.json (domyslnie tylko wypis, bez ruchu)"
    )
    parser.add_argument("--target", default=TARGET_PATH, help="plik celu z scan_cones.py")
    parser.add_argument("--index", type=int, default=0, help="ktory cel z pliku")
    parser.add_argument(
        "--transform", default=TRANSFORM_PATH, help="plik transformaty kamera -> baza"
    )
    parser.add_argument("--urdf", default=URDF_PATH)
    parser.add_argument(
        "--standoff",
        type=float,
        default=DEFAULT_STANDOFF_M,
        help="gdzie przed kamera ma wyladowac szyszka po dojazdzie [m]; TA SAMA "
        "liczba co --standoff w drive_to_target.py",
    )
    parser.add_argument(
        "--no-drive",
        action="store_true",
        help="licz chwyt na surowych wspolrzednych z cel.json, bez dojazdu platformy",
    )
    parser.add_argument(
        "--clearance", type=float, default=APPROACH_CLEARANCE_M, help="ile nad celem zawisnac [m]"
    )
    parser.add_argument("--lift", type=float, default=LIFT_M, help="ile podniesc po chwycie [m]")
    parser.add_argument(
        "--jaw-yaw-offset-deg",
        type=float,
        default=0.0,
        help="poprawka obrotu szczek, gdy kat z wizji okaze sie liczony inaczej",
    )
    parser.add_argument(
        "--max-reach",
        type=float,
        default=None,
        help="nadpisz gorna granice zasiegu [m] (domyslnie z URDF)",
    )
    parser.add_argument(
        "--dry-run",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="domyslnie wlaczony; --port go wylacza, --dry-run razem z --port = plan bez ruchu",
    )
    parser.add_argument(
        "--port", default=None, help="port szeregowy ramienia, np. COM10. BEZ NIEGO NIC SIE NIE RUSZA"
    )
    parser.add_argument("--id", default="so101", help="id kalibracji lerobota")
    parser.add_argument("--move-steps", type=int, default=25, help="interpolacja na jeden krok")
    parser.add_argument("--settle", type=float, default=0.4, help="pauza po kroku [s]")
    parser.add_argument("--json", default=None, help="zapisz plan do pliku JSON")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    dry_run = True if args.dry_run is None else args.dry_run
    if args.port and args.dry_run is None:
        dry_run = False
    if not dry_run and not args.port:
        print("BLAD: ruch wymaga --port (bez portu zostaje dry-run).")
        return 2

    try:
        transform = load_transform(args.transform)
        target, header = load_target(args.target, args.index)
        arm = ArmModel(args.urdf)
        print_header(target, header, args.index, args.target, transform, arm)

        if args.no_drive:
            print("-" * 72)
            print("DOJAZD POMINIETY (--no-drive): chwyt liczony na surowych wspolrzednych.")
            print()
            grasp_target = target
            drive = None
        else:
            drive = plan_drive(target, args.standoff, header)
            print_drive(drive)
            # Po dojazdzie szyszka lezy na osi, w zadanej odleglosci. Wymiary
            # zostaja z pomiaru, ale kat trzeba obrocic razem z platforma: cel o
            # namiarze b po obrocie w prawo o b ma w ukladzie detektora (w prawo,
            # w przod) kat wiekszy o b, wiec kat dluzszej osi tez rosnie o b.
            grasp_target = dict(target)
            grasp_target["forward_m"] = args.standoff
            grasp_target["lateral_m"] = 0.0
            grasp_target["ground_distance_m"] = args.standoff
            grasp_target["bearing_deg"] = 0.0
            grasp_target["angle_deg"] = float(target.get("angle_deg", 0.0) or 0.0) + drive[
                "obrot_deg"
            ]

        plan = plan_grasp(
            grasp_target,
            transform,
            arm,
            reach_limit_m=args.max_reach,
            jaw_yaw_offset_deg=args.jaw_yaw_offset_deg,
            clearance_m=args.clearance,
            lift_m=args.lift,
        )
        print_plan(plan, dry_run)

        if args.json:
            payload = {
                "cel": target,
                "cel_do_chwytu": grasp_target,
                "dojazd": drive,
                "plan": {
                    key: value
                    for key, value in plan.items()
                    if key not in ("proby_podejscia",)
                },
            }
            with open(args.json, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, ensure_ascii=True)
            print(f"Plan zapisany do {args.json}")

        if not dry_run:
            execute_plan(plan, args.port, args.id, args.move_steps, args.settle)
        return 0

    except PlanError as exc:
        print()
        print("ODRZUCONE: " + str(exc))
        return 2
    except KeyboardInterrupt:
        print("\nPrzerwano.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
