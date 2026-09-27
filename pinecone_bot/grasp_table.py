"""Tabela chwytow z uczenia: miejsce szyszki na obrazie (w HOME) -> nagrane pozycje ramienia.

Samo ramie, baza stoi. Kamera siedzi na ramieniu, wiec obraz ma sens tylko w jednej pozycji:
detekcja zawsze w HOME. Bez IK i bez kalibracji kamera-ramie: przy uczeniu czlowiek kladzie
szyszke, program zapisuje jej piksel, a czlowiek ustawia ramie reka nad szyszka ("pre")
i w pozycji chwytu ("grasp"). Przy zbieraniu bierzemy probke najblizsza w pikselach
(tools/teach_grasp.py uczy, tools/pick_loop.py zbiera).

Plik: motions/tables/grasp_table.json (ASCII); podkatalog, bo motions/*.json to same ruchy.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass

from .arm import GRIPPER_OPEN, JOINT_NAMES, Motion, Waypoint

TABLE_NAME = "grasp_table.json"


@dataclass
class GraspSample:
    px: float     # srodek szyszki na obrazie w HOME (kolumna)
    py: float     # wiersz
    pre: dict     # ramie nad szyszka (6 przegubow)
    grasp: dict   # szczeki wokol szyszki (6 przegubow)
    note: str = ""


def table_path(motions_dir: str) -> str:
    return os.path.join(motions_dir, "tables", TABLE_NAME)


def _pose(raw, where: str) -> dict:
    if not isinstance(raw, dict) or any(j not in raw for j in JOINT_NAMES):
        raise ValueError(f"{where}: pozycja musi miec przeguby {JOINT_NAMES}")
    return {j: float(raw[j]) for j in JOINT_NAMES}


def load_table(path: str) -> list:
    """Brak pliku = pusta tabela."""
    if not os.path.exists(path):
        return []
    with open(path, encoding="ascii") as fh:
        data = json.load(fh)
    samples = []
    for i, raw in enumerate(data.get("samples", [])):
        samples.append(GraspSample(
            px=float(raw["px"]), py=float(raw["py"]),
            pre=_pose(raw.get("pre"), f"{path}: probka {i} pre"),
            grasp=_pose(raw.get("grasp"), f"{path}: probka {i} grasp"),
            note=str(raw.get("note", "")),
        ))
    return samples


def save_table(path: str, samples: list) -> None:
    data = {
        "note": "tools/teach_grasp.py: piksel szyszki w HOME -> ramie nad szyszka (pre) i chwyt (grasp). "
                "Zmiana HOME, montazu kamery albo kalibracji serw = uczyc od nowa.",
        "samples": [
            {"px": round(s.px, 1), "py": round(s.py, 1),
             "pre": {j: round(s.pre[j], 2) for j in JOINT_NAMES},
             "grasp": {j: round(s.grasp[j], 2) for j in JOINT_NAMES},
             "note": s.note}
            for s in samples
        ],
    }
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="ascii") as fh:
        json.dump(data, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, path)


def pick_detection(dets: list):
    """Najwieksza cala (nie ucieta krawedzia) szyszka albo None."""
    whole = [d for d in dets if not d.partial]
    return max(whole, key=lambda d: d.area) if whole else None


def nearest_sample(samples: list, px: float, py: float, max_px: float):
    """(probka, odleglosc w px) najblizsza (px, py) albo None, gdy zadna nie jest blizej niz max_px."""
    best = None
    for s in samples:
        d = math.hypot(s.px - px, s.py - py)
        if d <= max_px and (best is None or d < best[1]):
            best = (s, d)
    return best


def grasp_motion(sample: GraspSample, squeeze: float = 0.0, slow: float = 1.0) -> Motion:
    """Nad szyszke z otwartym chwytakiem -> w dol -> zacisk (sprawdzany) -> w gore. HOME robi wolajacy.

    slow > 1 wydluza wszystkie czasy (pierwsze proby).
    """
    opened = lambda pose: {**pose, "gripper": GRIPPER_OPEN}  # noqa: E731
    closed = lambda pose: {**pose, "gripper": float(squeeze)}  # noqa: E731
    return Motion(name="pick", note=sample.note, waypoints=[
        Waypoint("nad szyszka", opened(sample.pre), 2.0 * slow),
        Waypoint("w dol", opened(sample.grasp), 1.5 * slow),
        Waypoint("zacisk", closed(sample.grasp), 1.0 * slow, check_gripper=True),
        Waypoint("w gore", closed(sample.pre), 1.5 * slow),
    ])
