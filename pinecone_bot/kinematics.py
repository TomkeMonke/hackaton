"""
Kinematyka SO-101 dla jogu XYZ w panelu ramienia: FK z URDF i maly krok IK.

Tylko numpy i xml.etree (bez ikpy/lerobot), wiec liczy sie tez na laptopie w testach.

Uklad: base_link z URDF (so101_urdf/so101_new_calib.urdf), metry. TCP = gripper_frame_link
(punkt miedzy szczekami). Katy wchodza w stopniach lerobot (DEGREES, zero = srodek zakresu
z kalibracji). URDF "new_calib" ma to samo zero co kalibracja lerobot, wiec domyslnie
kat URDF = kat lerobot (znak +1, offset 0). Jesli jog "gora" jedzie w dol albo w bok,
znak/offset przegubu poprawia sie w pinecone_config.json: arm.urdf_sign / arm.urdf_offset_deg
(docs/HARDWARE.md, pulapka 12).

jog_xyz: TCP przesuwa sie o delta (m) w ukladzie bazy, a pochylenie chwytaka (skladowa
pionowa osi podejscia) zostaje takie jak przed ruchem. 4 niewiadome (pan, lift, elbow,
wrist_flex) i 4 warunki (x, y, z, pochylenie); wrist_roll i chwytak bez zmian.
Rozwiazanie: tlumione najmniejsze kwadraty (DLS) na jakobianie liczonym numerycznie.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_URDF = os.path.join("so101_urdf", "so101_new_calib.urdf")
BASE_LINK = "base_link"
TIP_LINK = "gripper_frame_link"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
IK_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex")

TOL_M = 0.0005          # dokladnosc pozycji po IK
PITCH_WEIGHT = 0.05     # waga pochylenia wzgledem metrow (0.05 = 1 st ~ 0.9 mm)
MAX_JOINT_JUMP_DEG = 20.0  # krok jogu zmienia przegub o wiecej = blisko osobliwosci, odrzucamy


class IKError(RuntimeError):
    """Punktu nie da sie osiagnac (zasieg, zakres przegubow, osobliwosc)."""


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis: np.ndarray, q: float) -> np.ndarray:
    x, y, z = axis
    c, s, t = math.cos(q), math.sin(q), 1.0 - math.cos(q)
    return np.array([
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ])


def _vec(text: str | None, default: str) -> np.ndarray:
    return np.array([float(v) for v in (text or default).split()])


class So101Kinematics:
    def __init__(self, urdf_path: str | None = None, sign: dict | None = None, offset_deg: dict | None = None):
        path = urdf_path or DEFAULT_URDF
        if not os.path.isabs(path):
            path = os.path.join(REPO_ROOT, path)
        self.sign = {j: float((sign or {}).get(j, 1.0)) for j in ARM_JOINTS}
        self.offset_deg = {j: float((offset_deg or {}).get(j, 0.0)) for j in ARM_JOINTS}
        self.chain = self._load_chain(path)  # [(name, typ, T_origin 4x4, axis)]
        missing = [j for j in ARM_JOINTS if j not in [c[0] for c in self.chain]]
        if missing:
            raise ValueError(f"URDF {path}: brak przegubow {missing} miedzy {BASE_LINK} a {TIP_LINK}")

    @staticmethod
    def _load_chain(path: str) -> list:
        root = ET.parse(path).getroot()
        by_child = {}
        for joint in root.findall("joint"):
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            t = np.eye(4)
            if origin is not None:
                t[:3, :3] = _rpy(*_vec(origin.get("rpy"), "0 0 0"))
                t[:3, 3] = _vec(origin.get("xyz"), "0 0 0")
            axis_el = joint.find("axis")
            axis = _vec(axis_el.get("xyz") if axis_el is not None else None, "0 0 1")
            if np.linalg.norm(axis) > 0:
                axis = axis / np.linalg.norm(axis)
            by_child[child] = (joint.get("name"), joint.get("type"), t, axis, joint.find("parent").get("link"))
        chain = []
        link = TIP_LINK
        while link != BASE_LINK:
            if link not in by_child:
                raise ValueError(f"URDF {path}: nie ma drogi z {TIP_LINK} do {BASE_LINK}")
            name, typ, t, axis, parent = by_child[link]
            chain.append((name, typ, t, axis))
            link = parent
        return list(reversed(chain))

    def to_urdf_rad(self, joint: str, deg: float) -> float:
        return math.radians(self.sign[joint] * deg + self.offset_deg[joint])

    def fk(self, pose_deg: dict) -> np.ndarray:
        """Macierz 4x4 TCP w ukladzie base_link dla katow lerobot (brakujace przeguby = 0)."""
        t = np.eye(4)
        for name, typ, origin, axis in self.chain:
            t = t @ origin
            if typ in ("revolute", "continuous"):
                q = self.to_urdf_rad(name, float(pose_deg.get(name, 0.0)))
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle(axis, q)
                t = t @ rot
        return t

    def tcp(self, pose_deg: dict) -> tuple:
        """(xyz w m, pochylenie w st: skladowa pionowa osi podejscia, -90 = chwytak w dol)."""
        t = self.fk(pose_deg)
        return t[:3, 3].copy(), math.degrees(math.asin(max(-1.0, min(1.0, t[2, 2]))))

    def _task(self, pose_deg: dict) -> np.ndarray:
        t = self.fk(pose_deg)
        return np.array([t[0, 3], t[1, 3], t[2, 3], PITCH_WEIGHT * t[2, 2]])

    def jog_xyz(self, pose_deg: dict, delta_m, limits: dict, iterations: int = 60) -> dict:
        """Nowe katy (tylko IK_JOINTS) po przesunieciu TCP o delta_m, pochylenie bez zmian.

        limits: {joint: (lo, hi)} w stopniach lerobot. IKError, gdy nie wychodzi.
        """
        pose = {j: float(pose_deg[j]) for j in ARM_JOINTS}
        start = self._task(pose)
        goal = start + np.array([*np.asarray(delta_m, dtype=float), 0.0])
        q = np.array([pose[j] for j in IK_JOINTS])
        lo = np.array([limits[j][0] for j in IK_JOINTS])
        hi = np.array([limits[j][1] for j in IK_JOINTS])

        def task_at(qv):
            return self._task({**pose, **dict(zip(IK_JOINTS, qv))})

        eps = 1e-3
        lam = 1e-4  # J w m/st (~5e-3), wieksze tlumienie dusi kroki
        cur = task_at(q)
        for _ in range(iterations):
            err = goal - cur
            if np.linalg.norm(err[:3]) < TOL_M and abs(err[3]) < PITCH_WEIGHT * 0.01:
                break
            jac = np.empty((4, len(IK_JOINTS)))
            for i in range(len(IK_JOINTS)):
                dq = np.zeros(len(IK_JOINTS))
                dq[i] = eps
                jac[:, i] = (task_at(q + dq) - cur) / eps
            step = jac.T @ np.linalg.solve(jac @ jac.T + lam * lam * np.eye(4), err)
            step = np.clip(step, -5.0, 5.0)  # stopnie na iteracje
            q = np.clip(q + step, lo, hi)
            cur = task_at(q)
        pos_err = float(np.linalg.norm((goal - cur)[:3]))
        if pos_err > max(TOL_M * 4, 0.1 * float(np.linalg.norm(delta_m))):
            at_limit = [j for j, v, a, b in zip(IK_JOINTS, q, lo, hi) if v <= a + 0.05 or v >= b - 0.05]
            why = f"przeguby na granicy zakresu: {at_limit}" if at_limit else "poza zasiegiem"
            raise IKError(f"nie da sie ({why}, blad {pos_err * 1000:.1f} mm)")
        jump = max(abs(q[i] - pose[j]) for i, j in enumerate(IK_JOINTS))
        if jump > MAX_JOINT_JUMP_DEG:
            raise IKError(f"przegub zmienilby sie o {jump:.0f} st (blisko osobliwosci) - uzyj jogu przegubow")
        return {j: float(v) for j, v in zip(IK_JOINTS, q)}
