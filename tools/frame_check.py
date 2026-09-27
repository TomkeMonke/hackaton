"""Krok 1 przed jakimkolwiek IK (placo, GraspGenX): zera i znaki stawow.

Pytanie, na ktore odpowiada: czy "stopnie" z lerobota (kalibracja so101.json,
recznie poprawiony offset barku) to te same katy, ktorych oczekuje URDF
so101_new_calib.urdf? Jesli nie, kazde IK pojedzie w zla strone
(docs/HARDWARE.md, pulapka 12).

Metoda (bez zewnetrznych sensorow, sprawdza czlowiek z linijka):
  1. Odczyt stawow, FK z placo (lerobot.model.kinematics) -> polozenie koncowki
     (ramka gripper_frame_link) w ukladzie podstawy ramienia.
  2. Dla kazdego stawu po kolei: ruch o +DELTA stopni (tylko ten staw), odczyt,
     FK, wydruk PRZEWIDYWANEGO przesuniecia koncowki slowami
     ("+X 8.1 cm, -Z 2.3 cm"). Operator patrzy na ramie i odpowiada, czy
     koncowka faktycznie tak sie ruszyla (t/n), potem ramie wraca.
  3. Raport JSON (frame_check.json): dla kazdego stawu zadany i zmierzony
     ruch, FK przed/po, opis, werdykt operatora, notatka.

Interpretacja: staw, dla ktorego operator mowi "n", ma zly znak lub zle zero
wzgledem URDF -> poprawka w warstwie miedzy lerobotem a kinematyka
(offset/znak na staw), NIE w pliku kalibracji serw (NIE `lerobot calibrate`).

Uruchomienie NA Pi, interaktywnie, z wylacznikiem w rece:
    .venv/bin/python tools/frame_check.py                 # ramie z configu
    .venv/bin/python tools/frame_check.py --delta 10      # mniejszy krok
    .venv/bin/python tools/frame_check.py --fake          # bez sprzetu, tylko FK

lerobot/placo importowane leniwie: modul da sie zaimportowac na laptopie
(testy: tests/test_frame_check.py z atrapa kinematyki).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Callable, Protocol

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pinecone_bot.arm import HOME_POSE, JOINT_NAMES  # noqa: E402

ARM_JOINTS = [j for j in JOINT_NAMES if j != "gripper"]
DEFAULT_DELTA_DEG = 15.0
GRIPPER_FRAME = "gripper_frame_link"
URDF_CANDIDATES = (
    # kopia z SO-ARM100 razem z siatkami STL (placo laduje geometrie)
    os.path.join("examples", "phone_to_so100", "SO101", "so101_new_calib.urdf"),
    os.path.join("so101_urdf", "so101_new_calib.urdf"),
)


# ---------------------------------------------------------------------------
# Czysta logika (testowalna bez sprzetu)
# ---------------------------------------------------------------------------

class ArmApi(Protocol):
    def read(self) -> dict: ...
    def move(self, target: dict) -> None: ...


class Kinematics(Protocol):
    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray: ...


def find_urdf(repo_root: str = REPO_ROOT) -> str:
    for rel in URDF_CANDIDATES:
        p = os.path.join(repo_root, rel)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError("brak URDF: " + ", ".join(URDF_CANDIDATES))


def pose_to_array(pose: dict) -> np.ndarray:
    """Slownik {staw: stopnie} -> tablica w kolejnosci JOINT_NAMES (6 wartosci)."""
    return np.array([float(pose.get(j, 0.0)) for j in JOINT_NAMES], dtype=float)


def fk_xyz(kin: Kinematics, pose: dict) -> np.ndarray:
    """Polozenie koncowki [m] w ukladzie podstawy dla stawow w stopniach lerobota."""
    T = np.asarray(kin.forward_kinematics(pose_to_array(pose)), dtype=float)
    return T[:3, 3].copy()


def describe_delta(delta_m: np.ndarray, min_cm: float = 0.5) -> str:
    """Przesuniecie [m] -> slowa, np. "+X 8.1 cm, -Z 2.3 cm". Z = gora/dol."""
    parts = []
    for axis, value in zip("XYZ", delta_m):
        cm = float(value) * 100.0
        if abs(cm) < min_cm:
            continue
        sign = "+" if cm > 0 else "-"
        extra = ""
        if axis == "Z":
            extra = " (w gore)" if cm > 0 else " (w dol)"
        parts.append(f"{sign}{axis} {abs(cm):.1f} cm{extra}")
    return ", ".join(parts) if parts else f"brak ruchu (< {min_cm} cm)"


def check_joint(arm: ArmApi, kin: Kinematics, start: dict, joint: str, delta_deg: float,
                ask: Callable[[str], str], say: Callable[[str], None] = print,
                settle_s: float = 0.0) -> dict:
    """Ruch jednego stawu o delta, FK przed/po, pytanie do operatora, powrot."""
    before_pose = arm.read()
    before = fk_xyz(kin, before_pose)
    target = dict(start)
    target[joint] = float(start[joint]) + float(delta_deg)
    say(f"\n== {joint}: {start[joint]:+.1f} -> {target[joint]:+.1f} st")
    arm.move(target)
    if settle_s > 0:
        time.sleep(settle_s)
    after_pose = arm.read()
    after = fk_xyz(kin, after_pose)
    d = after - before
    desc = describe_delta(d)
    measured = float(after_pose[joint]) - float(before_pose[joint])
    say(f"   zmierzony ruch stawu: {measured:+.1f} st (zadano {delta_deg:+.1f})")
    say(f"   MODEL (URDF) mowi: koncowka przesunela sie {desc}")
    answer = ask("   Czy koncowka ruszyla sie tak naprawde? [t/n], albo notatka: ").strip()
    ok = None
    note = ""
    if answer.lower() in ("t", "tak", "y", "yes"):
        ok = True
    elif answer.lower() in ("n", "nie", "no"):
        ok = False
    else:
        note = answer
    arm.move(start)
    if settle_s > 0:
        time.sleep(settle_s)
    return {
        "joint": joint,
        "delta_deg": float(delta_deg),
        "measured_delta_deg": round(measured, 2),
        "pose_before": {k: round(float(v), 2) for k, v in before_pose.items()},
        "pose_after": {k: round(float(v), 2) for k, v in after_pose.items()},
        "fk_before_m": [round(float(x), 4) for x in before],
        "fk_after_m": [round(float(x), 4) for x in after],
        "predicted_delta_cm": [round(float(x) * 100.0, 1) for x in d],
        "description": desc,
        "operator_ok": ok,
        "note": note,
    }


def run_check(arm: ArmApi, kin: Kinematics, joints: list[str], delta_deg: float,
              ask: Callable[[str], str], say: Callable[[str], None] = print,
              urdf: str = "", settle_s: float = 0.0) -> dict:
    start = arm.read()
    start_xyz = fk_xyz(kin, start)
    zero_xyz = fk_xyz(kin, {j: 0.0 for j in JOINT_NAMES})
    home_xyz = fk_xyz(kin, HOME_POSE)
    say("Stawy teraz [st]: " + ", ".join(f"{j}={start[j]:+.1f}" for j in JOINT_NAMES if j in start))
    say(f"FK teraz            : {describe_xyz(start_xyz)}")
    say(f"FK URDF zero (0 st) : {describe_xyz(zero_xyz)}   <- tak URDF widzi poze 'straight'")
    say(f"FK HOME_POSE        : {describe_xyz(home_xyz)}")
    results = []
    for joint in joints:
        cmd = ask(f"\n[{joint}] Enter = ruch o {delta_deg:+.1f} st, s = pomin, q = koniec: ").strip().lower()
        if cmd == "q":
            break
        if cmd == "s":
            results.append({"joint": joint, "skipped": True})
            continue
        results.append(check_joint(arm, kin, start, joint, delta_deg, ask, say, settle_s))
    return {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "urdf": urdf,
        "gripper_frame": GRIPPER_FRAME,
        "delta_deg": float(delta_deg),
        "start_pose": {k: round(float(v), 2) for k, v in start.items()},
        "fk_start_m": [round(float(x), 4) for x in start_xyz],
        "fk_urdf_zero_m": [round(float(x), 4) for x in zero_xyz],
        "fk_home_pose_m": [round(float(x), 4) for x in home_xyz],
        "joints": results,
    }


def describe_xyz(xyz: np.ndarray) -> str:
    return "X {:+.1f} cm, Y {:+.1f} cm, Z {:+.1f} cm".format(*(float(v) * 100.0 for v in xyz))


def summarize(report: dict) -> str:
    lines = []
    for r in report.get("joints", []):
        if r.get("skipped"):
            lines.append(f"{r['joint']:>14}: pominiety")
            continue
        verdict = {True: "OK", False: "ZLE (znak/zero?)", None: "?"}[r.get("operator_ok")]
        lines.append(f"{r['joint']:>14}: {verdict:<17} model: {r['description']}"
                     + (f"  # {r['note']}" if r.get("note") else ""))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Sprzet (lerobot + placo importowane dopiero tutaj)
# ---------------------------------------------------------------------------

class FakeArm:
    """Atrapa: pamieta ostatnia komende, 'odczyt' = komenda (bez sprzetu)."""

    def __init__(self, pose: dict | None = None):
        self.pose = dict(pose or HOME_POSE)

    def read(self) -> dict:
        return dict(self.pose)

    def move(self, target: dict) -> None:
        self.pose.update({k: float(v) for k, v in target.items()})


class RealArm:
    """SO-101 przez arm_control (lerobot). move_to trzyma MAX_RELATIVE_TARGET."""

    def __init__(self, port: str, arm_id: str, steps: int = 10):
        import arm_control as ac  # noqa: WPS433 - lerobot tylko na Pi
        self._ac = ac
        self.arm = ac.make_arm(port=port, arm_id=arm_id)
        self.arm.connect(calibrate=False)
        self.steps = steps

    def read(self) -> dict:
        return self._ac.read_joint_positions(self.arm)

    def move(self, target: dict) -> None:
        self._ac.move_to(self.arm, target, steps=self.steps)

    def close(self) -> None:
        try:
            self.arm.disconnect()
        except Exception:  # noqa: BLE001
            pass


def make_kinematics(urdf: str):
    from lerobot.model.kinematics import RobotKinematics  # placo tylko na Pi
    return RobotKinematics(urdf_path=urdf, target_frame_name=GRIPPER_FRAME, joint_names=list(JOINT_NAMES))


def main() -> int:
    from pinecone_bot.config import Config

    ap = argparse.ArgumentParser(description="Zera i znaki stawow lerobot vs URDF (FK placo)")
    ap.add_argument("--delta", type=float, default=DEFAULT_DELTA_DEG, help="krok na staw [st]")
    ap.add_argument("--joints", default=",".join(ARM_JOINTS), help="lista stawow po przecinku")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--port", default=None, help="domyslnie cfg.arm.port")
    ap.add_argument("--fake", action="store_true", help="bez sprzetu: atrapa ramienia, prawdziwe FK")
    ap.add_argument("--out", default="frame_check.json")
    args = ap.parse_args()

    cfg = Config()
    urdf = args.urdf or find_urdf()
    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    bad = [j for j in joints if j not in ARM_JOINTS]
    if bad:
        print("nieznane stawy:", bad, "dostepne:", ARM_JOINTS)
        return 2
    kin = make_kinematics(urdf)
    print("URDF:", urdf)
    if args.fake:
        arm: ArmApi = FakeArm()
    else:
        print("UWAGA: ramie sie ruszy. Wylacznik w rece, kamera na ramieniu zabezpieczona.")
        arm = RealArm(port=args.port or cfg.arm.port, arm_id=cfg.arm.arm_id)
    try:
        report = run_check(arm, kin, joints, args.delta, ask=input, say=print, urdf=urdf,
                           settle_s=0.0 if args.fake else 0.8)
    finally:
        if hasattr(arm, "close"):
            arm.close()
    with open(args.out, "w", encoding="ascii") as f:
        json.dump(report, f, indent=2)
    print("\n== PODSUMOWANIE ==")
    print(summarize(report))
    print(f"\nRaport: {args.out}. Wklej go do docs/LOG.md (sekcja sesji) i do issue.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
