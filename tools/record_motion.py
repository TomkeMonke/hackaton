"""Nagrywa ruch ramienia CIAGLE, gdy czlowiek prowadzi je reka (torque off).

Nastepca legacy/arm_recordings/record_demo.py: bez jazdy do HOME na starcie
(kamera siedzi na ramieniu), koniec na 'q' zamiast po stalym czasie, wynik od
razu jako motions/<name>.json, ktory odtwarza tools/arm_play.py.

Uzycie (na Pi, z katalogu repo):
    python tools/record_motion.py --name grasp_near
    python tools/record_motion.py --name drop_box --every 0.5 --note "pojemnik po lewej"

Przebieg:
  1. Polaczenie z ramieniem, odczyt pozycji, torque WYLACZONY (trzymaj ramie).
  2. Ruszaj ramieniem; pozycje sa probkowane --rate razy na sekunde.
  3. 'q' + Enter konczy: torque wraca, ramie trzyma ostatnia pozycje,
     probki co --every sekund zapisane jako waypointy (pierwszy z dojazdem 1.5 s).

Odtworzenie: python tools/arm_play.py --motion <name>
Waypointy zaleza od kalibracji serw - po jej zmianie nagraj od nowa.
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot.arm import JOINT_NAMES, Motion, Waypoint, format_motion, motion_path, save_motion  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402

FIRST_SECONDS = 1.5  # dojazd z biezacej pozy do pierwszego nagranego punktu


def subsample(samples: list, every: float, first_seconds: float = FIRST_SECONDS) -> list:
    """(t, pose) co --rate -> waypointy co `every` sekund, zawsze z pierwsza i ostatnia probka.

    seconds waypointu = rzeczywisty odstep czasu od poprzedniego (ruch odtwarza sie
    w tym tempie, w jakim byl prowadzony), pierwszy waypoint dostaje first_seconds.
    """
    if not samples:
        return []
    picked = [samples[0]]
    for t, pose in samples[1:]:
        if t - picked[-1][0] >= every:
            picked.append((t, pose))
    if picked[-1][0] != samples[-1][0]:
        picked.append(samples[-1])
    out = []
    prev_t = None
    for t, pose in picked:
        seconds = first_seconds if prev_t is None else max(0.05, t - prev_t)
        out.append(Waypoint(label=f"t{t:.2f}", pose={j: float(pose[j]) for j in JOINT_NAMES}, seconds=seconds))
        prev_t = t
    return out


def _wait_for_q(stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            line = input()
        except EOFError:
            stop.set()
            return
        if line.strip().lower() == "q":
            stop.set()
            return
        print("  'q' + Enter konczy nagrywanie")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--name", required=True, help="nazwa ruchu -> motions/<name>.json")
    parser.add_argument("--port", default=None, help="domyslnie cfg.arm.port")
    parser.add_argument("--id", default=None, help="domyslnie cfg.arm.arm_id")
    parser.add_argument("--rate", type=float, default=10.0, help="probek na sekunde")
    parser.add_argument("--every", type=float, default=0.25, help="odstep waypointow [s]")
    parser.add_argument("--note", default="", help="opis: gdzie lezala szyszka, data, kalibracja")
    parser.add_argument("--force", action="store_true", help="nadpisz istniejacy plik bez pytania")
    args = parser.parse_args()

    cfg = Config.load()
    if args.port:
        cfg.arm.port = args.port
    if args.id:
        cfg.arm.arm_id = args.id
    path = motion_path(cfg.arm.motions_dir, args.name)
    if os.path.exists(path) and not args.force:
        print(f"{path} istnieje - podaj inna nazwe albo --force")
        return 2

    import arm_control as ac  # lerobot - tylko na Pi

    arm = ac.make_arm(port=cfg.arm.port, arm_id=cfg.arm.arm_id, max_relative_target=None)
    arm.connect(calibrate=False)
    samples: list = []
    stop = threading.Event()
    try:
        print("START:", {k: round(v, 1) for k, v in ac.read_joint_positions(arm).items()})
        arm.bus.disable_torque()
        print("Torque WYLACZONY. Prowadz ramie reka. 'q' + Enter konczy i zapisuje.")
        threading.Thread(target=_wait_for_q, args=(stop,), daemon=True).start()
        period = 1.0 / max(0.5, args.rate)
        t0 = time.monotonic()
        next_t = t0
        while not stop.is_set():
            try:
                pose = ac.read_joint_positions(arm, retries=1)
            except Exception as exc:  # pojedyncza zgubiona ramka nie przerywa nagrania
                print(f"  odczyt nieudany: {exc}")
                pose = None
            if pose and all(j in pose for j in JOINT_NAMES):
                samples.append((time.monotonic() - t0, pose))
                if len(samples) % int(args.rate) == 0:
                    print(f"  {samples[-1][0]:5.1f} s, {len(samples)} probek", end="\r", flush=True)
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    finally:
        stop.set()
        print("\nWlaczam torque (ramie trzyma pozycje)...")
        try:
            arm.bus.enable_torque()
        finally:
            try:
                arm.bus.disconnect(disable_torque=False)
            except TypeError:
                arm.disconnect()

    if not samples:
        print("brak probek, nic nie zapisano")
        return 1
    waypoints = subsample(samples, args.every)
    motion = Motion(name=args.name, waypoints=waypoints, note=args.note or f"record_motion, {len(samples)} probek")
    saved = save_motion(cfg.arm.motions_dir, motion)
    print(format_motion(motion))
    print(f"zapisano {saved}")
    print(f"odtworzenie: python tools/arm_play.py --motion {args.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
