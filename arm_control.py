"""
Sterowanie ramieniem SO-101 (Feetech STS3215 x6) przez lerobot SOFollower.

Zaleznosci: lerobot (SO101Follower alias = SOFollower), pyserial.

Pierwsze uruchomienie WYMAGA kalibracji (interaktywnej) - patrz `calibrate()`.
Plik kalibracji trafia do: ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<ARM_ID>.json

Przeguby (kolejnosc w lancuchu, zgodna z URDF so101_new_calib.urdf):
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll   [stopnie, ok. -100..100]
    gripper                                                          [0..100, 0=zamkniety, 100=otwarty]
"""

from __future__ import annotations

import argparse
import sys
import time

from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig

ARM_PORT = "COM10"
ARM_ID = "so101"

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Bezpieczna pozycja spoczynkowa - PO KALIBRACJI dobierz recznie i podmien te wartosci
# (np. przez `python arm_control.py status` w kilku pozach i spisanie wygodnej).
HOME_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 50.0,
}

# Limit ruchu na jedno wywolanie send_action (stopnie / jednostki motoru) - zabezpieczenie
# przed gwaltownym szarpnieciem gdy target jest daleko od pozycji obecnej.
MAX_RELATIVE_TARGET = 25.0


def make_arm(port: str = ARM_PORT, arm_id: str = ARM_ID, max_relative_target: float | None = MAX_RELATIVE_TARGET) -> SO101Follower:
    config = SO101FollowerConfig(
        port=port,
        id=arm_id,
        max_relative_target=max_relative_target,
    )
    return SO101Follower(config)


def read_joint_positions(arm: SO101Follower) -> dict[str, float]:
    obs = arm.get_observation()
    return {name: obs[f"{name}.pos"] for name in JOINT_NAMES if f"{name}.pos" in obs}


def move_to(arm: SO101Follower, target: dict[str, float], steps: int = 1, step_delay: float = 0.05) -> dict[str, float]:
    """Przesuwa podane przeguby do wartosci docelowych.

    steps > 1: interpoluje liniowo od obecnej pozycji do targetu (plynniejszy ruch,
    dodatkowo do MAX_RELATIVE_TARGET z konfiguracji).
    """
    current = read_joint_positions(arm)
    for name in target:
        if name not in JOINT_NAMES:
            raise ValueError(f"nieznany przegub: {name}")

    if steps <= 1:
        action = {f"{name}.pos": val for name, val in target.items()}
        return arm.send_action(action)

    last_sent: dict[str, float] = {}
    for i in range(1, steps + 1):
        frac = i / steps
        interp = {
            name: current[name] + (target[name] - current[name]) * frac
            for name in target
            if name in current
        }
        action = {f"{name}.pos": val for name, val in interp.items()}
        last_sent = arm.send_action(action)
        time.sleep(step_delay)
    return last_sent


def open_gripper(arm: SO101Follower, value: float = 100.0) -> None:
    move_to(arm, {"gripper": value})


def close_gripper(arm: SO101Follower, value: float = 0.0) -> None:
    move_to(arm, {"gripper": value})


def go_home(arm: SO101Follower, steps: int = 20) -> None:
    move_to(arm, HOME_POSE, steps=steps)


def _parse_move_args(pairs: list[str]) -> dict[str, float]:
    target: dict[str, float] = {}
    for pair in pairs:
        if "=" not in pair:
            raise ValueError(f"zly format '{pair}', oczekiwano joint=wartosc")
        name, val = pair.split("=", 1)
        name = name.strip()
        if name not in JOINT_NAMES:
            raise ValueError(f"nieznany przegub '{name}', dostepne: {JOINT_NAMES}")
        target[name] = float(val)
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description="Sterowanie ramieniem SO-101")
    parser.add_argument("--port", default=ARM_PORT)
    parser.add_argument("--id", default=ARM_ID)
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("calibrate", help="uruchom interaktywna kalibracje")
    sub.add_parser("status", help="wypisz biezace pozycje przegubow")
    p_home = sub.add_parser("home", help="jedz do pozycji HOME_POSE")
    p_home.add_argument("--steps", type=int, default=20)

    p_move = sub.add_parser("move", help="ustaw wybrane przeguby, np. move shoulder_pan=10 gripper=50")
    p_move.add_argument("targets", nargs="+")
    p_move.add_argument("--steps", type=int, default=20)

    sub.add_parser("open", help="otworz chwytak")
    sub.add_parser("close", help="zamknij chwytak")

    args = parser.parse_args()

    arm = make_arm(port=args.port, arm_id=args.id)
    try:
        arm.connect(calibrate=(args.cmd == "calibrate"))

        if args.cmd == "calibrate":
            print("Kalibracja zakonczona.")
        elif args.cmd == "status":
            pos = read_joint_positions(arm)
            for name in JOINT_NAMES:
                print(f"{name:16s} {pos.get(name, float('nan')):8.2f}")
        elif args.cmd == "home":
            go_home(arm, steps=args.steps)
            print("W pozycji home.")
        elif args.cmd == "move":
            target = _parse_move_args(args.targets)
            sent = move_to(arm, target, steps=args.steps)
            print("Wyslano:", sent)
        elif args.cmd == "open":
            open_gripper(arm)
        elif args.cmd == "close":
            close_gripper(arm)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
