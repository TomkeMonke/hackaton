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
import os
import random
import sys
import time

from lerobot.robots.so_follower.so_follower import SO101Follower
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig

ARM_PORT = os.environ.get("ROBOT_ARM_PORT", "COM10")
ARM_ID = "so101"

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Pozycja spoczynkowa (ramie zlozone) - pierwsza klatka demo2_fixed.csv, pod kalibracja
# z fix_shoulder_offset.py (2026-09-25). Po kazdej zmianie kalibracji spisz na nowo
# przez `./arm.sh status` w zlozonej pozie.
HOME_POSE = {
    "shoulder_pan": -5.45,
    "shoulder_lift": 88.92,
    "elbow_flex": 7.56,
    "wrist_flex": -87.87,
    "wrist_roll": 88.88,
    "gripper": 41.06,
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


def read_joint_positions(arm: SO101Follower, retries: int = 4, retry_delay: float = 0.3) -> dict[str, float]:
    last_exc: Exception | None = None
    for attempt in range(retries):
        try:
            obs = arm.get_observation()
            return {name: obs[f"{name}.pos"] for name in JOINT_NAMES if f"{name}.pos" in obs}
        except Exception as exc:
            last_exc = exc
            if attempt < retries - 1:
                time.sleep(retry_delay)
    assert last_exc is not None
    raise last_exc


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


# Pozycja zerowa kalibracji (srodek zakresu kazdego serwa) - dla SO-101 to ramie
# wyprostowane pionowo do gory. Gripper otwarty na 100, zeby nic nie trzymal w drodze.
STRAIGHT_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 100.0,
}


def go_straight(arm: SO101Follower, steps: int = 30) -> None:
    move_to(arm, STRAIGHT_POSE, steps=steps)


def gong(
    arm: SO101Follower,
    joint: str = "wrist_flex",
    windup_deg: float = -70.0,
    strike_deg: float = 90.0,
    hold_s: float = 0.35,
    return_steps: int = 15,
) -> None:
    """Zamach i uderzenie jednym przegubem - jak w gong.

    1. Powolny, bezpieczny zamach do pozycji windup (interpolowany, respektuje
       max_relative_target z configu).
    2. Pelnym pedem uderzenie w strike_deg - pojedynczy send_action bez interpolacji
       i bez klamrowania (max_relative_target wylaczony na czas ciosu), wiec serwo
       jedzie najszybciej jak potrafi.
    3. Powrot do pozycji neutralnej.
    """
    if joint not in DANCE_RANGE:
        raise ValueError(f"nieznany przegub do gonga: {joint}, dostepne: {list(DANCE_RANGE)}")

    current = read_joint_positions(arm)
    windup = dict(current)
    windup[joint] = windup_deg
    move_to(arm, windup, steps=20, step_delay=0.03)
    time.sleep(0.15)

    prev_max_relative_target = arm.config.max_relative_target
    arm.config.max_relative_target = None
    try:
        strike = dict(current)
        strike[joint] = strike_deg
        arm.send_action({f"{name}.pos": val for name, val in strike.items()})
        time.sleep(hold_s)
    finally:
        arm.config.max_relative_target = prev_max_relative_target

    move_to(arm, current, steps=return_steps, step_delay=0.03)


# Zakres losowych ruchow (stopnie od 0) - ok. 90% policzonego zakresu z kalibracji
# so101.json (range_min/max -> stopnie: (max-min)/2 * 360/4095), zeby zostawic margines
# przed mechanicznymi ogranicznikami. gripper w jednostkach 0-100.
DANCE_RANGE = {
    "shoulder_pan": 109.0,
    "shoulder_lift": 161.0,
    "elbow_flex": 161.0,
    "wrist_flex": 94.0,
    "wrist_roll": 162.0,
}

# Skok wzgledem poprzedniego celu na jeden tick - mniejszy = wolniejszy, plynniejszy ruch
# nawet przy duzym DANCE_RANGE. Robione w Pythonie (nie przez bus'owy max_relative_target,
# zeby uniknac dodatkowego sync_read Present_Position przy kazdym send_action).
DANCE_MAX_STEP = 8.0
DANCE_GRIPPER_START = 50.0
DANCE_GRIPPER_MAX_STEP = 12.0


def dance(arm: SO101Follower, duration_s: float = 20.0, tick_s: float = 0.6) -> None:
    """Losowe, ciagle ruchy przez `duration_s` sekund - w granicach DANCE_RANGE.

    Kazdy nowy cel (wliczajac gripper) jest ograniczony krokiem wzgledem poprzedniego,
    zeby serwa nie dostawaly gwaltownych skokow kierunku (przyczyna przeciazenia/
    zawieszenia bus'a). Bledy komunikacji sa logowane i pomijane, zeby caly taniec sie
    nie wywalil od jednego zgubionego pakietu.
    """
    current = {name: 0.0 for name in DANCE_RANGE}
    current["gripper"] = DANCE_GRIPPER_START

    # Wlasny step-limiting zastepuje bus'owy max_relative_target na czas tanca, zeby
    # send_action nie robil dodatkowego sync_read przed kazdym zapisem.
    prev_max_relative_target = arm.config.max_relative_target
    arm.config.max_relative_target = None
    try:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            target = {}
            for name, rng in DANCE_RANGE.items():
                desired = random.uniform(-rng, rng)
                step = max(-DANCE_MAX_STEP, min(DANCE_MAX_STEP, desired - current[name]))
                target[name] = current[name] + step
            desired_gripper = random.uniform(0.0, 100.0)
            step = max(
                -DANCE_GRIPPER_MAX_STEP,
                min(DANCE_GRIPPER_MAX_STEP, desired_gripper - current["gripper"]),
            )
            target["gripper"] = current["gripper"] + step
            current = target
            try:
                arm.send_action({f"{name}.pos": val for name, val in target.items()})
            except Exception as exc:
                print(f"(pominieto blad komunikacji: {exc})")
            time.sleep(tick_s)
    finally:
        arm.config.max_relative_target = prev_max_relative_target


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

    p_straight = sub.add_parser("straight", help="wyprostuj ramie pionowo do gory (pozycja zerowa)")
    p_straight.add_argument("--steps", type=int, default=30)

    p_move = sub.add_parser("move", help="ustaw wybrane przeguby, np. move shoulder_pan=10 gripper=50")
    p_move.add_argument("targets", nargs="+")
    p_move.add_argument("--steps", type=int, default=20)

    sub.add_parser("open", help="otworz chwytak")
    sub.add_parser("close", help="zamknij chwytak")

    p_gong = sub.add_parser("gong", help="zamach i uderzenie jednym przegubem")
    p_gong.add_argument("--joint", default="wrist_flex")
    p_gong.add_argument("--windup", type=float, default=-70.0)
    p_gong.add_argument("--strike", type=float, default=90.0)

    p_dance = sub.add_parser("dance", help="szalone losowe ruchy przez X sekund")
    p_dance.add_argument("--duration", type=float, default=20.0)
    p_dance.add_argument("--tick", type=float, default=0.6)

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
        elif args.cmd == "straight":
            go_straight(arm, steps=args.steps)
            print("Wyprostowane pionowo.")
        elif args.cmd == "gong":
            gong(arm, joint=args.joint, windup_deg=args.windup, strike_deg=args.strike)
            print("Gong!")
        elif args.cmd == "move":
            target = _parse_move_args(args.targets)
            sent = move_to(arm, target, steps=args.steps)
            print("Wyslano:", sent)
        elif args.cmd == "open":
            open_gripper(arm)
        elif args.cmd == "close":
            close_gripper(arm)
        elif args.cmd == "dance":
            print(f"Szalony taniec przez {args.duration:.0f}s...")
            dance(arm, duration_s=args.duration, tick_s=args.tick)
            try:
                go_home(arm, steps=20)
                print("Koniec, wrocilo do home.")
            except Exception as exc:
                print(f"(nie udalo sie wrocic do home po tancu: {exc})")
    finally:
        try:
            arm.disconnect()
        except Exception as exc:
            print(f"(disconnect nieudany, prawdopodobnie juz rozlaczone: {exc})")


if __name__ == "__main__":
    sys.exit(main())
