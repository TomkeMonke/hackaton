"""Chwyt szyszki polityka ACT, potem wrzut do sloika nagranym ruchem (dwa etapy).

Etap 1 (uczony): `lerobot-rollout` z wytrenowanym ACT przez --grasp-s sekund. ACT byl uczony
tylko na chwycie: z HOME do szyszki, zamkniecie chwytaka, lekkie uniesienie
(dataset local/so101_grasp, docs/POLICIES_LEROBOT.md).
Etap 2 (deterministyczny): `tools/arm_play.py --motion drop_box --home-after` - nad sloik,
otwarcie, powrot tym samym torem, HOME. Sloik stoi na robocie, wiec tego nie trzeba uczyc.

Uzycie (na Pi, z katalogu repo; nic innego nie moze trzymac portu ramienia):
    python tools/act_pick.py --policy ~/models/act_grasp/pretrained_model --dry-run   # tylko komendy
    python tools/act_pick.py --policy ~/models/act_grasp/pretrained_model
    python tools/act_pick.py --policy ... --repeat 3          # 3 cykle chwyt + wrzut
    python tools/act_pick.py --policy ... --skip-drop         # sam chwyt (test polityki)

Rollout konczy sie z --robot.disable_torque_on_disconnect=false: ramie trzyma szyszke
w powietrzu, zanim arm_play przejmie port. Pierwsze uruchomienie z wylacznikiem w rece.
"""
from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TASK = "Grasp the pine cone and lift it"  # musi byc ten sam co --dataset.single_task przy nagraniu
CAMERA_SERIAL = "030522070668"  # D435 na ramieniu (lerobot-find-cameras realsense)


def rollout_cmd(policy: str, seconds: float, port: str = "/dev/robot-arm", robot_id: str = "so101",
                camera_serial: str = CAMERA_SERIAL, fps: int = 30, device: str = "cpu",
                exe: str = ".venv/bin/lerobot-rollout") -> list:
    """Komenda etapu 1. Kamera musi miec ten sam klucz i rozdzielczosc co w datasecie (wrist 640x480).

    device jawnie: wagi z laptopa maja w configu "cuda", a na Pi jej nie ma.
    """
    cameras = (f"{{ wrist: {{type: intelrealsense, serial_number_or_name: {camera_serial}, "
               f"width: 640, height: 480, fps: {fps}}}}}")
    return [
        exe,
        "--strategy.type=base",
        f"--policy.path={policy}",
        "--robot.type=so101_follower",
        f"--robot.port={port}",
        f"--robot.id={robot_id}",
        f"--robot.cameras={cameras}",
        "--robot.disable_torque_on_disconnect=false",
        f"--task={TASK}",
        f"--duration={seconds:g}",
        f"--fps={fps}",
        f"--device={device}",
        "--play_sounds=false",
    ]


def drop_cmd(port: str = "/dev/robot-arm", motion: str = "drop_box", python: str = ".venv/bin/python") -> list:
    """Komenda etapu 2: nagrany ruch do sloika i HOME na koniec."""
    return [python, "tools/arm_play.py", "--motion", motion, "--port", port, "--home-after"]


def plan(args) -> list:
    """Lista komend dla wszystkich cykli (etap 1, etap 2, etap 1, ...)."""
    steps = []
    for _ in range(args.repeat):
        steps.append(rollout_cmd(args.policy, args.grasp_s, port=args.port, fps=args.fps))
        if not args.skip_drop:
            steps.append(drop_cmd(port=args.port, motion=args.motion))
    return steps


def main(argv=None, run=subprocess.run) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", required=True, help="katalog pretrained_model z lerobot-train")
    parser.add_argument("--grasp-s", type=float, default=15.0, help="czas etapu 1 [s] (jak episode_time_s)")
    parser.add_argument("--port", default=os.environ.get("ROBOT_ARM_PORT", "/dev/robot-arm"))
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--motion", default="drop_box", help="ruch etapu 2 (motions/<name>.json)")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--skip-drop", action="store_true", help="tylko etap 1")
    parser.add_argument("--dry-run", action="store_true", help="wypisz komendy, nic nie uruchamiaj")
    args = parser.parse_args(argv)

    steps = plan(args)
    for cmd in steps:
        print("$", " ".join(shlex.quote(c) for c in cmd), flush=True)
        if args.dry_run:
            continue
        rc = run(cmd, cwd=REPO_ROOT).returncode
        if rc != 0:
            print(f"Przerwane: kod wyjscia {rc}", file=sys.stderr)
            return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
