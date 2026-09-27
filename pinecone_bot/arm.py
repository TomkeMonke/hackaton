"""
Ramie "glupie": odtwarza nagrane ruchy po nazwie, nic wiecej.

Za ustawienie szyszki w nagranym miejscu odpowiada platforma. Zadnego IK.
Ruch = mala lista WAYPOINTOW w motions/<nazwa>.json (patrz load_motion),
zeby dalo sie go powtarzac i recznie poprawiac.

Sterowniki (cfg.arm.driver):
    sim         - SimArm: tylko spi i pyta symulator (world.try_grasp) o wynik
    waypoints   - WaypointArm: prawdziwe ramie przez arm_control.make_arm (lerobot)
    subprocess  - SubprocessArm: odpala zewnetrzny skrypt zespolu (legacy/arm_recordings/replay_demo.py)

Interfejs (uzywany przez maszyne stanow):
    replay(name) -> True  (ruch wykonany, chwytak cos trzyma)
                    False (ruch wykonany, chwytak zamknal sie na niczym;
                           reszta waypointow pominieta, chwytak otwarty)
                    None  (brak informacji o chwytaku)
    home()
    close()

Modul importuje sie bez lerobot: arm_control (ktory na poziomie modulu
importuje lerobot) jest ladowany dopiero w konstruktorze WaypointArm.
"""
from __future__ import annotations

import json
import logging
import os
import subprocess
import time
from dataclasses import dataclass
from typing import Callable, Protocol

log = logging.getLogger(__name__)

# Kopia arm_control.JOINT_NAMES - arm_control importuje lerobot na poziomie
# modulu, wiec nie da sie go zaimportowac na laptopie. tests/test_arm.py
# sprawdza, ze listy sa identyczne (gdy lerobot jest dostepny).
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Kopia arm_control.HOME_POSE (kalibracja z fix_shoulder_offset.py, 2026-09-25).
# WaypointArm.home() uzywa arm_control.HOME_POSE (zrodlo prawdy); ta kopia
# sluzy tylko do motions/home.json i do dry-run bez lerobot.
HOME_POSE = {
    "shoulder_pan": 1.27,
    "shoulder_lift": -85.05,
    "elbow_flex": 99.0,
    "wrist_flex": -102.11,
    "wrist_roll": 89.10,
    "gripper": 1.69,
}

GRIPPER_OPEN = 100.0   # arm_control: gripper 0 = zamkniety, 100 = otwarty
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Format pliku ruchu
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    label: str
    pose: dict            # {joint: deg}, wszystkie 6 przegubow
    seconds: float = 1.5  # czas interpolowanego dojazdu do tego punktu
    check_gripper: bool = False  # po dojezdzie sprawdz, czy chwytak cos trzyma


@dataclass
class Motion:
    name: str
    waypoints: list
    note: str = ""

    @property
    def total_seconds(self) -> float:
        return sum(wp.seconds for wp in self.waypoints)


def parse_motion(data: dict, source: str = "<dict>") -> Motion:
    """Zamienia slownik z JSON na Motion i waliduje nazwy przegubow."""
    if "waypoints" not in data or not isinstance(data["waypoints"], list):
        raise ValueError(f"{source}: brak listy 'waypoints'")
    if not data["waypoints"]:
        raise ValueError(f"{source}: pusta lista 'waypoints'")
    waypoints = []
    for i, raw in enumerate(data["waypoints"]):
        pose = raw.get("pose")
        if not isinstance(pose, dict):
            raise ValueError(f"{source}: waypoint {i} bez 'pose'")
        unknown = sorted(set(pose) - set(JOINT_NAMES))
        if unknown:
            raise ValueError(f"{source}: waypoint {i}: nieznane przeguby {unknown}")
        missing = [j for j in JOINT_NAMES if j not in pose]
        if missing:
            raise ValueError(f"{source}: waypoint {i}: brakuje przegubow {missing}")
        seconds = float(raw.get("seconds", 1.5))
        if seconds < 0:
            raise ValueError(f"{source}: waypoint {i}: ujemne 'seconds'")
        waypoints.append(Waypoint(
            label=str(raw.get("label", f"wp{i}")),
            pose={j: float(pose[j]) for j in JOINT_NAMES},
            seconds=seconds,
            check_gripper=bool(raw.get("check_gripper", False)),
        ))
    return Motion(name=str(data.get("name", source)), waypoints=waypoints, note=str(data.get("note", "")))


def resolve_motions_dir(motions_dir: str) -> str:
    """Sciezka bezwzgledna -> jak jest; wzgledna -> najpierw cwd, potem katalog repo."""
    if os.path.isabs(motions_dir):
        return motions_dir
    if os.path.isdir(motions_dir):
        return os.path.abspath(motions_dir)
    return os.path.join(REPO_ROOT, motions_dir)


def motion_path(motions_dir: str, name: str) -> str:
    return os.path.join(resolve_motions_dir(motions_dir), f"{name}.json")


def load_motion(motions_dir: str, name: str) -> Motion:
    path = motion_path(motions_dir, name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"brak pliku ruchu: {path}")
    with open(path, encoding="utf-8") as fh:
        data = json.load(fh)
    return parse_motion(data, source=path)


def save_motion(motions_dir: str, motion: Motion) -> str:
    path = motion_path(motions_dir, motion.name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {
        "name": motion.name,
        "note": motion.note,
        "waypoints": [
            {
                "label": wp.label,
                "pose": {j: round(wp.pose[j], 2) for j in JOINT_NAMES},
                "seconds": wp.seconds,
                **({"check_gripper": True} if wp.check_gripper else {}),
            }
            for wp in motion.waypoints
        ],
    }
    with open(path, "w", encoding="ascii") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=True)
        fh.write("\n")
    return path


def format_motion(motion: Motion) -> str:
    """Czytelny wydruk ruchu (dry-run)."""
    lines = [f"ruch '{motion.name}': {len(motion.waypoints)} waypointow, {motion.total_seconds:.1f} s lacznie"]
    if motion.note:
        lines.append(f"  note: {motion.note}")
    for i, wp in enumerate(motion.waypoints):
        pose = " ".join(f"{j}={wp.pose[j]:.1f}" for j in JOINT_NAMES)
        flag = "  [check_gripper]" if wp.check_gripper else ""
        lines.append(f"  {i}: {wp.label:24s} {wp.seconds:4.1f} s  {pose}{flag}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Interfejs
# ---------------------------------------------------------------------------

class Arm(Protocol):
    def replay(self, name: str) -> bool | None: ...
    def home(self) -> None: ...
    def close(self) -> None: ...


# ---------------------------------------------------------------------------
# sim
# ---------------------------------------------------------------------------

class SimArm:
    """Symulator: spi tyle, ile trwa ruch, i pyta swiat, czy chwyt sie udal."""

    def __init__(self, cfg, world=None, sleep: Callable[[float], None] = time.sleep):
        self.cfg = cfg
        self.world = world
        self._sleep = sleep
        self.replayed: list = []  # historia wywolan (do testow/logow)

    def _duration(self, name: str) -> float:
        try:
            return load_motion(self.cfg.arm.motions_dir, name).total_seconds
        except (FileNotFoundError, ValueError):
            return 3.0  # brak pliku w symulacji to nie blad - przyjmij typowy czas

    def replay(self, name: str) -> bool | None:
        self.replayed.append(name)
        self._sleep(self._duration(name))
        if self.world is None or not hasattr(self.world, "try_grasp"):
            return None
        return bool(self.world.try_grasp(name))

    def home(self) -> None:
        self._sleep(self._duration("home"))

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# waypoints (prawdziwe ramie)
# ---------------------------------------------------------------------------

class WaypointArm:
    """Odtwarza motions/<name>.json na SO-101 przez lerobot (arm_control.make_arm).

    Dojazd do kazdego waypointu jest interpolowany liniowo od biezacej pozycji
    w czasie `seconds` (ok. rate_hz komend/s). Wymaga max_relative_target=None:
    z ustawionym limitem kazdy send_action robi dodatkowy sync_read i przy
    szybkiej petli zapycha magistrale Feetech (PROGRESS.md). Interpolacja
    liczona jest w Pythonie, wiec kroki sa male niezaleznie od tego.

    arm_control.move_to nie jest uzywane: interpoluje po liczbie krokow, a nie
    po czasie, i ma wbudowane time.sleep (nie do wstrzykniecia w testach).
    """

    def __init__(
        self,
        cfg,
        arm=None,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        rate_hz: float = 30.0,
        settle_s: float = 0.4,
        verify_tol_deg: float = 4.0,
        verify: bool = True,
        home_on_empty: bool = True,
    ):
        self.cfg = cfg
        self._sleep = sleep
        self._clock = clock
        self.rate_hz = rate_hz
        self.settle_s = settle_s
        self.verify_tol_deg = verify_tol_deg
        self.verify = verify
        # False (panel --no-home, kamera na ramieniu): po pustym chwycie tylko otworz
        # chwytak i przerwij ruch, bez powrotu do HOME (HOME_POSE uderzylby w kamere).
        self.home_on_empty = home_on_empty
        self._last_cmd: dict | None = None
        self._home_pose: dict = dict(HOME_POSE)

        if arm is None:
            import arm_control  # lazy: ciagnie lerobot, ktorego nie ma na laptopie
            self._home_pose = dict(arm_control.HOME_POSE)
            arm = arm_control.make_arm(port=cfg.arm.port, arm_id=cfg.arm.arm_id, max_relative_target=None)
            arm.connect(calibrate=False)
            self._owns_arm = True
        else:
            self._owns_arm = False
        self.arm = arm

    # --- odczyt / zapis ----------------------------------------------------

    def _read_pose(self, retries: int = 4, retry_delay: float = 0.3) -> dict | None:
        """Jak arm_control.read_joint_positions, ale z wstrzykiwanym sleep i bez wyjatku."""
        last_exc = None
        for attempt in range(retries):
            try:
                obs = self.arm.get_observation()
                pose = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES if f"{j}.pos" in obs}
                if pose:
                    return pose
                return None
            except Exception as exc:  # noqa: BLE001 - magistrala Feetech gubi pakiety
                last_exc = exc
                if attempt < retries - 1:
                    self._sleep(retry_delay)
        log.warning("odczyt pozycji nieudany po %d probach: %s", retries, last_exc)
        return None

    def _send(self, pose: dict) -> None:
        self.arm.send_action({f"{j}.pos": float(v) for j, v in pose.items()})
        # scal z poprzednia komenda: po samym otwarciu chwytaka ({"gripper": 100}) reszta
        # przegubow musi zostac znana, inaczej przy nieudanym odczycie _move rusza tylko chwytakiem
        self._last_cmd = {**(self._last_cmd or {}), **{j: float(v) for j, v in pose.items()}}

    def _current_pose(self) -> dict:
        pose = self._read_pose()
        if pose is not None and all(j in pose for j in JOINT_NAMES):
            return pose
        if self._last_cmd is not None:
            log.warning("brak odczytu pozycji, interpoluje od ostatniej komendy")
            return dict(self._last_cmd)
        raise RuntimeError("nie da sie odczytac pozycji ramienia i nie ma ostatniej komendy")

    def _move(self, target: dict, seconds: float) -> None:
        """Liniowa interpolacja od biezacej pozycji do target w `seconds`."""
        start = self._current_pose()
        joints = [j for j in target if j in start]
        steps = max(1, int(round(seconds * self.rate_hz))) if seconds > 0 else 1
        period = seconds / steps
        t0 = self._clock()
        for i in range(1, steps + 1):
            frac = i / steps
            self._send({j: start[j] + (target[j] - start[j]) * frac for j in joints})
            # spij do planowanego czasu i-tego kroku (odporne na wolny send_action);
            # takze po ostatnim kroku, zeby caly dojazd trwal dokladnie `seconds`
            wait = t0 + i * period - self._clock()
            if wait > 0:
                self._sleep(wait)

    def _verify(self, wp: Waypoint, skip_gripper: bool) -> dict | None:
        pose = self._read_pose()
        if pose is None:
            return None
        if not self.verify:
            return pose
        off = {
            j: pose[j] - wp.pose[j]
            for j in JOINT_NAMES
            if j in pose and not (skip_gripper and j == "gripper")
        }
        bad = {j: round(d, 1) for j, d in off.items() if abs(d) >= self.verify_tol_deg}
        if bad:
            log.warning("waypoint '%s' nie osiagniety (tol %.1f): odchylki %s", wp.label, self.verify_tol_deg, bad)
        return pose

    # --- interfejs ---------------------------------------------------------

    def replay(self, name: str) -> bool | None:
        motion = load_motion(self.cfg.arm.motions_dir, name)
        result: bool | None = None
        log.info("replay '%s': %d waypointow, %.1f s", motion.name, len(motion.waypoints), motion.total_seconds)
        for wp in motion.waypoints:
            log.info("  -> %s (%.1f s)", wp.label, wp.seconds)
            self._move(wp.pose, wp.seconds)
            if self.settle_s > 0:
                self._sleep(self.settle_s)
            pose = self._verify(wp, skip_gripper=wp.check_gripper)
            if not wp.check_gripper:
                continue
            if pose is None or "gripper" not in pose:
                log.warning("check_gripper: brak odczytu chwytaka, wynik nieznany")
                continue
            reading = pose["gripper"]
            if reading < self.cfg.arm.empty_gripper_below:
                # PROGRESS.md: odczyt ~2 po zamknieciu = chwytak pusty
                log.info("chwytak PUSTY (odczyt %.1f < %.1f) - przerywam, otwieram, wracam do home",
                         reading, self.cfg.arm.empty_gripper_below)
                self._move({"gripper": GRIPPER_OPEN}, 0.8)
                # ramie nie moze zostac wyciagniete przy ziemi: zaslania kamere i szoruje przy cofaniu
                if self.home_on_empty:
                    self.home()
                else:
                    log.info("home_on_empty=False: zostaje w miejscu")
                return False
            log.info("chwytak trzyma (odczyt %.1f)", reading)
            result = True
        return result

    def has_motion(self, name: str) -> bool:
        return os.path.exists(motion_path(self.cfg.arm.motions_dir, name))

    def home(self) -> None:
        if os.path.exists(motion_path(self.cfg.arm.motions_dir, "home")):
            self.replay("home")
            return
        self._move(dict(self._home_pose), 2.0)
        if self.settle_s > 0:
            self._sleep(self.settle_s)

    def close(self) -> None:
        if self._owns_arm:
            try:
                self.arm.disconnect()
            except Exception as exc:  # noqa: BLE001
                log.warning("disconnect nieudany (pewnie juz rozlaczone): %s", exc)


# ---------------------------------------------------------------------------
# subprocess (skrypt zespolu bez zmian)
# ---------------------------------------------------------------------------

class SubprocessArm:
    """Odpala cfg.arm.subprocess_cmd z {port} i {name}. Nie wie nic o chwytaku (None)."""

    def __init__(self, cfg, home_cmd: str = "python arm_control.py --port {port} home", run=subprocess.run):
        self.cfg = cfg
        self.home_cmd = home_cmd
        self._run = run

    def _exec(self, template: str, name: str) -> int:
        cmd = template.format(port=self.cfg.arm.port, name=name)
        log.info("subprocess: %s", cmd)
        proc = self._run(cmd, shell=True, cwd=REPO_ROOT, check=False)
        code = getattr(proc, "returncode", 0)
        if code != 0:
            log.warning("komenda zakonczona kodem %s: %s", code, cmd)
        return code

    def replay(self, name: str) -> bool | None:
        self._exec(self.cfg.arm.subprocess_cmd, name)
        return None

    def home(self) -> None:
        self._exec(self.home_cmd, "home")

    def close(self) -> None:
        pass


# ---------------------------------------------------------------------------
# fabryka
# ---------------------------------------------------------------------------

def make_arm(cfg, **kw) -> Arm:
    driver = cfg.arm.driver
    if driver == "sim":
        return SimArm(cfg, **kw)
    if driver == "waypoints":
        return WaypointArm(cfg, **kw)
    if driver == "subprocess":
        return SubprocessArm(cfg, **kw)
    raise ValueError(f"nieznany cfg.arm.driver: {driver!r} (sim | waypoints | subprocess)")
