"""
Logika panelu recznego sterowania ramieniem SO-101 (serwer: tools/arm_web.py).

Tu jest wszystko, co da sie sprawdzic bez sprzetu i bez lerobot:
    - kolejka komend: JEDNA komenda naraz, wykonywana w watku roboczym,
    - przyciecie celu do zakresu z kalibracji serw (limits_from_calibration),
    - wlasne ograniczenie kroku na tick (max_step), bo ramie jedzie z
      max_relative_target=None (docs/HARDWARE.md, pulapka 10),
    - STOP: czysci kolejke i przerywa biezacy ruch miedzy tickami,
    - odczyt pozycji tylko gdy ramie stoi, nie czesciej niz read_period_s.

Komendy (slowniki, jak przychodza z przegladarki):
    {"cmd": "jog", "joint": "elbow_flex", "step": -5}
    {"cmd": "home"}
    {"cmd": "open"} / {"cmd": "close"}
    {"cmd": "motion", "name": "grasp_mid"}
    {"cmd": "stop"}          (nie idzie do kolejki, dziala od razu)
    {"cmd": "jog_xyz", "axis": "z", "step_mm": 10}
        TCP (punkt miedzy szczekami) o step_mm wzdluz osi bazy ramienia, pochylenie chwytaka
        bez zmian (pinecone_bot/kinematics.py). Wymaga kinematyki (URDF) w panelu.
    {"cmd": "urdf_zero"}     (od razu, bez kolejki) ramie stoi wyprostowane poziomo do przodu:
        odczyt pan/lift/elbow/wrist_flex staje sie zerem URDF, offsety ida do pinecone_config.json.

Nagrywanie ruchu z panelu (od razu, bez kolejki; szkic trzymany w panelu):
    {"cmd": "add_point", "label": "nad szyszka", "seconds": 1.5, "check_gripper": false}
        dodaje BIEZACA poze: przeguby z odczytu serw (lub ostatniej komendy, gdy
        odczytu brak), chwytak z ostatniej komendy (po "close" = 0, czyli zacisk;
        odczyt zamknietego chwytaka to szerokosc szyszki, nie cel).
    {"cmd": "drop_point"} / {"cmd": "clear_points"}
    {"cmd": "save_motion", "name": "grasp_cam", "note": "...", "overwrite": false}
        zapisuje szkic do motions/<name>.json (format jak tools/record_waypoints.py).

Przed pierwszym udanym HOME przyjmowane sa tylko "home" i "stop"
(serwer po starcie sam wrzuca HOME do kolejki).

manual_only=True (tools/arm_web.py --no-home, np. gdy na ramieniu siedzi kamera):
bez HOME przy starcie, "home" odrzucane, jog i chwytak od razu, liczone od
odczytanej pozycji. Ruchy z motions/ sa dozwolone (nagrywa sie je z panelu pod
biezacy montaz), ale przy pustym chwycie ramie NIE wraca do HOME (home_on_empty=False).
UWAGA: stare ruchy (grasp_mid, home, drop_box) koncza w HOME_POSE - w tym trybie
odtwarzaj tylko ruchy nagrane pod aktualny montaz.

Jog liczy cel od OSTATNIEJ WYSLANEJ komendy (setpoint), a nie od odczytu -
dzieki temu komenda nie robi sync_read. HOME i ruchy z motions/ odtwarza
WaypointArm (pinecone_bot/arm.py) z tym samym ramieniem.
"""
from __future__ import annotations

import collections
import logging
import os
import re
import threading
import time
from typing import Callable

from .arm import (
    GRIPPER_OPEN,
    HOME_POSE,
    JOINT_NAMES,
    Motion,
    Waypoint,
    WaypointArm,
    motion_path,
    resolve_motions_dir,
    save_motion,
)

log = logging.getLogger(__name__)

GRIPPER_CLOSED = 0.0          # arm_control.close_gripper: 0 = zamkniety
STEP_CHOICES = (1.0, 5.0, 10.0)
XYZ_STEPS_MM = (5.0, 10.0, 20.0)
XYZ_AXES = {"x": 0, "y": 1, "z": 2}
MAX_QUEUE = 10                # wiecej oczekujacych komend = klikanie na oslep, odrzucamy
# Staw dalej niz tyle poza zakresem kalibracji = jog zablokowany. Serwo i tak utnie cel do
# swojego limitu pozycji (EEPROM), wiec "ruch o 1 st" stalby sie skokiem do granicy zakresu.
OUT_OF_RANGE_TOL = 1.0
RECOVER_TOL = 10.0            # do tylu st za granica jog w strone zakresu jest dozwolony
MAX_DRAFT = 40                # waypointow w szkicu ruchu nagrywanego z panelu
MOTION_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
RAW_RESOLUTION = 4095         # STS3215: 4096 krokow, lerobot dzieli przez (4096 - 1)

# Maksymalna zmiana celu na jeden tick (stopnie; gripper w jednostkach 0-100).
# Przy rate_hz=25: 25 st/s dla przegubow, 50 j/s dla chwytaka (polowa wartosci z 2026-09-26 rano).
DEFAULT_MAX_STEP = {j: 1.0 for j in JOINT_NAMES}
DEFAULT_MAX_STEP["gripper"] = 2.0


class Stopped(Exception):
    """Biezaca komenda przerwana przez STOP."""


# ---------------------------------------------------------------------------
# zakres z kalibracji
# ---------------------------------------------------------------------------

def _field(obj, name):
    return obj[name] if isinstance(obj, dict) else getattr(obj, name)


def limits_from_calibration(calibration: dict, norm_modes: dict) -> dict:
    """{joint: (lo, hi)} w jednostkach lerobot, liczone tak jak lerobot normalizuje.

    calibration: {joint: MotorCalibration albo dict z range_min/range_max} (arm.calibration)
    norm_modes:  {joint: "DEGREES" | "RANGE_M100_100" | "RANGE_0_100"} (arm.bus.motors[j].norm_mode.name)

    DEGREES: deg = (raw - srodek) * 360 / 4095, wiec zakres jest symetryczny wokol 0.
    """
    limits = {}
    for joint in JOINT_NAMES:
        if joint not in calibration:
            raise ValueError(f"brak przegubu '{joint}' w kalibracji serw")
        lo_raw = float(_field(calibration[joint], "range_min"))
        hi_raw = float(_field(calibration[joint], "range_max"))
        if hi_raw <= lo_raw:
            raise ValueError(f"zla kalibracja '{joint}': range_min {lo_raw} >= range_max {hi_raw}")
        mode = str(norm_modes.get(joint, ""))
        if mode == "DEGREES":
            half = (hi_raw - lo_raw) / 2 * 360.0 / RAW_RESOLUTION
            limits[joint] = (-half, half)
        elif mode == "RANGE_M100_100":
            limits[joint] = (-100.0, 100.0)
        elif mode == "RANGE_0_100":
            limits[joint] = (0.0, 100.0)
        else:
            raise ValueError(f"nieznany tryb normalizacji '{mode}' dla '{joint}'")
    return limits


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def jog_target(current: float, step: float, lo: float, hi: float) -> float:
    """current + step przyciete do [lo, hi], ale nigdy w strone przeciwna do kroku.

    (Jesli przegub stoi juz poza zakresem, "+" nie moze go szarpnac w dol do hi.)
    """
    target = clamp(current + step, lo, hi)
    if step > 0 and target < current:
        return current
    if step < 0 and target > current:
        return current
    return target


def list_motions(motions_dir: str) -> list:
    path = resolve_motions_dir(motions_dir)
    if not os.path.isdir(path):
        return []
    return sorted(f[:-5] for f in os.listdir(path) if f.endswith(".json"))


# ---------------------------------------------------------------------------
# ramie z zapamietana ostatnia komenda
# ---------------------------------------------------------------------------

class RecordingArm:
    """Przepuszcza wywolania do ramienia i pamieta ostatnio WYSLANY cel kazdego przegubu.

    WaypointArm dostaje to zamiast golego ramienia, wiec po przerwanym ruchu z
    motions/ panel wie, dokad serwa faktycznie jada (ostatni sync_write).
    """

    def __init__(self, arm):
        self.arm = arm
        self.last_sent: dict = {}

    def send_action(self, action: dict):
        result = self.arm.send_action(action)
        for key, val in action.items():
            if key.endswith(".pos"):
                self.last_sent[key[:-4]] = float(val)
        return result

    def get_observation(self):
        return self.arm.get_observation()

    def __getattr__(self, name):
        return getattr(self.arm, name)


# ---------------------------------------------------------------------------
# panel
# ---------------------------------------------------------------------------

class _Cmd:
    __slots__ = ("data", "gen")

    def __init__(self, data: dict, gen: int):
        self.data = data
        self.gen = gen


class ArmPanel:
    def __init__(
        self,
        arm,
        cfg,
        limits: dict,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
        rate_hz: float = 25.0,
        max_step: dict | None = None,
        read_period_s: float = 0.5,
        manual_only: bool = False,
        kinematics=None,
        config_path: str | None = None,
    ):
        self.cfg = cfg
        self.kin = kinematics          # So101Kinematics albo None (jog XYZ wylaczony)
        self.config_path = config_path  # gdzie urdf_zero zapisuje offsety (None = domyslny)
        self.limits = dict(limits)
        self.rate_hz = rate_hz
        self.max_step = dict(max_step or DEFAULT_MAX_STEP)
        self.read_period_s = read_period_s
        self._raw_sleep = sleep
        self._clock = clock

        self.arm = RecordingArm(arm)
        # WaypointArm spi przez _sleep, wiec STOP przerywa tez odtwarzanie motions/.
        # manual_only: po pustym chwycie bez powrotu do HOME (kamera na ramieniu).
        self.waypoints = WaypointArm(
            cfg, arm=self.arm, sleep=self._sleep, clock=clock, home_on_empty=not manual_only
        )
        self.draft: list = []      # Waypoint-y nagrywane z panelu (add_point), do save_motion

        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._queue: collections.deque = collections.deque()
        self._gen = 0              # zwiekszane przez STOP; komenda ze starsza generacja = przerwana
        self._running: _Cmd | None = None
        self._current_gen = 0
        self.manual_only = manual_only
        self.homed = manual_only   # manual_only: brak bramki HOME, jog od odczytu
        self.positions: dict = {}  # ostatni odczyt z serw
        self.last_read_t: float | None = None
        self.last_error: str | None = None
        self.last_result: str = ""
        self._last_read_attempt = -1e9
        self._thread: threading.Thread | None = None
        self._quit = False

    # --- wejscie z przegladarki -------------------------------------------

    def submit(self, data: dict) -> tuple:
        """Zwraca (ok, komunikat). Sprawdza komende i wrzuca do kolejki."""
        cmd = str(data.get("cmd", ""))
        if cmd == "stop":
            self.stop()
            return True, "STOP"
        if cmd in ("add_point", "drop_point", "clear_points", "save_motion"):
            return self._draft_cmd(cmd, data)
        if cmd == "urdf_zero":
            return self._urdf_zero()
        if cmd not in ("jog", "jog_xyz", "home", "open", "close", "motion"):
            return False, f"nieznana komenda '{cmd}'"
        if cmd == "jog_xyz":
            if self.kin is None:
                return False, "jog XYZ wylaczony (brak kinematyki/URDF)"
            axis = data.get("axis")
            if axis not in XYZ_AXES:
                return False, f"os musi byc jedna z {tuple(XYZ_AXES)}"
            try:
                step_mm = float(data.get("step_mm"))
            except (TypeError, ValueError):
                return False, "zly krok"
            if abs(step_mm) not in XYZ_STEPS_MM:
                return False, f"krok musi byc jednym z {XYZ_STEPS_MM} mm"
            data = {"cmd": "jog_xyz", "axis": axis, "step_mm": step_mm}
        if cmd == "jog":
            joint = data.get("joint")
            if joint not in JOINT_NAMES:
                return False, f"nieznany przegub '{joint}'"
            try:
                step = float(data.get("step"))
            except (TypeError, ValueError):
                return False, "zly krok"
            if abs(step) not in STEP_CHOICES:
                return False, f"krok musi byc jednym z {STEP_CHOICES}"
            data = {"cmd": "jog", "joint": joint, "step": step}
        if cmd == "motion":
            name = data.get("name")
            if name not in list_motions(self.cfg.arm.motions_dir):
                return False, f"brak ruchu '{name}' w motions/"
            data = {"cmd": "motion", "name": name}
        if self.manual_only and cmd == "home":
            return False, "HOME wylaczone (--no-home)"
        with self._lock:
            if not self.homed and cmd != "home":
                return False, "najpierw HOME"
            if len(self._queue) >= MAX_QUEUE:
                return False, "kolejka pelna"
            self._queue.append(_Cmd(dict(data), self._gen))
            self._wake.notify()
        return True, "w kolejce"

    def stop(self) -> None:
        with self._lock:
            self._gen += 1
            self._queue.clear()
            self._wake.notify()
        log.info("STOP")

    # --- kinematyka (jog XYZ) ----------------------------------------------

    def _urdf_zero(self) -> tuple:
        """Biezacy ODCZYT pan/lift/elbow/wrist_flex = zero URDF. Offsety do configu i do kinematyki."""
        from .kinematics import IK_JOINTS

        if self.kin is None:
            return False, "brak kinematyki/URDF"
        with self._lock:
            read = dict(self.positions)
            busy = self._running is not None or bool(self._queue)
        if busy:
            return False, "ramie w ruchu - poczekaj"
        missing = [j for j in IK_JOINTS if j not in read]
        if missing:
            return False, f"brak odczytu {missing} - poczekaj na odczyt"
        offsets = dict(self.cfg.arm.urdf_offset_deg)
        for joint in IK_JOINTS:
            offsets[joint] = round(-self.kin.sign[joint] * read[joint], 2)
        self.cfg.arm.urdf_offset_deg = offsets
        self.kin.offset_deg.update(offsets)
        try:
            path = self.cfg.save(self.config_path)
        except OSError as exc:
            return False, f"offsety ustawione, ale zapis nieudany: {exc}"
        log.info("zero URDF: %s -> %s", offsets, path)
        return True, "zero URDF zapisane: " + ", ".join(f"{j} {v:+.1f}" for j, v in offsets.items() if j in IK_JOINTS)

    def tcp(self) -> dict | None:
        """Pozycja TCP (mm) i pochylenie chwytaka (st) z odczytu serw, brak odczytu -> z komendy."""
        if self.kin is None:
            return None
        from .kinematics import ARM_JOINTS

        with self._lock:
            pose = {**self.arm.last_sent, **self.positions}
        if any(j not in pose for j in ARM_JOINTS):
            return None
        xyz, pitch = self.kin.tcp(pose)
        return {"x": round(xyz[0] * 1000, 1), "y": round(xyz[1] * 1000, 1), "z": round(xyz[2] * 1000, 1),
                "pitch": round(pitch, 1)}

    # --- nagrywanie ruchu z panelu -------------------------------------------

    def current_pose(self) -> dict:
        """Poza do zapisu: przeguby z odczytu (brak -> ostatnia komenda), chwytak z komendy.

        Odczyt zamknietego chwytaka to szerokosc trzymanej szyszki, a nie cel zacisku;
        ostatnia komenda ("close" = 0) jest tym, co ruch ma potem powtorzyc.
        """
        sent = dict(self.arm.last_sent)
        with self._lock:
            read = dict(self.positions)
        pose = {}
        for joint in JOINT_NAMES:
            if joint == "gripper":
                val = sent.get(joint, read.get(joint))
            else:
                val = read.get(joint, sent.get(joint))
            if val is None:
                raise RuntimeError(f"nie znam pozycji {joint} - poczekaj na odczyt albo rusz stawem")
            pose[joint] = float(val)
        return pose

    def _draft_cmd(self, cmd: str, data: dict) -> tuple:
        if cmd == "add_point":
            if len(self.draft) >= MAX_DRAFT:
                return False, f"szkic ma juz {MAX_DRAFT} punktow"
            try:
                pose = self.current_pose()
                seconds = float(data.get("seconds", 1.5))
            except (RuntimeError, TypeError, ValueError) as exc:
                return False, str(exc)
            if not 0.0 <= seconds <= 30.0:
                return False, "czas dojazdu 0..30 s"
            label = str(data.get("label") or f"wp{len(self.draft)}")[:40]
            wp = Waypoint(label=label, pose=pose, seconds=seconds, check_gripper=bool(data.get("check_gripper")))
            self.draft.append(wp)
            return True, f"punkt {len(self.draft)}: {label}"
        if cmd == "drop_point":
            if not self.draft:
                return False, "szkic pusty"
            wp = self.draft.pop()
            return True, f"usunieto '{wp.label}'"
        if cmd == "clear_points":
            self.draft = []
            return True, "szkic wyczyszczony"
        # save_motion
        name = str(data.get("name") or "")
        if not MOTION_NAME_RE.match(name):
            return False, "nazwa: litery, cyfry, '-' i '_' (max 40)"
        if not self.draft:
            return False, "szkic pusty - najpierw dodaj punkty"
        path = motion_path(self.cfg.arm.motions_dir, name)
        if os.path.exists(path) and not data.get("overwrite"):
            return False, f"ruch '{name}' juz istnieje (zaznacz nadpisanie)"
        motion = Motion(name=name, waypoints=list(self.draft), note=str(data.get("note") or "")[:300])
        try:
            save_motion(self.cfg.arm.motions_dir, motion)
        except OSError as exc:
            return False, f"zapis nieudany: {exc}"
        self.draft = []
        log.info("zapisano ruch %s (%d punktow)", path, len(motion.waypoints))
        return True, f"zapisano motions/{name}.json ({len(motion.waypoints)} punktow)"

    # --- stan dla przegladarki --------------------------------------------

    def snapshot(self) -> dict:
        sent = dict(self.arm.last_sent)  # kopia: watek roboczy dopisuje bez blokady
        draft = [
            {"label": wp.label, "seconds": wp.seconds, "check_gripper": wp.check_gripper,
             "pose": {j: round(v, 1) for j, v in wp.pose.items()}}
            for wp in list(self.draft)
        ]
        tcp = self.tcp()  # przed blokada: tcp() sam bierze _lock
        with self._lock:
            running = self._running.data if self._running else None
            return {
                "draft": draft,
                "positions": {j: round(v, 2) for j, v in self.positions.items()},
                "setpoint": {j: round(v, 2) for j, v in sent.items()},
                "limits": {j: [round(lo, 1), round(hi, 1)] for j, (lo, hi) in self.limits.items()},
                "busy": running,
                "queue": len(self._queue),
                "homed": self.homed,
                "manual_only": self.manual_only,
                "error": self.last_error,
                "result": self.last_result,
                "read_age_s": None if self.last_read_t is None else round(self._clock() - self.last_read_t, 1),
                "motions": list_motions(self.cfg.arm.motions_dir),
                "joints": JOINT_NAMES,
                "steps": list(STEP_CHOICES),
                "xyz_steps": list(XYZ_STEPS_MM) if self.kin is not None else [],
                "tcp": tcp,
            }

    # --- watek roboczy ----------------------------------------------------

    def _sleep(self, seconds: float) -> None:
        if self._current_gen != self._gen:
            raise Stopped()
        if seconds > 0:
            self._raw_sleep(seconds)
        if self._current_gen != self._gen:
            raise Stopped()

    def process_one(self) -> bool:
        """Wykonuje jedna komende z kolejki. False = kolejka pusta."""
        with self._lock:
            if not self._queue:
                return False
            cmd = self._queue.popleft()
            self._running = cmd
            self._current_gen = cmd.gen
        try:
            if cmd.gen != self._gen:
                raise Stopped()
            self.last_result = self._execute(cmd.data)
            self.last_error = None
        except Stopped:
            self.last_result = f"przerwane: {cmd.data.get('cmd')}"
        except Exception as exc:  # noqa: BLE001 - magistrala Feetech gubi pakiety, panel ma przezyc
            log.exception("komenda %s nieudana", cmd.data)
            self.last_error = f"{cmd.data.get('cmd')}: {exc}"
        finally:
            with self._lock:
                self._running = None
            # po ruchu odczytaj pozycje przy najblizszej okazji
            self._last_read_attempt = -1e9
        return True

    def _execute(self, data: dict) -> str:
        cmd = data["cmd"]
        if cmd == "home":
            self.waypoints.home()
            self.homed = True
            return "HOME osiagniety"
        if cmd == "motion":
            result = self.waypoints.replay(data["name"])
            return f"ruch '{data['name']}' zakonczony (chwytak: {result})"
        if cmd == "open":
            self._step_to({"gripper": clamp(GRIPPER_OPEN, *self.limits["gripper"])})
            return "chwytak otwarty"
        if cmd == "close":
            self._step_to({"gripper": clamp(GRIPPER_CLOSED, *self.limits["gripper"])})
            return "chwytak zamkniety"
        if cmd == "jog":
            joint, step = data["joint"], data["step"]
            start = self._setpoint()[joint]
            lo, hi = self.limits[joint]
            for label, val in (("komenda", start), ("odczyt", self.positions.get(joint))):
                if val is None or lo - OUT_OF_RANGE_TOL <= val <= hi + OUT_OF_RANGE_TOL:
                    continue
                # Lekko za granica (np. wrist_roll z kamera ugina sie pod ciezarem o kilka st za limit):
                # jog W STRONE zakresu jest bezpieczny - cel lezy w zakresie, skok <= RECOVER_TOL + krok.
                inward = (val < lo and step > 0) or (val > hi and step < 0)
                if not (inward and min(abs(val - lo), abs(val - hi)) <= RECOVER_TOL):
                    raise RuntimeError(
                        f"{joint} poza zakresem kalibracji ({label} {val:.1f}, zakres {lo:.1f}..{hi:.1f}): "
                        f"serwo skoczyloby do granicy o {min(abs(val - lo), abs(val - hi)):.0f} st - jog w strone zakresu"
                        f" (do {RECOVER_TOL:.0f} st za granica) albo ustaw recznie"
                    )
            target = jog_target(start, step, *self.limits[joint])
            self._step_to({joint: target})
            return f"{joint}: {start:.1f} -> {target:.1f}"
        if cmd == "jog_xyz":
            from .kinematics import IK_JOINTS

            pose = self._setpoint()
            for joint in IK_JOINTS:
                lo, hi = self.limits[joint]
                for label, val in (("komenda", pose[joint]), ("odczyt", self.positions.get(joint))):
                    if val is not None and not (lo - OUT_OF_RANGE_TOL <= val <= hi + OUT_OF_RANGE_TOL):
                        raise RuntimeError(f"{joint} poza zakresem kalibracji ({label} {val:.1f}) - najpierw jog stawu")
            delta = [0.0, 0.0, 0.0]
            delta[XYZ_AXES[data["axis"]]] = data["step_mm"] / 1000.0
            target = self.kin.jog_xyz(pose, delta, self.limits)
            self._step_to(target)
            return f"TCP {data['axis']} {data['step_mm']:+.0f} mm"
        raise ValueError(f"nieznana komenda {cmd}")

    def _setpoint(self) -> dict:
        """Pelna poza startowa: ostatnio wyslane cele, braki z ostatniego odczytu."""
        pose = {**self.positions, **self.arm.last_sent}
        missing = [j for j in JOINT_NAMES if j not in pose]
        if missing:
            hint = "poczekaj na odczyt pozycji" if self.manual_only else "wcisnij HOME"
            raise RuntimeError(f"nie znam pozycji {missing} - {hint}")
        return pose

    def _step_to(self, target: dict) -> None:
        """Dojazd do celu z krokiem <= max_step na tick, rate_hz tickow/s, bez odczytow."""
        pose = self._setpoint()
        period = 1.0 / self.rate_hz
        while True:
            nxt = {}
            done = True
            for joint, goal in target.items():
                cur = pose[joint]
                step = clamp(goal - cur, -self.max_step[joint], self.max_step[joint])
                nxt[joint] = cur + step
                if abs(goal - nxt[joint]) > 1e-6:
                    done = False
            self.arm.send_action({f"{j}.pos": v for j, v in nxt.items()})
            pose.update(nxt)
            self._sleep(period)
            if done:
                return

    def maybe_read(self) -> bool:
        """Odczyt pozycji, jesli minelo read_period_s od poprzedniej proby. True = odczytano."""
        now = self._clock()
        if now - self._last_read_attempt < self.read_period_s:
            return False
        self._last_read_attempt = now
        try:
            obs = self.arm.get_observation()
        except Exception as exc:  # noqa: BLE001
            self.last_error = f"odczyt: {exc}"
            return False
        pose = {j: float(obs[f"{j}.pos"]) for j in JOINT_NAMES if f"{j}.pos" in obs}
        if pose:
            with self._lock:
                self.positions = pose
                self.last_read_t = now
        return bool(pose)

    def run_forever(self) -> None:
        while not self._quit:
            if self.process_one():
                continue
            self.maybe_read()
            with self._lock:
                if not self._queue and not self._quit:
                    self._wake.wait(timeout=self.read_period_s)

    def start(self, home_first: bool = True) -> None:
        if home_first and not self.manual_only:
            with self._lock:
                self._queue.append(_Cmd({"cmd": "home"}, self._gen))
        self._thread = threading.Thread(target=self.run_forever, name="arm-panel", daemon=True)
        self._thread.start()

    def shutdown(self, timeout: float = 3.0) -> None:
        self.stop()
        with self._lock:
            self._quit = True
            self._wake.notify()
        if self._thread is not None:
            self._thread.join(timeout)


# ---------------------------------------------------------------------------
# atrapa ramienia (testy i `tools/arm_web.py --fake` na laptopie)
# ---------------------------------------------------------------------------

# Zakres w surowych krokach enkodera podobny do SO-101 (wartosci przykladowe,
# NIE z naszej kalibracji - prawdziwa jest tylko na Pi, w ~/.cache/huggingface/...).
FAKE_CALIBRATION = {
    "shoulder_pan": {"range_min": 800, "range_max": 3300},
    "shoulder_lift": {"range_min": 1006, "range_max": 3089},
    "elbow_flex": {"range_min": 850, "range_max": 3200},
    "wrist_flex": {"range_min": 900, "range_max": 3250},
    "wrist_roll": {"range_min": 100, "range_max": 3950},
    "gripper": {"range_min": 2000, "range_max": 3400},
}
FAKE_NORM_MODES = {j: "DEGREES" for j in JOINT_NAMES}
FAKE_NORM_MODES["gripper"] = "RANGE_0_100"


class FakeSO101:
    """Serwa idealnie sledza komendy. Liczy wywolania, zeby testy mogly sprawdzic ruch magistrali."""

    def __init__(self, pose: dict | None = None):
        start = dict(HOME_POSE)
        start["shoulder_lift"] = -40.0  # nie w HOME, zeby start panelu mial co robic
        self.pose = dict(pose or start)
        self.actions: list = []
        self.reads = 0
        self.fail_reads = 0  # ile kolejnych odczytow ma rzucic wyjatek

    def connect(self, calibrate=False):
        pass

    def disconnect(self):
        pass

    def send_action(self, action: dict):
        self.actions.append(dict(action))
        for key, val in action.items():
            self.pose[key[:-4]] = float(val)
        return action

    def get_observation(self):
        self.reads += 1
        if self.fail_reads > 0:
            self.fail_reads -= 1
            raise ConnectionError("There is no status packet!")
        return {f"{j}.pos": v for j, v in self.pose.items()}
