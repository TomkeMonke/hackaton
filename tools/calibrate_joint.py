"""Kalibracja stawu ramienia SO-101 (domyslnie shoulder_lift; --joint all = wszystkie), bez `lerobot calibrate`.

`lerobot calibrate` nadpisuje wszystkie stawy (w tym reczne poprawki barku, docs/HARDWARE.md
pulapki 1 i 3). To narzedzie rusza tylko wybrany staw: Homing_Offset + limity pozycji w EEPROM
serwa i jego wpis w so101.json. Reszta pliku zostaje bez zmian, przed zapisem robiona jest kopia.

Uzycie (na Pi, z katalogu repo; nic innego nie moze trzymac portu - zatrzymaj tools/arm_web.py):
    python tools/calibrate_joint.py                    # tylko odczyt: rejestry serwa vs plik
    python tools/calibrate_joint.py --record 20        # torque OFF na stawie, 20 s: przeprowadz staw
                                                       # recznie przez CALY zakres; wypisuje propozycje
    python tools/calibrate_joint.py --record 20 --write   # j.w. + zapis (pyta o potwierdzenie "TAK")
    python tools/calibrate_joint.py --joint all --record 30 --write
                                                       # wszystkie 6 stawow naraz, jedno "TAK"

UWAGA: --record zdejmuje torque ze stawu - ramie opadnie w tym stawie. Trzymaj je.
Przy --joint all torque schodzi ze WSZYSTKICH stawow: cale ramie jest wiotkie.
Staw, ktory prawie sie nie ruszal (albo zrobil pelny obrot), jest pomijany i nie jest zapisywany.

Model Feetech STS3215 (tak liczy lerobot): Present = Actual - Homing_Offset (mod 4096).
Nowy offset ustawia srodek nagranego zakresu na 2047, wiec zakres jest daleko od zera enkodera
(przejscie przez zero = serwo jedzie "naokolo", pulapka 3). Homing_Offset ma zakres +-2047.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time

RESOLUTION = 4096
CENTER = 2047
MAX_OFFSET = 2047
# id jak w lerobot so_follower.py (pole "id" w JSON jest ignorowane, pulapka 2)
MOTOR_IDS = {"shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3, "wrist_flex": 4, "wrist_roll": 5, "gripper": 6}
CALIB_PATH = os.path.expanduser("~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json")
MIN_SPAN = 200  # ponizej tego staw prawie nie byl ruszany - to nie jest zakres


def unwrap(samples: list) -> list:
    """Surowe Present (0..4095) -> ciagla sekwencja (przejscie 4095->0 nie robi skoku)."""
    out: list = []
    for raw in samples:
        if not out:
            out.append(float(raw))
            continue
        delta = (raw - out[-1]) % RESOLUTION
        if delta > RESOLUTION / 2:
            delta -= RESOLUTION
        out.append(out[-1] + delta)
    return out


def wrap_offset(value: float) -> int:
    """Offset do zakresu -2047..2047 (modulo 4096)."""
    v = int(round(value)) % RESOLUTION
    if v > MAX_OFFSET:
        v -= RESOLUTION
    return v


def propose(present_samples: list, old_offset: int) -> dict:
    """Z surowych Present nagranych przy starym offsecie liczy nowy offset i limity.

    Actual = Present + old_offset. Nowy offset H taki, ze srodek zakresu Actual - H = 2047.
    """
    if not present_samples:
        raise ValueError("brak probek")
    cont = unwrap(present_samples)
    lo_a, hi_a = min(cont) + old_offset, max(cont) + old_offset
    span = hi_a - lo_a
    if span < MIN_SPAN:
        raise ValueError(f"zakres tylko {span:.0f} krokow - staw prawie sie nie ruszal")
    if span >= RESOLUTION - 1:
        raise ValueError("zakres >= pelny obrot - nagranie niewiarygodne")
    new_offset = wrap_offset((lo_a + hi_a) / 2 - CENTER)
    # wrap_offset zmienia offset o wielokrotnosc 4096 - przesun przedzial tak,
    # zeby jego srodek w nowych jednostkach Present wypadl na 2047
    lo_p = lo_a - new_offset
    lo_p -= round(((lo_p + span / 2) - CENTER) / RESOLUTION) * RESOLUTION
    return {
        "homing_offset": new_offset,
        "range_min": int(round(lo_p)),
        "range_max": int(round(lo_p + span)),
        "span": int(round(span)),
    }


def deg_half_range(cal: dict) -> float:
    return (cal["range_max"] - cal["range_min"]) / 2 * 360.0 / (RESOLUTION - 1)


def present_to_deg(present: float, cal: dict) -> float:
    """Tak jak lerobot (DEGREES): (raw - srodek zakresu) * 360 / 4095."""
    return (present - (cal["range_min"] + cal["range_max"]) / 2) * 360.0 / (RESOLUTION - 1)


def load_calib(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def propose_many(samples: dict, offsets: dict) -> tuple:
    """propose() dla kilku stawow. Zwraca (propozycje, bledy); staw z bledem nie ma propozycji."""
    proposals: dict = {}
    errors: dict = {}
    for joint, present_samples in samples.items():
        try:
            proposals[joint] = propose(present_samples, offsets[joint])
        except ValueError as exc:
            errors[joint] = str(exc)
    return proposals, errors


def save_joints(path: str, news: dict) -> str:
    """Zapisuje wpisy podanych stawow (reszta pliku bez zmian), jedna kopia zapasowa. Zwraca jej sciezke."""
    calib = load_calib(path)
    backup = f"{path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
    shutil.copy2(path, backup)
    for joint, new in news.items():
        entry = dict(calib[joint])
        entry.update({k: new[k] for k in ("homing_offset", "range_min", "range_max")})
        calib[joint] = entry
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(calib, fh, indent=4)
    return backup


def save_joint(path: str, joint: str, new: dict) -> str:
    """Zapisuje wpis jednego stawu (reszta pliku bez zmian). Zwraca sciezke kopii zapasowej."""
    return save_joints(path, {joint: new})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", default="shoulder_lift", choices=list(MOTOR_IDS) + ["all"],
                        help="staw albo 'all' (wszystkie naraz)")
    parser.add_argument("--port", default=os.environ.get("ROBOT_ARM_PORT", "/dev/robot-arm"))
    parser.add_argument("--calib", default=CALIB_PATH)
    parser.add_argument("--record", type=float, default=0.0, metavar="S",
                        help="nagraj zakres przez S sekund (torque OFF na stawie)")
    parser.add_argument("--write", action="store_true", help="po nagraniu zapisz EEPROM + JSON (pyta)")
    args = parser.parse_args()
    if args.write and args.record <= 0:
        parser.error("--write wymaga --record")

    from lerobot.motors import Motor, MotorNormMode  # lazy: lerobot tylko na Pi
    from lerobot.motors.feetech import FeetechMotorsBus

    joints = list(MOTOR_IDS) if args.joint == "all" else [args.joint]
    calib = load_calib(args.calib)
    bus = FeetechMotorsBus(args.port, {j: Motor(MOTOR_IDS[j], "sts3215", MotorNormMode.DEGREES) for j in joints})
    bus.connect()
    try:
        def reg(name: str, joint: str) -> int:
            return int(bus.read(name, joint, normalize=False))

        offsets = {}
        for joint in joints:
            file_cal = calib[joint]
            offset = reg("Homing_Offset", joint)
            present = reg("Present_Position", joint)
            lim = (reg("Min_Position_Limit", joint), reg("Max_Position_Limit", joint))
            offsets[joint] = offset
            print(f"{joint} (id {MOTOR_IDS[joint]})")
            print(f"  serwo: Homing_Offset {offset}, limity {lim[0]}..{lim[1]}, Present {present}")
            print(f"  plik:  homing_offset {file_cal['homing_offset']}, zakres {file_cal['range_min']}..{file_cal['range_max']}"
                  f" (+-{deg_half_range(file_cal):.1f} st)")
            print(f"  kat wg pliku: {present_to_deg(present, file_cal):.1f} st")
            if offset != file_cal["homing_offset"] or lim != (file_cal["range_min"], file_cal["range_max"]):
                print("  !! rejestry serwa NIE zgadzaja sie z plikiem - lerobot liczy katy z pliku")
        if args.record <= 0:
            return 0

        print(f"\nTORQUE OFF na: {', '.join(joints)} - TRZYMAJ RAMIE. Przez {args.record:.0f} s przeprowadz"
              " powoli kazdy staw przez CALY zakres (oba konce).")
        bus.disable_torque(joints)
        samples: dict = {j: [] for j in joints}
        t_end = time.monotonic() + args.record
        next_note = time.monotonic() + 5.0
        while time.monotonic() < t_end:
            for joint in joints:
                try:
                    samples[joint].append(reg("Present_Position", joint))
                except Exception as exc:  # noqa: BLE001 - pojedynczy zgubiony pakiet nie przerywa nagrania
                    print(f"  (pominieto odczyt {joint}: {exc})")
            if time.monotonic() >= next_note:
                print(f"  zostalo {max(0.0, t_end - time.monotonic()):.0f} s")
                next_note += 5.0
            time.sleep(0.05)

        proposals, errors = propose_many(samples, offsets)
        print()
        for joint in joints:
            if joint in errors:
                print(f"{joint}: POMINIETY - {errors[joint]}")
                continue
            new = proposals[joint]
            new_cal = {**calib[joint], **new}
            now_present = samples[joint][-1] + offsets[joint] - new["homing_offset"]
            print(f"{joint}: nagrano {len(samples[joint])} probek, zakres {new['span']} krokow"
                  f" (+-{deg_half_range(new_cal):.1f} st)")
            print(f"  PROPOZYCJA: Homing_Offset {new['homing_offset']}, limity {new['range_min']}..{new['range_max']},"
                  f" staw teraz bylby na {present_to_deg(now_present, new_cal):.1f} st")
        if not proposals:
            print("  nic do zapisania")
            return 1
        if not args.write:
            print("  (bez --write nic nie zapisano)")
            return 0
        if input(f"Zapisac {', '.join(proposals)} do EEPROM serw i do pliku kalibracji? wpisz TAK: ").strip() != "TAK":
            print("  nie zapisano")
            return 0
        written: dict = {}
        for joint, new in proposals.items():
            bus.write("Homing_Offset", joint, new["homing_offset"], normalize=False)
            bus.write("Min_Position_Limit", joint, new["range_min"], normalize=False)
            bus.write("Max_Position_Limit", joint, new["range_max"], normalize=False)
            back = (reg("Homing_Offset", joint), reg("Min_Position_Limit", joint), reg("Max_Position_Limit", joint))
            print(f"  {joint} po zapisie: offset {back[0]}, limity {back[1]}..{back[2]}")
            if back != (new["homing_offset"], new["range_min"], new["range_max"]):
                print(f"  !! {joint}: odczyt po zapisie inny niz zapisany - jego wpis w pliku NIE zmieniony")
                continue
            written[joint] = new
        if written:
            backup = save_joints(args.calib, written)
            print(f"  plik zapisany ({', '.join(written)}): {args.calib} (kopia: {backup})")
            print("  Po zmianie kalibracji spisz na nowo HOME_POSE i sprawdz motions/ (pulapka 12).")
        return 0 if len(written) == len(proposals) else 1
    finally:
        bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    sys.exit(main())
