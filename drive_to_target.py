"""
Dojazd do zapamietanej szyszki: obrot w miejscu + jazda prosto, open-loop.

Wejsciem jest plik z `scan_cones.py --json cel.json`. Kamera przestaje mierzyc
ponizej ~0.27 m, wiec ostatni odcinek i tak jedziemy BEZ sprzezenia zwrotnego -
caly manewr jest wyliczany raz, z pamieci, i odtwarzany z zegarka. Nie ma
odometrii: droga i kat wychodza wylacznie z czasu trzymania komendy.

Manewr (dwie fazy, kazda z rampa na wejsciu i wyjsciu):

    1. obrot w miejscu o `bearing_deg` (dodatni = cel w prawo)
    2. jazda prosto na `ground_distance_m` minus zapas `--standoff`

Zapas jest po to, zeby stanac PRZED szyszka, w zasiegu chwytaka, a nie na niej.

DLACZEGO OBROT W MIEJSCU, A NIE LUK
Plytka miesza kanaly: left = speed + steer, right = speed - steer. Gdy
|pwm_steer| >= |pwm_speed|, jedno kolo idzie do tylu. W jazdy po luku to blad
(platforma kreci sie w miejscu zamiast jechac), ale przy speed = 0 to dokladnie
to, czego chcemy: czysty obrot na miejscu. Dlatego faza 1 ma speed = 0, a faza 2
ma steer = 0 - zaden z tych stanow nie jest lukiem i nie da sie go pomylic.

IMPORT PAKIETU makarena
Gotowa warstwa sterowania (Link/Platform: send/hold/ramp/stop, powtarzanie
komendy, bo plytka gasi silniki po ~500 ms ciszy) lezy POZA tym repo, w
C:\\Users\\Modern 14\\makarena. Sciezka jest doklejana do sys.path w
`import_makarena()`. Inna lokalizacja: ustaw zmienna srodowiskowa MAKARENA_DIR
albo podaj --makarena-dir. Interpreter z pyserial:

    C:\\Users\\Modern 14\\makarena\\.venv\\Scripts\\python.exe

KALIBRACJA OPEN-LOOP - PROCEDURA KROK PO KROKU
Potrzebne sa dwie stale: ile metrow na sekunde jazdy i ile stopni na sekunde
obrotu, przy konkretnym speed/steer i na konkretnej nawierzchni. Domyslne
wartosci w drive_calibration.json sa ZGADNIETE ("measured": false) i sluza tylko
do przeklikania planu - na nich nie wolno jechac po cel.

    1. Wolna plaska przestrzen, min. 2 m przed platforma, wylacznik w rece.
    2. Tasma malarska na podlodze: znacznik startu przy jednym kole.
       python drive_to_target.py --calibrate --port COM9
       Skrypt jedzie 3 s prosto (z rampami), staje, potem 3 s obrotu i staje.
    3. Po fazie jazdy zmierz miarka przebyta droge tego samego kola [m].
       Wpisz do drive_calibration.json: drive.meters_per_second = droga /
       (hold_seconds + ramp_seconds) - te dwie liczby skrypt wypisuje na koncu,
       nie dziel przez surowe 3 s.
    4. Po fazie obrotu zmierz kat obrotu platformy [st]. Najprosciej: przed
       obrotem poloz tasme wzdluz platformy, po obrocie druga, i zmierz kat
       miedzy nimi (albo obroc o pelne 360 st i podziel czas przez 4 - blad
       odczytu jest wtedy najmniejszy).
       turn.degrees_per_second = kat / (hold_seconds + ramp_seconds).
    5. Ustaw "measured": true w obu sekcjach i w polu na gorze pliku.
    6. Kontrola: python drive_to_target.py --calibrate --calibration-seconds 2
       powinno dac 2/3 drogi i 2/3 kata z punktu 3-4. Jesli nie, nawierzchnia
       slizga sie na starcie - podnies --ramp-seconds i powtorz.
    7. Kalibracja jest wazna dla JEDNEJ pary speed/steer i JEDNEJ nawierzchni.
       Zmiana speed, opon, dywanu albo poziomu baterii = pomiar od nowa.

Dlaczego dzielimy przez (hold + ramp), a nie przez czas calej fazy: rampa w gore
i rampa w dol daja razem srednio jedna rampe jazdy z pelna predkoscia (trapez),
wiec droga = predkosc * (hold_seconds + ramp_seconds). Ten sam model liczy czasy
w druga strone, gdy planujemy dojazd.

UZYCIE
    python drive_to_target.py                          # plan z cel.json, zero serialu
    python drive_to_target.py --standoff 0.15          # wiekszy zapas przed szyszka
    python drive_to_target.py --target 1               # drugi cel z pliku
    python drive_to_target.py --port COM9              # PRAWDZIWA jazda
    python drive_to_target.py --port COM9 --dry-run    # port podany, ale nadal bez ruchu
    python drive_to_target.py --calibrate --port COM9  # przejazd wzorcowy do miarki

Bez --port skrypt NIGDY nie otwiera portu i nie rusza silnikow.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from contextlib import contextmanager

# --- stale -------------------------------------------------------------------

# Pakiet makarena lezy poza repo; MAKARENA_DIR / --makarena-dir to nadpisuje.
MAKARENA_DIR = os.environ.get("MAKARENA_DIR", r"C:\Users\Modern 14\makarena")

HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(HERE, "drive_calibration.json")
TARGETS_FILE = "cel.json"

# Predkosc jazdy sprawdzona w praktyce: 0.06 -> PWM 30 (figure_eight.py).
DRIVE_SPEED = 0.06
# Obrot: speed = 0, wiec kola dostaja +-pwm_steer. 0.075 * 400 = PWM 30, czyli
# tyle samo na kolo co jazda prosto - najmniejsza niespodzianka w tarciu.
TURN_STEER = 0.075

# ZGADNIETE, NIEZMIERZONE - patrz procedura w docstringu.
GUESS_DRIVE_MPS = 0.12
GUESS_TURN_DPS = 40.0

RAMP_SECONDS = 0.4  # rampa na starcie i na koncu kazdego ruchu
SETTLE_SECONDS = 0.5  # postoj miedzy fazami, zeby platforma przestala sie kiwac
CALIBRATION_SECONDS = 3.0  # dlugosc kazdej fazy przejazdu wzorcowego

STANDOFF_M = 0.12  # domyslny zapas: stajemy 12 cm przed szyszka

# left = speed + steer, right = speed - steer, wiec steer dodatni pcha lewe kolo
# w przod a prawe w tyl = obrot w PRAWO. Gdy platforma kreci sie odwrotnie, to
# jest kwestia podlaczenia silnikow - uzyj --invert-steer, nie zmieniaj tej stalej.
TURN_RIGHT_STEER_SIGN = 1.0

MIN_PHASE_SECONDS = 0.05  # krotsza faza = szum, pomijamy ja
# Ponizej tego ruch sklada sie prawie tylko z ramp i moze w ogole nie zerwac
# tarcia statycznego - liniowy model czasu przestaje tu obowiazywac.
MIN_TRUSTED_SECONDS = 0.5
MAX_TRAVEL_M = 1.5  # dalej niz to w open-loop nie ma sensu (zasieg skanu to 1 m)
MAX_TURN_DEG = 100.0  # wiekszy obrot = cel praktycznie z boku, ktos sie pomylil
BEARING_TOLERANCE_DEG = 1.0  # rozjazd bearing_deg vs atan2(lateral, forward)
DISTANCE_TOLERANCE_M = 0.01  # rozjazd ground_distance_m vs hypot(forward, lateral)

DEFAULT_CALIBRATION = {
    "_uwaga": (
        "WARTOSCI DOMYSLNE SA ZGADNIETE I NIEZMIERZONE. Zmierz je miarka "
        "(python drive_to_target.py --calibrate --port COMx), wpisz tutaj i "
        "ustaw measured na true. Do tego czasu dojazd jest tylko symulacja."
    ),
    "measured": False,
    "drive": {
        "speed": DRIVE_SPEED,
        "meters_per_second": GUESS_DRIVE_MPS,
        "measured": False,
        "_uwaga": "droga zmierzona miarka / (hold_seconds + ramp_seconds)",
    },
    "turn": {
        "steer": TURN_STEER,
        "degrees_per_second": GUESS_TURN_DPS,
        "measured": False,
        "_uwaga": "kat zmierzony / (hold_seconds + ramp_seconds), obrot w miejscu",
    },
    "ramp_seconds": RAMP_SECONDS,
    "settle_seconds": SETTLE_SECONDS,
    "surface": "niezmierzone - wpisz na czym kalibrowano (beton, dywan, parkiet)",
}


# --- import pakietu makarena -------------------------------------------------


def import_makarena(makarena_dir=MAKARENA_DIR):
    """
    Doklej sciezke do pakietu makarena i zwroc (Platform, platform_module).

    Pakiet jest poza tym repo, wiec bez tego kroku nie ma go na sys.path.
    platform_module jest potrzebny osobno, bo wirtualny zegar w dry-run podmienia
    w nim monotonic/sleep.
    """
    if not os.path.isdir(os.path.join(makarena_dir, "makarena")):
        raise SystemExit(
            f"Nie widze pakietu makarena w {makarena_dir!r}.\n"
            "Podaj --makarena-dir <sciezka do repo makarena> albo ustaw "
            "zmienna srodowiskowa MAKARENA_DIR."
        )
    if makarena_dir not in sys.path:
        sys.path.insert(0, makarena_dir)
    from makarena.platform import Platform  # noqa: PLC0415
    from makarena import platform as platform_module  # noqa: PLC0415

    return Platform, platform_module


# --- kalibracja --------------------------------------------------------------


def load_calibration(path=CALIBRATION_FILE, create=True):
    """Wczytaj plik kalibracji; przy braku zapisz domyslny (oznaczony jako zgadniety)."""
    if not os.path.exists(path):
        if create:
            save_calibration(DEFAULT_CALIBRATION, path)
            print(f"Brak {path} - zapisalem domyslny, NIEZMIERZONY.")
        return json.loads(json.dumps(DEFAULT_CALIBRATION))
    with open(path) as handle:
        data = json.load(handle)
    # Braki uzupelniamy domyslnymi, zeby stary plik nie wywalal skryptu.
    merged = json.loads(json.dumps(DEFAULT_CALIBRATION))
    for key, value in data.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key].update(value)
        else:
            merged[key] = value
    return merged


def save_calibration(data, path=CALIBRATION_FILE):
    with open(path, "w") as handle:
        json.dump(data, handle, indent=2)
        handle.write("\n")


def calibration_warnings(cal):
    """Lista ostrzezen o niezmierzonych stalych - wypisywana glosno przed jazda."""
    notes = []
    if not cal["drive"].get("measured"):
        notes.append(
            f"drive.meters_per_second = {cal['drive']['meters_per_second']} "
            "NIEZMIERZONE (zgadniete) - dystans jazdy jest tylko szacunkiem"
        )
    if not cal["turn"].get("measured"):
        notes.append(
            f"turn.degrees_per_second = {cal['turn']['degrees_per_second']} "
            "NIEZMIERZONE (zgadniete) - kat obrotu jest tylko szacunkiem"
        )
    return notes


# --- plan manewru ------------------------------------------------------------


def make_phase(name, label, speed, steer, seconds_at_speed, ramp_seconds):
    """
    Jedna faza ruchu jako trapez: rampa w gore, trzymanie, rampa w dol.

    seconds_at_speed to czas, ktory dalby ta sama droge/kat przy stalej
    predkosci. Rampy razem licza sie jak jedna rampa z pelna predkoscia, wiec
    hold = seconds_at_speed - ramp. Gdy ruch jest krotszy niz rampa, skracamy
    rampe - nigdy nie wydluzamy ruchu.
    """
    ramp = min(ramp_seconds, max(0.0, seconds_at_speed))
    hold = max(0.0, seconds_at_speed - ramp)
    return {
        "name": name,
        "label": label,
        "speed": speed,
        "steer": steer,
        "ramp_seconds": ramp,
        "hold_seconds": hold,
        "seconds_at_speed": seconds_at_speed,
        "wall_seconds": 2 * ramp + hold,
    }


def build_plan(
    target,
    cal,
    standoff_m=STANDOFF_M,
    camera_forward_offset_m=0.0,
    ramp_seconds=None,
    allow_long=False,
):
    """
    Zamien jeden cel ze skanu na plan: obrot w miejscu, potem jazda prosto.

    Geometria. Kamera lezy na platformie `camera_forward_offset_m` przed
    punktem, wokol ktorego platforma sie obraca (srodek osi). Obrot idzie wokol
    tego srodka, nie wokol kamery, wiec cel przeliczamy do ukladu srodka:

        forward_c = forward_m + offset,  lateral_c = lateral_m

    Po obrocie o kat srodka cel lezy na osi jazdy, a kamera - stojaca offset
    przed srodkiem - ma do niego (dystans srodka - offset). Zapas odejmujemy od
    tego, czyli `standoff_m` jest liczony OD KAMERY: to ten sam punkt, z ktorego
    pomiar powstal. Gdy offset = 0 (domyslnie w cel.json) upraszcza sie to do
    travel = ground_distance_m - standoff_m.

    Zwraca slownik z faza po fazie i lista ostrzezen. Nic nie rusza silnikow -
    ta funkcja jest w calosci testowalna bez sprzetu.
    """
    if ramp_seconds is None:
        ramp_seconds = float(cal.get("ramp_seconds", RAMP_SECONDS))

    warnings = []
    offset = float(camera_forward_offset_m)
    forward = float(target["forward_m"])
    lateral = float(target["lateral_m"])
    ground = float(target.get("ground_distance_m", math.hypot(forward, lateral)))

    # Kontrola spojnosci wejscia - lapie pomylone znaki i pomieszane jednostki
    # zanim platforma pojedzie w zla strone.
    hypot_check = math.hypot(forward, lateral)
    if abs(hypot_check - ground) > DISTANCE_TOLERANCE_M:
        warnings.append(
            f"ground_distance_m {ground:.3f} nie zgadza sie z hypot(forward, "
            f"lateral) {hypot_check:.3f} - ktore jest prawdziwe?"
        )
    bearing_file = target.get("bearing_deg")
    bearing_calc = math.degrees(math.atan2(lateral, forward))
    if bearing_file is not None and abs(float(bearing_file) - bearing_calc) > BEARING_TOLERANCE_DEG:
        warnings.append(
            f"bearing_deg {float(bearing_file):+.1f} nie zgadza sie z "
            f"atan2(lateral, forward) {bearing_calc:+.1f} - sprawdz konwencje znaku"
        )

    forward_c = forward + offset
    lateral_c = lateral
    if offset:
        # Kamera przesunieta wzgledem srodka obrotu, wiec kat i dystans trzeba
        # przeliczyc do ukladu srodka obrotu.
        distance_c = math.hypot(forward_c, lateral_c)
        turn_deg = math.degrees(math.atan2(lateral_c, forward_c))
    else:
        # Bez offsetu bierzemy kat i dystans wprost z pliku wizji - to one sa
        # pomiarem, a hypot(forward, lateral) tylko ich kontrola powyzej.
        distance_c = ground
        turn_deg = float(bearing_file) if bearing_file is not None else bearing_calc

    travel_m = distance_c - offset - float(standoff_m)
    if travel_m <= 0.0:
        warnings.append(
            f"zapas {standoff_m:.3f} m jest wiekszy niz dystans do celu "
            f"{distance_c - offset:.3f} m - nie ma czego jechac, sama korekta kata"
        )
        travel_m = 0.0
    if travel_m > MAX_TRAVEL_M and not allow_long:
        raise SystemExit(
            f"Wyliczona jazda {travel_m:.3f} m przekracza limit {MAX_TRAVEL_M} m. "
            "Skan siega 1 m, wiec to najpewniej zly plik albo zle jednostki. "
            "Swiadomie dluzej: --allow-long."
        )
    if abs(turn_deg) > MAX_TURN_DEG and not allow_long:
        raise SystemExit(
            f"Wyliczony obrot {turn_deg:+.1f} st przekracza limit {MAX_TURN_DEG} st. "
            "Cel jest praktycznie z boku - sprawdz plik. Swiadomie: --allow-long."
        )

    drive_speed = float(cal["drive"]["speed"])
    drive_mps = float(cal["drive"]["meters_per_second"])
    turn_steer = float(cal["turn"]["steer"])
    turn_dps = float(cal["turn"]["degrees_per_second"])
    if drive_mps <= 0 or turn_dps <= 0:
        raise SystemExit(
            "Kalibracja ma zerowa albo ujemna predkosc - popraw drive_calibration.json."
        )

    phases = []

    turn_seconds = abs(turn_deg) / turn_dps
    if turn_seconds >= MIN_PHASE_SECONDS:
        side = "prawo" if turn_deg >= 0 else "lewo"
        phases.append(
            make_phase(
                "turn",
                f"obrot w {side} {abs(turn_deg):.1f} st",
                # Obrot w miejscu: speed = 0, wiec mix daje kolom +-steer.
                0.0,
                math.copysign(turn_steer, turn_deg) * TURN_RIGHT_STEER_SIGN,
                turn_seconds,
                ramp_seconds,
            )
        )
    else:
        warnings.append(f"obrot {turn_deg:+.1f} st jest pomijalny - pomijam faze obrotu")

    drive_seconds = travel_m / drive_mps
    if drive_seconds >= MIN_PHASE_SECONDS:
        phases.append(
            make_phase(
                "drive",
                f"jazda prosto {travel_m:.3f} m",
                drive_speed,
                0.0,  # zero skretu: jedyny stan, ktorego mix nie zamieni w pivot
                drive_seconds,
                ramp_seconds,
            )
        )
    else:
        warnings.append("dystans jazdy jest pomijalny - pomijam faze jazdy")

    for phase in phases:
        if phase["seconds_at_speed"] < MIN_TRUSTED_SECONDS:
            warnings.append(
                f"{phase['label']} to tylko {phase['seconds_at_speed']:.2f} s ruchu - "
                "same rampy, tarcie statyczne moze zjesc caly ruch; sprawdz na oko, "
                "czy platforma w ogole drgnela"
            )

    settle = float(cal.get("settle_seconds", SETTLE_SECONDS))
    wall = sum(p["wall_seconds"] for p in phases) + settle * max(0, len(phases) - 1)

    return {
        "turn_deg": turn_deg,
        "travel_m": travel_m,
        "standoff_m": float(standoff_m),
        "camera_forward_offset_m": offset,
        "target_forward_m": forward,
        "target_lateral_m": lateral,
        "target_ground_distance_m": ground,
        "distance_from_pivot_m": distance_c,
        "ramp_seconds": ramp_seconds,
        "settle_seconds": settle,
        "drive_mps": drive_mps,
        "turn_dps": turn_dps,
        "phases": phases,
        "warnings": warnings,
        "wall_seconds": wall,
    }


def build_calibration_plan(cal, seconds=CALIBRATION_SECONDS, ramp_seconds=None):
    """Przejazd wzorcowy: N s jazdy prosto, potem N s obrotu, do zmierzenia miarka."""
    if ramp_seconds is None:
        ramp_seconds = float(cal.get("ramp_seconds", RAMP_SECONDS))
    phases = [
        make_phase(
            "drive",
            f"wzorcowa jazda prosto {seconds:.1f} s",
            float(cal["drive"]["speed"]),
            0.0,
            seconds,
            ramp_seconds,
        ),
        make_phase(
            "turn",
            f"wzorcowy obrot w prawo {seconds:.1f} s",
            0.0,
            float(cal["turn"]["steer"]) * TURN_RIGHT_STEER_SIGN,
            seconds,
            ramp_seconds,
        ),
    ]
    settle = float(cal.get("settle_seconds", SETTLE_SECONDS))
    return {
        "calibration": True,
        "ramp_seconds": ramp_seconds,
        "settle_seconds": settle,
        "phases": phases,
        "warnings": [],
        "wall_seconds": sum(p["wall_seconds"] for p in phases) + settle,
    }


def describe_plan(plan):
    """Plan jako tekst - to samo idzie na ekran w dry-run i przed prawdziwa jazda."""
    lines = []
    if not plan.get("calibration"):
        side = "prawo" if plan["target_lateral_m"] >= 0 else "lewo"
        lines.append(
            f"Cel: do przodu {plan['target_forward_m']:.3f} m, w {side} "
            f"{abs(plan['target_lateral_m']):.3f} m, po ziemi "
            f"{plan['target_ground_distance_m']:.3f} m"
        )
        lines.append(
            f"Zapas przed szyszka {plan['standoff_m']:.3f} m, offset kamery "
            f"{plan['camera_forward_offset_m']:.3f} m -> obrot "
            f"{plan['turn_deg']:+.1f} st, jazda {plan['travel_m']:.3f} m"
        )
        lines.append(
            f"Kalibracja: {plan['drive_mps']:.3f} m/s, {plan['turn_dps']:.1f} st/s"
        )
    for index, phase in enumerate(plan["phases"], start=1):
        lines.append(
            f"  {index}. {phase['label']:<28} speed {phase['speed']:+.3f} "
            f"steer {phase['steer']:+.3f}  rampa {phase['ramp_seconds']:.2f} s "
            f"+ trzymanie {phase['hold_seconds']:.2f} s + rampa "
            f"{phase['ramp_seconds']:.2f} s = {phase['wall_seconds']:.2f} s"
        )
    lines.append(f"  razem {plan['wall_seconds']:.2f} s ruchu i postojow")
    for note in plan["warnings"]:
        lines.append(f"  UWAGA: {note}")
    return "\n".join(lines)


# --- wykonanie ---------------------------------------------------------------


def run_plan(plan, platform, verbose=True):
    """
    Odtworz plan na platformie. Kazda faza: rampa w gore, trzymanie, rampa w dol.

    `ramp` zmienia oba kanaly naraz - inaczej steer wchodzilby przy zerowej
    predkosci i platforma szarpalaby pivotem na starcie kazdej fazy.
    """
    for index, phase in enumerate(plan["phases"]):
        if verbose:
            print(f"  -> {phase['label']}")
        speed, steer = phase["speed"], phase["steer"]
        platform.ramp(0.0, speed, 0.0, steer, phase["ramp_seconds"])
        platform.hold(speed, steer, phase["hold_seconds"])
        platform.ramp(speed, 0.0, steer, 0.0, phase["ramp_seconds"])
        platform.stop(plan["settle_seconds"] if index < len(plan["phases"]) - 1 else 0.3)


@contextmanager
def virtual_clock(platform_module):
    """
    Podmien monotonic/sleep w makarena.platform na wirtualny zegar.

    Dzieki temu dry-run przechodzi DOKLADNIE ten sam kod ramp/hold/stop co
    prawdziwa jazda, tylko natychmiast, i widzi kazda komende przez on_command.
    Test planu opiera sie na tym samym.
    """
    state = {"t": 0.0}
    real_monotonic = platform_module.monotonic
    real_sleep = platform_module.sleep

    def fake_sleep(seconds):
        # Minimalny krok, zeby petla hold() nie zawisla na zerowej reszcie czasu.
        state["t"] += max(float(seconds), 1e-6)

    platform_module.monotonic = lambda: state["t"]
    platform_module.sleep = fake_sleep
    try:
        yield state
    finally:
        platform_module.monotonic = real_monotonic
        platform_module.sleep = real_sleep


def simulate_plan(plan, makarena_dir=MAKARENA_DIR, invert_speed=False, invert_steer=False):
    """
    Przejedz plan na sucho i zwroc (lista komend, czas symulowany).

    Komendy to (t, pwm_speed, pwm_steer, speed, steer) - to samo, co poszloby na
    port. Port nie jest otwierany (dry_run=True w Link).
    """
    Platform, platform_module = import_makarena(makarena_dir)
    commands = []
    with virtual_clock(platform_module) as clock:
        platform = Platform(
            dry_run=True,
            invert_speed=invert_speed,
            invert_steer=invert_steer,
            on_command=lambda t, ps, pt, s, st: commands.append((t, ps, pt, s, st)),
        )
        platform.link.open()  # tylko wypis [dry-run], bez portu
        run_plan(plan, platform, verbose=False)
        platform.stop(0.0)
        elapsed = clock["t"]
    return commands, elapsed


def phase_command_spans(plan, commands):
    """
    Podziel strumien komend na fazy, po kolejnosci - do kontroli w testach.

    Faza konczy sie na stop(), a stop wysyla zera, wiec granice bierzemy z
    liczby komend: kazda faza to ciagly kawalek, a wszystko po ostatniej
    komendzie niezerowej danej fazy nalezy do stopu.
    """
    spans = {}
    cursor = 0
    for phase in plan["phases"]:
        span = []
        nonzero_seen = False
        while cursor < len(commands):
            _, pwm_speed, pwm_steer, _, _ = commands[cursor]
            zero = pwm_speed == 0 and pwm_steer == 0
            if zero and nonzero_seen:
                break  # weszlismy w stop() po tej fazie
            if not zero:
                nonzero_seen = True
            span.append(commands[cursor])
            cursor += 1
        # Przewin zera stopu, zeby nastepna faza startowala od swoich komend.
        while cursor < len(commands):
            _, pwm_speed, pwm_steer, _, _ = commands[cursor]
            if pwm_speed != 0 or pwm_steer != 0:
                break
            cursor += 1
        spans[phase["name"]] = span
    return spans


# --- wejscie / CLI -----------------------------------------------------------


def load_targets(path):
    if not os.path.exists(path):
        raise SystemExit(
            f"Nie ma pliku {path}. Zrob skan: python scan_cones.py --json {path}"
        )
    with open(path) as handle:
        data = json.load(handle)
    targets = data.get("targets") or []
    if not targets:
        raise SystemExit(
            f"{path} nie ma zadnego celu - skan nic nie znalazl. Powtorz skan."
        )
    return data, targets


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Dojazd open-loop do szyszki zapamietanej przez scan_cones.py"
    )
    p.add_argument("--targets-file", default=TARGETS_FILE, help="plik z celami ze skanu")
    p.add_argument(
        "--target",
        type=int,
        default=0,
        help="ktory cel z pliku (0 = najblizszy, lista jest sortowana)",
    )
    p.add_argument(
        "--standoff",
        type=float,
        default=STANDOFF_M,
        help="zapas przed szyszka [m]; o tyle krocej jedziemy, zeby stanac w "
        "zasiegu chwytaka a nie na celu",
    )
    p.add_argument("--port", help="port szeregowy Xiao, np. COM9; BEZ NIEGO NIE MA JAZDY")
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="wymus symulacje nawet z podanym --port (bez portu jest domyslna)",
    )
    p.add_argument("--calibrate", action="store_true", help="przejazd wzorcowy do miarki")
    p.add_argument(
        "--calibration-seconds",
        type=float,
        default=CALIBRATION_SECONDS,
        help="dlugosc kazdej fazy przejazdu wzorcowego [s]",
    )
    p.add_argument("--calibration-file", default=CALIBRATION_FILE)
    p.add_argument("--makarena-dir", default=MAKARENA_DIR, help="repo z pakietem makarena")
    p.add_argument("--ramp-seconds", type=float, help="nadpisz rampe z kalibracji [s]")
    p.add_argument("--speed", type=float, help="nadpisz predkosc jazdy (0..1)")
    p.add_argument("--steer", type=float, help="nadpisz sile obrotu w miejscu (0..1)")
    p.add_argument("--invert-speed", action="store_true", help="platforma jedzie w tyl")
    p.add_argument("--invert-steer", action="store_true", help="obrot wychodzi w druga strone")
    p.add_argument(
        "--allow-long",
        action="store_true",
        help=f"pozwol na jazde ponad {MAX_TRAVEL_M} m / obrot ponad {MAX_TURN_DEG} st",
    )
    p.add_argument("--verbose", action="store_true", help="wypisuj kazda komende PWM")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    cal = load_calibration(args.calibration_file)
    if args.speed is not None:
        cal["drive"]["speed"] = args.speed
    if args.steer is not None:
        cal["turn"]["steer"] = args.steer

    if args.calibrate:
        plan = build_calibration_plan(
            cal, seconds=args.calibration_seconds, ramp_seconds=args.ramp_seconds
        )
        print("Przejazd wzorcowy do kalibracji (procedura w docstringu skryptu).")
    else:
        data, targets = load_targets(args.targets_file)
        if not 0 <= args.target < len(targets):
            raise SystemExit(
                f"--target {args.target} poza zakresem: plik ma {len(targets)} celow."
            )
        plan = build_plan(
            targets[args.target],
            cal,
            standoff_m=args.standoff,
            camera_forward_offset_m=data.get("camera_forward_offset_m", 0.0),
            ramp_seconds=args.ramp_seconds,
            allow_long=args.allow_long,
        )

    print(describe_plan(plan))

    notes = calibration_warnings(cal)
    if notes:
        print("Kalibracja niepelna:")
        for note in notes:
            print(f"  - {note}")

    dry_run = args.dry_run or args.port is None
    if dry_run:
        if args.port is None and not args.dry_run:
            print("Brak --port, wiec tylko symulacja - silniki nietkniete.")
        commands, elapsed = simulate_plan(
            plan,
            makarena_dir=args.makarena_dir,
            invert_speed=args.invert_speed,
            invert_steer=args.invert_steer,
        )
        print(
            f"[dry-run] {len(commands)} komend, symulowany czas {elapsed:.2f} s, "
            f"ostatnia {commands[-1][1]} / {commands[-1][2]}"
        )
        return 0

    if notes and not args.calibrate:
        raise SystemExit(
            "Nie jedziemy po cel na niezmierzonej kalibracji - dystans i kat "
            "bylyby zgadniete. Najpierw --calibrate i wpisanie pomiarow do "
            f"{args.calibration_file} (measured: true)."
        )

    Platform, _ = import_makarena(args.makarena_dir)
    print(f"JAZDA na {args.port}. Wylacznik w rece, Ctrl+C zeruje PWM.")
    platform = Platform(
        port=args.port,
        invert_speed=args.invert_speed,
        invert_steer=args.invert_steer,
        verbose=args.verbose,
    )
    with platform:
        try:
            run_plan(plan, platform)
        except KeyboardInterrupt:
            print("\nPrzerwano - zeruje PWM.")
    if args.calibrate:
        phase_by_name = {p["name"]: p for p in plan["phases"]}
        drive = phase_by_name["drive"]
        turn = phase_by_name["turn"]
        print("\nZmierz miarka i policz:")
        print(
            f"  drive.meters_per_second = droga [m] / "
            f"{drive['hold_seconds'] + drive['ramp_seconds']:.2f}"
        )
        print(
            f"  turn.degrees_per_second = kat [st] / "
            f"{turn['hold_seconds'] + turn['ramp_seconds']:.2f}"
        )
        print(f"Wpisz do {args.calibration_file} i ustaw measured na true.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
