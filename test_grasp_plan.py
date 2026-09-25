"""
Test planu chwytu BEZ SPRZETU: bez ramienia, bez portu, bez kamery.

Uruchomienie (zwykly python, bez pytest):
    python test_grasp_plan.py

Sprawdzamy to, co naprawde moze zniszczyc chwyt albo ramie:
  * sekwencja ma sensowne kroki w sensownej kolejnosci i trafia w cel,
  * chwytak otwiera sie SZERZEJ niz szyszka, a zaciska wezej,
  * cel poza zasiegiem jest ODRZUCANY z czytelnym komunikatem, a nie cicho
    obcinany do czegos, co ramie akurat dosiegnie,
  * dry-run nie tyka lerobota ani portu.
"""

from __future__ import annotations

import contextlib
import io
import json
import math
import os
import sys
import tempfile
import traceback

import approach_and_grasp as ag

# Cel jak z prawdziwego skanu (scan_cones.py --json): szyszka 0.5 m przed
# kamera, 11 cm w prawo, os chwytania 1.3 cm.
CEL = {
    "forward_m": 0.497,
    "lateral_m": 0.114,
    "ground_distance_m": 0.510,
    "distance_m": 0.517,
    "bearing_deg": 13.0,
    "width_m": 0.013,
    "length_m": 0.029,
    "height_m": 0.024,
    "angle_deg": -31.4,
    "seen_in": "20/20",
    "spread_forward_mm": 0.2,
}
NAGLOWEK = {"camera_height_m": 0.1, "camera_forward_offset_m": 0.0, "scanned_at": "test"}

KROKI_OCZEKIWANE = [
    "pozycja wyjsciowa",
    "nad szyszka",
    "otwarcie chwytaka",
    "opuszczenie na szyszke",
    "zacisniecie",
    "podniesienie",
]


def transformata() -> ag.Transform:
    """Transformata z repo, a jak jej nie ma - domyslne oszacowanie z kodu."""
    return ag.load_transform(ag.TRANSFORM_PATH)


def ramie() -> ag.ArmModel:
    return ag.ArmModel()


def cel_po_dojazdzie(standoff_m: float = ag.DEFAULT_STANDOFF_M) -> dict:
    """Cel przeliczony tak, jak robi to main() po dojazdzie platformy."""
    drive = ag.plan_drive(CEL, standoff_m, NAGLOWEK)
    target = dict(CEL)
    target["forward_m"] = standoff_m
    target["lateral_m"] = 0.0
    target["ground_distance_m"] = standoff_m
    target["angle_deg"] = CEL["angle_deg"] + drive["obrot_deg"]
    return target


# ---------------------------------------------------------------- testy


def test_dojazd_zdejmuje_dystans_i_namiar():
    drive = ag.plan_drive(CEL, 0.15, NAGLOWEK)
    assert abs(drive["obrot_deg"] - CEL["bearing_deg"]) < 1e-9, drive
    assert abs(drive["przod_m"] - (CEL["ground_distance_m"] - 0.15)) < 1e-9, drive
    assert drive["po_dojazdzie"]["forward_m"] == 0.15
    assert drive["po_dojazdzie"]["lateral_m"] == 0.0
    # Jazda do tylu ma byc zgloszona, nie przemilczana.
    blisko = ag.plan_drive({**CEL, "ground_distance_m": 0.05}, 0.15, NAGLOWEK)
    assert blisko["przod_m"] < 0
    assert any("TYLU" in note.upper() for note in blisko["uwagi"]), blisko["uwagi"]


def test_plan_ma_sensowne_kroki():
    plan = ag.plan_grasp(cel_po_dojazdzie(), transformata(), ramie())
    nazwy = [krok["nazwa"] for krok in plan["kroki"]]
    assert nazwy == KROKI_OCZEKIWANE, nazwy

    # IK naprawde trafilo w punkty, a nie "gdzies obok".
    for label, residual in plan["residua"].items():
        assert residual["pozycja_mm"] <= ag.IK_POSITION_TOLERANCE_M * 1000, (label, residual)
        assert residual["os_deg"] <= ag.IK_AXIS_TOLERANCE_DEG, (label, residual)

    # Kolejnosc wysokosci: zawisniecie wyzej niz chwyt, podniesienie znowu wyzej.
    krok = {k["nazwa"]: k for k in plan["kroki"]}
    nad = krok["nad szyszka"]["punkt_bazy_m"]
    chwyt = krok["opuszczenie na szyszke"]["punkt_bazy_m"]
    gora = krok["podniesienie"]["punkt_bazy_m"]
    assert nad[2] > chwyt[2] + 0.02, (nad, chwyt)
    assert gora[2] > chwyt[2] + 0.02, (gora, chwyt)

    # Punkt chwytu lezy nad ziemia, nie w ziemi. Ziemia w ukladzie bazy to
    # translacja_m.z transformaty.
    ziemia_z = transformata().translation[2]
    assert chwyt[2] > ziemia_z, (chwyt[2], ziemia_z)
    assert plan["wysokosc_chwytu_m"] >= ag.MIN_GRASP_HEIGHT_M
    assert plan["wysokosc_chwytu_m"] <= CEL["height_m"], plan["wysokosc_chwytu_m"]

    # Zaden krok nie wychodzi za limit przegubu z URDF.
    arm = ramie()
    for k in plan["kroki"]:
        radiany = {n: math.radians(v) for n, v in k["katy_deg"].items()}
        assert not arm.limit_violations(radiany), (k["nazwa"], arm.limit_violations(radiany))

    # Kazdy krok ma gotowa komende dla arm_control.move_to.
    for k in plan["kroki"]:
        assert set(k["lerobot"]) == set(ag.JOINT_NAMES) | {ag.GRIPPER_JOINT}, k["lerobot"]
        for name, value in k["lerobot"].items():
            assert isinstance(value, float) and not math.isnan(value), (k["nazwa"], name, value)

    # Chwyt nie zmienia pozy: zacisniecie idzie z tych samych katow co opuszczenie.
    assert krok["zacisniecie"]["katy_deg"] == krok["opuszczenie na szyszke"]["katy_deg"]
    # Otwarcie chwytaka tez nie rusza ramieniem - szczeki otwieraja sie w powietrzu.
    assert krok["otwarcie chwytaka"]["katy_deg"] == krok["nad szyszka"]["katy_deg"]


def test_chwytak_otwiera_sie_szerzej_niz_szyszka():
    plan = ag.plan_grasp(cel_po_dojazdzie(), transformata(), ramie())
    szerokosc = CEL["width_m"]
    chwytak = plan["chwytak"]
    assert chwytak["open_m"] > szerokosc + ag.GRIPPER_MIN_MARGIN_M, chwytak
    assert chwytak["close_m"] < szerokosc, chwytak
    assert chwytak["open_cmd"] > chwytak["close_cmd"], chwytak

    krok = {k["nazwa"]: k for k in plan["kroki"]}
    # Przed zejsciem i w czasie zejscia szczeki sa szerzej niz szyszka.
    for nazwa in ("otwarcie chwytaka", "opuszczenie na szyszke"):
        assert krok[nazwa]["chwytak_rozwarcie_m"] > szerokosc, (nazwa, krok[nazwa])
    # Po zacisnieciu wezej - inaczej szyszka wypadnie.
    assert krok["zacisniecie"]["chwytak_rozwarcie_m"] < szerokosc

    # Komenda tez musi rosnac razem z rozwarciem (0 = zamkniety).
    assert krok["otwarcie chwytaka"]["chwytak_cmd"] > krok["zacisniecie"]["chwytak_cmd"]


def test_za_szeroka_szyszka_odrzucona():
    szeroka = dict(cel_po_dojazdzie())
    szeroka["width_m"] = 0.06  # szerzej niz szczeki sie otwieraja
    try:
        ag.plan_grasp(szeroka, transformata(), ramie())
    except ag.PlanError as exc:
        tekst = str(exc)
        assert "ZA SZEROKA" in tekst.upper(), tekst
        assert "6.0 cm" in tekst, tekst
    else:
        raise AssertionError("za szeroka szyszka przeszla przez plan")


def test_cel_poza_zasiegiem_odrzucony_a_nie_obciety():
    """Surowy cel z 0.5 m: bez dojazdu ramie tam nie siega."""
    try:
        plan = ag.plan_grasp(CEL, transformata(), ramie())
    except ag.PlanError as exc:
        tekst = str(exc)
        assert "POZA ZASIEGIEM" in tekst.upper(), tekst
        # Komunikat ma powiedziec ILE brakuje i co z tym zrobic.
        assert "cm" in tekst and "shoulder_pan" in tekst, tekst
        assert "Podjedz" in tekst, tekst
    else:
        raise AssertionError(
            "cel 0.5 m przed kamera zostal zaplanowany - czyli gdzies po drodze "
            f"zostal cicho obciety: {plan['punkt_chwytu_baza_m']}"
        )


def test_cel_w_granicy_ale_bez_ik_tez_odrzucony():
    """Druga bramka: punkt miesci sie w sumie dlugosci czlonow, a IK nie wychodzi.

    Wymuszamy pionowe podejscie - to ono ogranicza zasieg najbardziej.
    """
    daleki = dict(cel_po_dojazdzie(standoff_m=0.34))
    arm = ramie()
    punkt = transformata().to_base(ag.grasp_point_ground(daleki)[0])
    odleglosc = float(
        math.sqrt(sum((punkt[i] - arm.pan_origin[i]) ** 2 for i in range(3)))
    )
    assert odleglosc < arm.reach_bound_m, (
        "test bez sensu: punkt wypadl juz za granica geometryczna",
        odleglosc,
        arm.reach_bound_m,
    )
    try:
        ag.plan_grasp(daleki, transformata(), arm, approach_pitches_deg=(90.0,))
    except ag.PlanError as exc:
        tekst = str(exc)
        assert "IK" in tekst, tekst
        assert "nic nie zostalo obciete" in tekst, tekst
        assert "mm" in tekst, tekst
    else:
        raise AssertionError("nieosiagalny punkt zostal zaplanowany jako pionowy chwyt")


def test_obrot_szczek_jest_prostopadly_do_dluzszej_osi():
    transform = transformata()
    for kat in (-31.4, 0.0, 45.0, 89.0):
        cel = {**CEL, "angle_deg": kat}
        dluzsza_os = ag.wrap90(kat + ag.LONG_AXIS_YAW_FROM_DETECTOR_DEG)
        szczeki = ag.desired_jaw_yaw_deg(cel, transform, 0.0)
        roznica = abs(ag.wrap90(szczeki - dluzsza_os))
        assert abs(roznica - 90.0) < 1e-6, (kat, dluzsza_os, szczeki, roznica)


def test_wrist_roll_idzie_za_katem_szyszki():
    """Obrot szyszki ma wyladowac w wrist_roll, a nie zginac po drodze."""
    transform = transformata()
    arm = ramie()
    poprzedni = None
    for kat in (-40.0, 0.0, 40.0):
        cel = dict(cel_po_dojazdzie())
        cel["angle_deg"] = kat
        plan = ag.plan_grasp(cel, transform, arm)
        zadany = plan["obrot_szczek_deg"]
        osiagniety = plan["residua"]["opuszczenie"]["obrot_szczek_deg"]
        assert abs(osiagniety) <= ag.JAW_YAW_TOLERANCE_DEG, (kat, zadany, osiagniety)
        roll = plan["kroki"][3]["katy_deg"]["wrist_roll"]
        if poprzedni is not None:
            assert abs(roll - poprzedni) > 1.0, (kat, roll, poprzedni)
        poprzedni = roll


def test_transformata_jest_oznaczona_jako_oszacowanie():
    transform = transformata()
    assert transform.measured is False, "ktos oznaczyl transformate jako zmierzona"
    opis = " ".join(transform.describe())
    assert "OSZACOWANIE" in opis, opis
    # Brakujacy plik nie ma prawa wysypac skryptu - schodzi na domyslne.
    zastepcza = ag.load_transform(os.path.join(tempfile.gettempdir(), "nie-ma-takiego.json"))
    assert zastepcza.measured is False
    assert zastepcza.translation.shape == (3,)


def test_brak_celu_to_czytelny_blad():
    brak = os.path.join(tempfile.gettempdir(), "cel-ktorego-nie-ma.json")
    try:
        ag.load_target(brak, 0)
    except ag.PlanError as exc:
        assert "scan_cones" in str(exc), str(exc)
    else:
        raise AssertionError("brak pliku celu przeszedl bez bledu")

    with tempfile.TemporaryDirectory() as folder:
        pusty = os.path.join(folder, "cel.json")
        with open(pusty, "w", encoding="utf-8") as handle:
            json.dump({"targets": []}, handle)
        try:
            ag.load_target(pusty, 0)
        except ag.PlanError as exc:
            assert "nie ma zadnego celu" in str(exc), str(exc)
        else:
            raise AssertionError("pusta lista celow przeszla bez bledu")

        with open(pusty, "w", encoding="utf-8") as handle:
            json.dump({"targets": [CEL]}, handle)
        try:
            ag.load_target(pusty, 5)
        except ag.PlanError as exc:
            assert "indeksie 5" in str(exc), str(exc)
        else:
            raise AssertionError("zly indeks celu przeszedl bez bledu")


def test_dry_run_nie_tyka_lerobota_ani_portu():
    with tempfile.TemporaryDirectory() as folder:
        plik = os.path.join(folder, "cel.json")
        with open(plik, "w", encoding="utf-8") as handle:
            json.dump({**NAGLOWEK, "targets": [CEL]}, handle)

        bufor = io.StringIO()
        with contextlib.redirect_stdout(bufor):
            kod = ag.main(["--target", plik])
        wypis = bufor.getvalue()
        assert kod == 0, wypis
        assert "DRY-RUN" in wypis, wypis
        assert "nic nie zostalo wyslane" in wypis, wypis
        assert "OSZACOWANIE" in wypis, "wypis nie mowi, ze transformata jest oszacowana"
        for nazwa in KROKI_OCZEKIWANE:
            assert nazwa.upper() in wypis, nazwa
        assert "lerobot" not in sys.modules, "dry-run zaimportowal lerobota"

        # Zadanie ruchu bez portu musi sie skonczyc bledem, nie ruchem.
        bufor = io.StringIO()
        with contextlib.redirect_stdout(bufor):
            kod = ag.main(["--target", plik, "--no-dry-run"])
        assert kod == 2, bufor.getvalue()
        assert "wymaga --port" in bufor.getvalue()

        # Cel poza zasiegiem: kod wyjscia 2, nie 0.
        bufor = io.StringIO()
        with contextlib.redirect_stdout(bufor):
            kod = ag.main(["--target", plik, "--no-drive"])
        assert kod == 2, bufor.getvalue()
        assert "ODRZUCONE" in bufor.getvalue()


def test_plan_zapisuje_sie_do_json():
    with tempfile.TemporaryDirectory() as folder:
        plik = os.path.join(folder, "cel.json")
        wynik = os.path.join(folder, "plan.json")
        with open(plik, "w", encoding="utf-8") as handle:
            json.dump({**NAGLOWEK, "targets": [CEL]}, handle)
        with contextlib.redirect_stdout(io.StringIO()):
            kod = ag.main(["--target", plik, "--json", wynik])
        assert kod == 0
        with open(wynik, "r", encoding="utf-8") as handle:
            zapisane = json.load(handle)
        assert len(zapisane["plan"]["kroki"]) == len(KROKI_OCZEKIWANE)
        assert zapisane["dojazd"]["obrot_deg"] == CEL["bearing_deg"]


# ---------------------------------------------------------------- runner


def main() -> int:
    # Bez ikpy nie ma IK - lepiej powiedziec to raz, niz 12 razy w kazdym tescie.
    try:
        ramie()
    except ag.PlanError as exc:
        print(f"Nie da sie zbudowac modelu ramienia: {exc}")
        return 1

    testy = [
        (name, obj)
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and callable(obj)
    ]
    bledy = []
    for name, test in testy:
        try:
            test()
        except Exception as exc:  # noqa: BLE001 - runner ma zlapac wszystko
            bledy.append((name, exc))
            print(f"[BLAD] {name}: {type(exc).__name__}: {exc}")
            traceback.print_exc()
        else:
            print(f"[OK]   {name}")
    print()
    print(f"{len(testy) - len(bledy)}/{len(testy)} testow przeszlo")
    if bledy:
        print("Nie przeszlo: " + ", ".join(name for name, _ in bledy))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
