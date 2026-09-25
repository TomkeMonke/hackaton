"""
Test planu dojazdu z drive_to_target.py - bez sprzetu, bez portu, bez pytest.

    python test_drive_plan.py

Sprawdzane jest to, co w open-loop boli najbardziej i czego na sprzecie nie
wychwycisz bez rozbijania platformy:

  * znak obrotu: cel w prawo (bearing dodatni) -> steer dodatni -> obrot w prawo
  * czasy zgodne ze stalymi kalibracji, z modelem trapezu (hold + ramp)
  * zapas faktycznie skraca dystans jazdy
  * faza jazdy prosto NIGDY nie wchodzi w |steer| >= |speed|, czyli nie zamienia
    sie w pivot - to jedyny stan, ktory cicho psuje caly dojazd

Ostatni punkt jest sprawdzany na PRAWDZIWYM strumieniu komend: plan przechodzi
przez Platform(dry_run=True) z wirtualnym zegarem, wiec leci dokladnie ten kod
ramp/hold/stop, ktory pojdzie na port.
"""

import math

from drive_to_target import (
    DEFAULT_CALIBRATION,
    build_calibration_plan,
    build_plan,
    load_calibration,
    phase_command_spans,
    simulate_plan,
)

# Cel z realnego skanu (przyklad z cel.json): 0.497 m w przod, 11.4 cm w prawo.
TARGET = {
    "forward_m": 0.497,
    "lateral_m": 0.114,
    "bearing_deg": 13.0,
    "ground_distance_m": 0.510,
    "width_m": 0.013,
    "angle_deg": -31.4,
}

# Kalibracja testowa: okragle liczby, zeby czasy dawaly sie policzyc w glowie.
CAL = {
    "drive": {"speed": 0.06, "meters_per_second": 0.10, "measured": True},
    "turn": {"steer": 0.075, "degrees_per_second": 20.0, "measured": True},
    "ramp_seconds": 0.4,
    "settle_seconds": 0.5,
}


def phases_by_name(plan):
    return {p["name"]: p for p in plan["phases"]}


def test_znak_obrotu_w_prawo():
    plan = build_plan(TARGET, CAL, standoff_m=0.12)
    turn = phases_by_name(plan)["turn"]
    assert plan["turn_deg"] > 0, plan["turn_deg"]
    assert turn["steer"] > 0, turn["steer"]
    assert "prawo" in turn["label"], turn["label"]
    # Obrot w miejscu = speed dokladnie 0, inaczej to luk a nie obrot.
    assert turn["speed"] == 0.0, turn["speed"]


def test_znak_obrotu_w_lewo():
    left = dict(TARGET, lateral_m=-0.114, bearing_deg=-13.0)
    plan = build_plan(left, CAL, standoff_m=0.12)
    turn = phases_by_name(plan)["turn"]
    assert plan["turn_deg"] < 0, plan["turn_deg"]
    assert turn["steer"] < 0, turn["steer"]
    assert "lewo" in turn["label"], turn["label"]
    # Lustrzany cel musi dac ten sam czas obrotu, tylko w druga strone.
    prawo = phases_by_name(build_plan(TARGET, CAL, standoff_m=0.12))["turn"]
    assert abs(turn["seconds_at_speed"] - prawo["seconds_at_speed"]) < 1e-9
    assert abs(turn["steer"] + prawo["steer"]) < 1e-9


def test_czasy_zgodne_z_kalibracja():
    plan = build_plan(TARGET, CAL, standoff_m=0.12)
    turn = phases_by_name(plan)["turn"]
    drive = phases_by_name(plan)["drive"]

    # 13 st przy 20 st/s = 0.65 s "przy pelnej predkosci".
    assert abs(turn["seconds_at_speed"] - 13.0 / 20.0) < 1e-9, turn
    # 0.510 - 0.120 = 0.390 m przy 0.10 m/s = 3.90 s.
    assert abs(plan["travel_m"] - 0.390) < 1e-9, plan["travel_m"]
    assert abs(drive["seconds_at_speed"] - 3.90) < 1e-9, drive

    # Model trapezu: rampa w gore + rampa w dol = jedna rampa pelnej predkosci,
    # wiec hold = seconds_at_speed - ramp, a zegarek widzi 2*ramp + hold.
    for phase in (turn, drive):
        assert abs(phase["hold_seconds"] - (phase["seconds_at_speed"] - phase["ramp_seconds"])) < 1e-9
        assert abs(phase["wall_seconds"] - (2 * phase["ramp_seconds"] + phase["hold_seconds"])) < 1e-9
        assert phase["hold_seconds"] >= 0.0

    # Predkosci z kalibracji trafiaja do komend 1:1.
    assert drive["speed"] == CAL["drive"]["speed"]
    assert abs(turn["steer"]) == CAL["turn"]["steer"]


def test_ruch_krotszy_od_rampy_skraca_rampe():
    # 1 st przy 20 st/s = 0.05 s, czyli krocej niz rampa 0.4 s.
    blisko = dict(TARGET, lateral_m=0.0087, bearing_deg=1.0)
    turn = phases_by_name(build_plan(blisko, CAL, standoff_m=0.12))["turn"]
    assert turn["hold_seconds"] == 0.0, turn
    assert abs(turn["ramp_seconds"] - 0.05) < 1e-9, turn
    # Skracamy rampe, a nie wydluzamy ruch - inaczej obrot przestrzeli kat.
    assert abs(turn["seconds_at_speed"] - 0.05) < 1e-9, turn


def test_zapas_skraca_dystans():
    bez = build_plan(TARGET, CAL, standoff_m=0.0)
    maly = build_plan(TARGET, CAL, standoff_m=0.10)
    duzy = build_plan(TARGET, CAL, standoff_m=0.25)

    assert abs(bez["travel_m"] - TARGET["ground_distance_m"]) < 1e-9, bez["travel_m"]
    assert abs(bez["travel_m"] - maly["travel_m"] - 0.10) < 1e-9
    assert duzy["travel_m"] < maly["travel_m"] < bez["travel_m"]
    # Krotszy dystans = krotsza jazda, bo czas jest jedyna miara drogi.
    assert (
        phases_by_name(duzy)["drive"]["seconds_at_speed"]
        < phases_by_name(maly)["drive"]["seconds_at_speed"]
        < phases_by_name(bez)["drive"]["seconds_at_speed"]
    )
    # Zapas nie rusza kata - to dwie niezalezne rzeczy.
    assert abs(duzy["turn_deg"] - bez["turn_deg"]) < 1e-9


def test_zapas_wiekszy_niz_dystans_nie_jedzie():
    plan = build_plan(TARGET, CAL, standoff_m=0.60)
    assert plan["travel_m"] == 0.0, plan["travel_m"]
    assert "drive" not in phases_by_name(plan), "nie powinno byc fazy jazdy"
    assert any("zapas" in w for w in plan["warnings"]), plan["warnings"]


def test_offset_kamery_przelicza_geometrie():
    # Kamera 20 cm przed srodkiem obrotu: cel widziany pod 13 st jest wzgledem
    # srodka pod mniejszym katem, a dystans do przejechania sie nie zmienia.
    plan = build_plan(TARGET, CAL, standoff_m=0.12, camera_forward_offset_m=0.20)
    bez = build_plan(TARGET, CAL, standoff_m=0.12)
    assert 0 < plan["turn_deg"] < bez["turn_deg"], (plan["turn_deg"], bez["turn_deg"])
    expected = math.degrees(math.atan2(0.114, 0.497 + 0.20))
    assert abs(plan["turn_deg"] - expected) < 1e-9
    # Cialo jest sztywne, wiec offset prawie nie zmienia drogi (tu < 3 cm).
    assert abs(plan["travel_m"] - bez["travel_m"]) < 0.03


def test_niespojne_wejscie_daje_ostrzezenie():
    zly = dict(TARGET, bearing_deg=-13.0)  # znak odwrotny niz lateral_m
    plan = build_plan(zly, CAL, standoff_m=0.12)
    assert any("bearing_deg" in w for w in plan["warnings"]), plan["warnings"]


def test_jazda_prosto_nie_wchodzi_w_pivot():
    """
    Na prawdziwym strumieniu komend: w fazie jazdy zaden tick nie moze miec
    |pwm_steer| >= |pwm_speed| przy niezerowym pwm_speed - to wlasnie moment, w
    ktorym plytka puszcza jedno kolo w tyl i platforma kreci sie zamiast jechac.
    """
    plan = build_plan(TARGET, CAL, standoff_m=0.12)
    commands, elapsed = simulate_plan(plan)
    assert commands, "symulacja nie wyslala ani jednej komendy"
    spans = phase_command_spans(plan, commands)

    drive = spans["drive"]
    assert drive, "faza jazdy nie ma komend"
    for t, pwm_speed, pwm_steer, _, _ in drive:
        assert pwm_steer == 0, f"jazda prosto ma niezerowy steer {pwm_steer} w t={t:.2f}"
        if pwm_speed != 0:
            assert abs(pwm_steer) < abs(pwm_speed), (t, pwm_speed, pwm_steer)
    assert any(c[1] != 0 for c in drive), "faza jazdy nie ruszyla silnikow"

    # Faza obrotu ma byc odwrotnie: zerowy speed i niezerowy steer (pivot
    # zamierzony), inaczej platforma jechalaby po luku zamiast sie obrocic.
    turn = spans["turn"]
    for t, pwm_speed, pwm_steer, _, _ in turn:
        assert pwm_speed == 0, f"obrot ma niezerowy speed {pwm_speed} w t={t:.2f}"
    assert any(c[2] != 0 for c in turn), "faza obrotu nie ruszyla silnikow"

    # Symulowany czas nie moze byc krotszy od planu - inaczej komendy nie
    # trzymalyby sie przez caly ruch i plytka zgasilaby silniki w polowie.
    assert elapsed >= plan["wall_seconds"] - 0.05, (elapsed, plan["wall_seconds"])
    # Ostatnia komenda to zawsze stop.
    assert commands[-1][1] == 0 and commands[-1][2] == 0, commands[-1]


def test_komendy_powtarzane_dosc_czesto():
    """Plytka gasi silniki po ~500 ms ciszy, wiec odstepy musza byc znacznie mniejsze."""
    plan = build_plan(TARGET, CAL, standoff_m=0.12)
    commands, _ = simulate_plan(plan)
    times = [c[0] for c in commands]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert max(gaps) < 0.2, f"najwieksza przerwa {max(gaps):.3f} s, limit plytki to 0.5 s"


def test_plan_kalibracji():
    plan = build_calibration_plan(CAL, seconds=3.0)
    drive = phases_by_name(plan)["drive"]
    turn = phases_by_name(plan)["turn"]
    assert abs(drive["seconds_at_speed"] - 3.0) < 1e-9
    assert abs(turn["seconds_at_speed"] - 3.0) < 1e-9
    assert drive["steer"] == 0.0 and turn["speed"] == 0.0
    # Mnoznik do wpisania w kalibracje: droga / (hold + ramp).
    assert abs((drive["hold_seconds"] + drive["ramp_seconds"]) - 3.0) < 1e-9
    commands, _ = simulate_plan(plan)
    spans = phase_command_spans(plan, commands)
    for t, pwm_speed, pwm_steer, _, _ in spans["drive"]:
        assert pwm_steer == 0, (t, pwm_steer)


def test_domyslna_kalibracja_oznaczona_jako_niezmierzona():
    assert DEFAULT_CALIBRATION["measured"] is False
    assert DEFAULT_CALIBRATION["drive"]["measured"] is False
    assert DEFAULT_CALIBRATION["turn"]["measured"] is False
    # Plik na dysku (jesli istnieje) musi dac sie wczytac i miec obie sekcje.
    cal = load_calibration(create=False)
    assert cal["drive"]["meters_per_second"] > 0
    assert cal["turn"]["degrees_per_second"] > 0


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print(f"OK  {test.__name__}")
    print(f"\nOK - {len(tests)} testow przeszlo")


if __name__ == "__main__":
    main()
