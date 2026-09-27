# live_grasp - chwytanie szyszki samym ramieniem (eksperyment 2026-09-26)

Instrukcja krok po kroku i stan prac: `docs/LIVE_GRASP.md`.

Skrypty ida na Pi przez ssh stdin (`pi.sh`), nic nie jest kopiowane na Pi. `percept.py` (glebia: plaszczyzna
ziemi, wysokosc, szyszki) jest doklejany przed kazdym skryptem.

- `teach_clean.py` - uczenie: HOME -> 1 szyszka -> chwyt reka -> zapis (glowna droga dalej)
- `fit.py` - model szyszka (kamera w HOME) -> stawy chwytu, blad "zostaw-jedna"
- `pick.py`, `pick_loop.py` - autonomiczny chwyt z modelu + sloik
- `rec.py`, `replay.py` - nagranie ruchu reka i odtworzenie cyklu
- `snap_only.py`, `step.py` - zdjecie / pojedynczy ruch
- `attempt.py`, `servo.py`, `grasp_servo.py` - proby, ktore nie zadzialaly (opis w docs)
- `data/` - nagranie 2 min (stawy), sesje uczenia 1-2 (bez zdjec), model, sciezka do sloika
