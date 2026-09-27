# STACK - na czym zbudowane jest wykrywanie i zbieranie szyszek

**Stan na:** 2026-09-27, master `4f754c3`. Aktualny stan sprzetu i nastepne kroki: `docs/STATUS.md`.

Krotko: zadnego ML ani wytrenowanej polityki. `pyrealsense2` i `lerobot` sa, ale tylko jako
sterowniki sprzetu. Reszta to klasyczne OpenCV + nagrane ruchy + maszyna stanow.

## Wykrywanie szyszek - pyrealsense2 + OpenCV

- Kamera przez `pyrealsense2` (`pinecone_bot/camera.py`, import leniwy, tylko na Pi).
- Glowny stos (`pinecone_bot/detector.py`) wykrywa **po kolorze**: prog HSV (odcien + nasycenie)
  -> morfologia -> kontury -> filtr pola. Pierwsza w liscie = najnizej w obrazie = najblizej robota.
- Nowy prog (commit 2ba7fc9) daje 0 bledow w dwoch swiatlach; na Pi wciaz jest stary prog (V<95).
- Detekcja z samej glebi (plaszczyzna podloza + to, co nad nia wystaje) zostala w `scan_cones.py`,
  ale glowny stos jej nie uzywa.

## Ramie - lerobot 0.6.1 tylko jako sterownik serw SO-101

- Chwyt = odtwarzanie nagranego ruchu. Ramie prowadzone reka, `tools/record_motion.py` zapisuje
  pozycje 10 Hz do `motions/*.json`, `WaypointArm` (`pinecone_bot/arm.py`) odtwarza z interpolacja.
- Bez IK. Stare podejscie na `ikpy` + URDF lezy w `legacy/ik_approach/`.
- Baza podjezdza tak, zeby szyszka stanela w stalym miejscu obrazu (`target_row`), ramie robi
  zawsze ten sam nagrany ruch.
- `teleop_mirror.py`: leader prowadzony reka steruje followerem na zywo (klasy lerobot). Narzedzie,
  nie czesc autonomicznego zbierania.

## Jazda

- Kurs z zyroskopu telefonu przyklejonego do bazy (phyphox, `pinecone_bot/heading.py`), zamiast
  hallotronow z plyty hovera (plyta przerobiona i niedostepna). Na razie tylko symulacja.

## Calosc

- Maszyna stanow `pinecone_bot/brain.py` na Raspberry Pi 5: szukanie pasami -> detekcja ->
  dojazd -> chwyt. Kamera siedzi teraz na ramieniu.

## Stan

- Cale `pinecone_bot` NIE jechalo jeszcze na sprzecie (symulacja 5/5 szyszek).
- `grasp_near` nagrany tylko na Pi (nie w repo), nieodtworzony.
- `shoulder_lift` poza zakresem kalibracji.
- `lsusb` pokazuje kamere jako D435, dokumentacja mowi D415 - do sprawdzenia.
