# STATUS - na czym stoimy

Jeden ekran. Aktualizuje go KAZDY PR (checkbox w szablonie PR). Historia jest w `docs/LOG.md`,
zadania i przypisania na tablicy Projects (link nizej). Czego nie ma tutaj albo w issue, nie istnieje.

**Stan na:** 2026-09-27 15:00 (ACT: dataset 50 epizodow leaderem, trening na RTX 3070, checkpointy na Pi i masterze)
**Robot (kto ma sprzet, do kiedy):** frane (sesja trwa)
**Tablica zadan:** TODO wkleic link do GitHub Projects (zaklada pawel120, patrz docs/CONTRIBUTING.md)

## Dziala

- Ramie SO-101: skalibrowane po naprawie barku, `./arm.sh home|status|open|close`. NIE uruchamiac `lerobot calibrate`.
- Podwozie: Xiao + panel webowy (`python web_control.py`, WASD, osemka, pokrycie), `drive_step.py` do pojedynczych krokow.
- Kamera D415: podglad `rs_mjpeg_server.py` (glebia 424x240 -> mniejszy MinZ, bliski dywan ma ciagla glebie), detekcja szyszek z glebi (`scan_cones.py`, rozrzut < 2 mm).
- Nowy stos `pinecone_bot` (PR #14 + poprawki PR #16): symulacja na laptopie zbiera 5/5 szyszek, 66 testow zielonych.
  Ramie odtwarza nagrane punkty, baza ustawia szyszke z obrazu, maszyna stanow, szukanie pasami. Bez IK, bez ML.
- Detektor HSV: prog w branchu (commit 2ba7fc9, `pinecone_config.json`) rozdziela po odcieniu+nasyceniu: lo [130,20,20], hi [179,160,255], min_area_px 400, morph_ksize 9 -> 0 bledow w dwoch swiatlach (20 + 18 klatek kontrolnych, przeszukano 20160 kombinacji). Poprzedni prog V<95 rozdzielal po jasnosci i w drugim swietle gubil polowe szyszek (18/18 bledow) - NIEAKTUALNY. Na Pi wciaz jest stary prog (V<95, min_area 300, morph 7) - nowy jeszcze NIE wypchniety (laptop na chwile stracil siec do Pi).
- Polaczenie z Pi po WiFi: `robot.local` (mDNS, git-bash, nie PowerShell) albo `robot@172.20.10.4` (hotspot "iPhone pawel", DHCP), SSH ping 11-109 ms, klucz SSH laptopa juz na Pi (bez hasla). Pi ma tez eth0 192.168.137.5 (kabel).
- Panel webowy ramienia `tools/arm_web.py` (port 8010): jog kazdego stawu o 1/5/10, HOME, chwytak, ruchy z `motions/`, STOP.
  Jazda + ramie w jednym miejscu: sekcja ramienia w panelu jazdy (:8000, `frontend.html`), glowny STOP zatrzymuje tez ramie.
  Dwa procesy na Pi: `web_control.py` i `tools/arm_web.py` (UI ramienia wspolne: `arm_panel.js`).
  Logika w `pinecone_bot/arm_panel.py` (kolejka, zakres z kalibracji, limit kroku), 22 testy; sprawdzony w przegladarce na atrapie (`--fake`).
  `--no-home`: bez HOME przy starcie, HOME i `motions/` wylaczone, tylko jog i chwytak (kamera siedzi teraz na ramieniu - HOME w nia uderzy).
- Pasy po kursie (`cfg.heading`, `pinecone_bot/heading.py`, RUNBOOK "Pasy po kursie"): obroty do kata z zyroskopu telefonu (phyphox, remote access), na prostej regulator P kursu, bezpieczniki -> pasy z czasu. W symulacji z poslizgiem 15% koniec wzorca 0.10 m od idealu (bez kursu 3.5 m). `--heading phyphox|odometry|none`. Tylko symulacja i testy.
- `motions/grasp_mid.json`: chwyt z `demo2_fixed.csv` (aktualna kalibracja). `home.json`, `drop_box.json` (placeholder).
- `tools/record_motion.py` (commit 40a75aa): ciagle nagranie ruchu ramienia prowadzonego reka (bez jazdy do HOME, kamera na ramieniu), probki 10 Hz, 'q'+Enter konczy i oddaje torque, zapis `motions/<name>.json` (waypointy co 0.25 s w tempie prowadzenia, pierwszy z dojazdem 1.5 s); odtwarzanie `tools/arm_play.py --motion <name>`. Testy `tests/test_record_motion.py` (3). Zastapilo dla operatora `tools/record_waypoints.py` (punkt po punkcie, uciazliwe) i legacy `record_demo.py` (jazda do HOME, stala liczba sekund).

## Nie dziala / nie sprawdzone

- `pinecone_bot` NIE JECHAL jeszcze na sprzecie. Wszystko ponizej to pierwsze uruchomienie (docs/RUNBOOK.md).
- Nowy prog HSV (branch, commit 2ba7fc9) NIE jest jeszcze wpisany na Pi - do wypchniecia razem z blokada AWB/ekspozycji (`lock_auto`, sekcja "camera" configu, PR #30), ktora jest na masterze, ale NIE na Pi (`pinecone_bot/camera.py`/`config.py` na Pi sa starsze). Reka w kadrze ma podobny odcien co szyszka (bloby 9000-31500 px, szyszka max ~4000 px) - `max_area_px` 40000 tego nie odrzuca, warto zmniejszyc do ~8000 (niezmienione).
- `lsusb` zglasza kamere jako D435 (8086:0b07), docs mowia D415 - sprawdzic model.
- Kamera stoi za nisko: miejsce chwytu (17 cm przed kamera) jest w martwej strefie glebi (~31 cm). Trzeba przestawic.
- Panel ramienia (`tools/arm_web.py --no-home`) chodzi na Pi, jog NIE sprawdzony na ramieniu.
- `shoulder_lift` stoi poza zakresem kalibracji (odczyt 127.7 st, zakres +-91.6; kamera na ramieniu). Panel blokuje jog tego stawu - trzeba go ustawic recznie albo sprawdzic kalibracje pod nowy montaz (NIE `lerobot calibrate`).
- Hotspot: ping do Pi skacze do 240 ms i gubi pakiety, heartbeat panelu jazdy (1 s) co chwile wpada w failsafe (robot staje na chwile).
- Chwyty `grasp_far`, `drop_box` nie nagrane (config ma na razie tylko `grasp_mid`).
- `motions/grasp_near.json` nagrany NA PI (`tools/record_motion.py`, 112 waypointow, 33 s; chwytak 34 -> 1.3; `shoulder_lift` od -42 st przy chwycie do 121.7 st w pozie spoczynkowej - POZA zakresem kalibracji +-91.6, ticki 1006..3089, homing_offset 1977). Plik jest tylko na Pi (NIE w repo). NIE odtworzony - przed pierwszym `tools/arm_play.py --motion grasp_near` sprawdzic odczytem Min/Max_Position_Limit z serwa, czy limit pozycji w EEPROM nie utnie celu (bark moglby skoczyc ~30 st do granicy na starcie).
- Znak skretu Xiao i mapowanie PWM -> m/s niezmierzone (`cfg.base.xiao_*`, `cfg.control.steer_sign`).
- phyphox NIE sprawdzony na telefonie: czy iPhone jako hotspot wystawia serwer pod 172.20.10.1:8080, znak kursu (`heading.sign`), czy ekran nie gasnie. Test: `python tools/phyphox_check.py` na Pi, obrot recznie o 90 st.
- Bipropellant na plycie hovera: plyta jest przerobiona i niedostepna (2026-09-27), wiec hallotronow nie bedzie; kurs z telefonu zamiast nich. Stary test (nieaktualny):
  `python tools/bip_probe.py --port /dev/ttyAMA0` (nie rusza silnikow, sprawdza ASCII i protokol binarny na 3 baudach).
- WiFi na Pi DZIALA (wczesniej ten plik mowil, ze nie): eth0 192.168.137.5 (kabel) i wlan0 172.20.10.4 (hotspot "iPhone pawel", DHCP - adres moze sie zmienic). Kod na Pi nadal wchodzi przez `deploy/push_to_pi.sh` / scp (internet/`git pull` na Pi niesprawdzone).
- Po restarcie Pi zadne panele nie wstaja same: `web_control.py` i `tools/arm_web.py` trzeba odpalac recznie, `robot-web.service` nie jest zainstalowany. Nadal nie odpalone w tej sesji.
- Zasilanie z akumulatora 12 V: issue #8, nie zaczete.
- Polityki uczone (ML): decyzja frane 2026-09-27 - robimy lerobot ACT rownolegle (`docs/POLICIES_LEROBOT.md`, SETUP.md sekcja "lerobot do ACT").
  Laptop GOTOWY: torch 2.11 cu128 (CUDA na RTX 3070), lerobot 0.6.1 [phone,feetech,async], `so101.json` skopiowany z Pi.
  Pi GOTOWE: lerobot 0.6.1 + placo (IK), hebi-py/teleop (telefon), grpcio (async client), torchvision; importy i IK na URDF sprawdzone.
  `~/hackaton/examples/phone_to_so100/` na Pi (skrypty v0.6.1 + SO101 z STL, poza gitem). Teleop telefonem, druga kamera,
  nagranie datasetu: nic nie odpalone. JEST leader SO-101 (sala 435 D) -> sciezka glowna to `lerobot-record` z leaderem
  (SETUP.md "Wariant z leaderem"), telefon/placo tylko awaryjnie. Leader bez kalibracji; nowa regula udev `robot-leader`
  (po serialu CH343) w `deploy/99-robot.rules`, WGRANA na Pi.
- ACT (2026-09-27 po poludniu): dataset `datasets/so101_grasp2` (50 epizodow, 22451 klatek, kamera wrist D435 serial
  030522070668, leader skalibrowany 12:30) NAGRANY na Pi przez druga sesje i wrzucony na master (wideo w Git LFS).
  Trening na laptopie (`tools/train_win.py`, batch 8, AMP, ~3 kroki/s po podpieciu zasilacza): loss 26 -> 1.24 przy
  kroku 2000 (l1 0.31). Checkpointy 1000/2000 na Pi w `~/models/act_so101_grasp2/<krok>/pretrained_model`, 1000 tez
  na masterze (`models/`, LFS). Rollout lokalnie na Pi (0.65 s na 100 akcji): komenda w SETUP.md "Rollout polityki
  ACT na Pi". NIE sprawdzone na robocie w chwili pisania.
- Zera stawow vs URDF i kamera na ramieniu: `tools/frame_check.py` (FK placo + werdykt operatora) i
  `tools/hand_eye_calib.py` (marker/collect/solve/predict, AX=XB) gotowe z testami (28), instrukcja `docs/ARM_FRAMES.md`.
  NIE uruchomione na sprzecie - do zrobienia przez sesje przy Pi. To warunek wstepny dla IK/GraspGenX.

## Nastepne 3 kroki (w tej kolejnosci)

0. (sesja przy Pi, rownolegle z ACT) `docs/ARM_FRAMES.md`: `tools/frame_check.py` z wylacznikiem, potem
   `tools/hand_eye_calib.py collect/solve` z markerem ArUco -> `camera_on_arm.json` do repo.
1. Wpisac nowy prog HSV na Pi (albo push z brancha po merge) i sprawdzic na zywo; odczytac limity EEPROM barku, potem `tools/arm_play.py --motion grasp_near` z reka na wylaczniku.
2. Nagrac `drop_box` (`tools/record_motion.py --name drop_box`), dopisac chwyty do `cfg.grasps`, `tools/calibrate_target.py`.
3. `tools/base_test.py`, `tools/phyphox_check.py` (znak kursu), potem `python -m pinecone_bot.main --dry-run --heading phyphox`, potem `--real` z wylacznikiem w rece.

## Blokery

- Merge PR #16 (poprawki z issue #15) - bez tego `--real` konczy sie bledem przy chwycie near/far.
- Branch protection na master i tablica Projects: do zrobienia przez pawel120 (docs/CONTRIBUTING.md).

## Kto co robi (obszary; wpiszcie nazwiska)

| Obszar | Osoba | Biezace issue |
|---|---|---|
| baza / hover (Xiao, bipropellant, base_test) | ? | |
| ramie (nagrania chwytow, record_waypoints) | ? | |
| wizja + kalibracja (kamera, HSV, calibrate_target) | ? | |
| integracja + docs (brain, testy, STATUS/LOG) | ? | |
| elektryka (zasilanie 12 V, e-stop) | ? | #8 |
