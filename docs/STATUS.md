# STATUS - na czym stoimy

Jeden ekran. Aktualizuje go KAZDY PR (checkbox w szablonie PR). Historia jest w `docs/LOG.md`,
zadania i przypisania na tablicy Projects (link nizej). Czego nie ma tutaj albo w issue, nie istnieje.

**Stan na:** 2026-09-27 15:00 (ACT: dataset 50 epizodow leaderem, trening na RTX 3070, checkpointy na Pi i masterze; mapa ogrodu D435 + zygzak po mapie - `docs/MAPA.md`)
**Robot (kto ma sprzet, do kiedy):** frane (sesja trwa)
**Tablica zadan:** TODO wkleic link do GitHub Projects (zaklada pawel120, patrz docs/CONTRIBUTING.md)

## Dziala

- Ramie SO-101: skalibrowane po naprawie barku, `./arm.sh home|status|open|close`. NIE uruchamiac `lerobot calibrate`.
- Podwozie: Xiao + panel webowy (`python web_control.py`, WASD, osemka, pokrycie), `drive_step.py` do pojedynczych krokow.
- Panel zbiorczy `tools/robot_panel.py` (:8090, PR #52): jedno okno SSH zamiast trzech. Sam odpala
  jazde (`web_control.py`), ramie (`tools/arm_web.py --no-home`) i kamere (`tools/vision_web.py`, :8020), pokazuje
  podglad z ramkami szyszek, liczbe szyszek w kadrze + wykres z minuty, odleglosc z glebi, pasek "jak widzi robot"
  (obraz -> maska HSV -> szyszki -> glebia), tarcze stawow, WASD, STOP (spacja: jazda + ramie), uslugi start/stop/restart,
  logi na zywo i ZIP (`/logs.zip`, z `pinecone_log.csv`), zdrowie Pi, zdarzenia, "co zbudowalismy" (liczby z repo).
  `/show` = tryb pokazu na projektor (bez sterowania, NIE trzyma heartbeatu jazdy). Sprawdzony na laptopie w `--demo`
  (ramie-atrapa, kamera z symulatora): liczenie, restart uslug, logi, telefon 375 px. 20 nowych testow, 164 zielone.
- Podglad kamery OBOK lerobot: `tools/cam_preview.py lerobot-record|lerobot-rollout ...` (port 8081) - serwer MJPEG
  w tym samym procesie, podglada `read_latest()` kamery lerobot (nie zajmuje jej drugi raz). `act_pick.py` uzywa go
  domyslnie. Testy 6 + sprawdzone z prawdziwym lerobot `OpenCVCamera` na laptopie; NA PI z RealSense NIE sprawdzone.
- Kamera D415: podglad `rs_mjpeg_server.py` (glebia 424x240 -> mniejszy MinZ, bliski dywan ma ciagla glebie), kolory glebi jak w RealSense Viewer (`--colormap viewer`, domyslnie; stara skala liniowa: `--colormap fixed`), detekcja szyszek z glebi (`scan_cones.py`, rozrzut < 2 mm).
- Nowy stos `pinecone_bot` (PR #14 + poprawki PR #16): symulacja na laptopie zbiera 5/5 szyszek, 66 testow zielonych.
  Ramie odtwarza nagrane punkty, baza ustawia szyszke z obrazu, maszyna stanow, szukanie pasami. Bez IK, bez ML.
- Detektor HSV: prog w branchu (commit 2ba7fc9, `pinecone_config.json`) rozdziela po odcieniu+nasyceniu: lo [130,20,20], hi [179,160,255], min_area_px 400, morph_ksize 9 -> 0 bledow w dwoch swiatlach (20 + 18 klatek kontrolnych, przeszukano 20160 kombinacji). Poprzedni prog V<95 rozdzielal po jasnosci i w drugim swietle gubil polowe szyszek (18/18 bledow) - NIEAKTUALNY. Na Pi wciaz jest stary prog (V<95, min_area 300, morph 7) - nowy jeszcze NIE wypchniety (laptop na chwile stracil siec do Pi).
- Polaczenie z Pi po WiFi: `robot.local` (mDNS, git-bash, nie PowerShell) albo `robot@172.20.10.4` (hotspot "iPhone pawel", DHCP), SSH ping 11-109 ms, klucz SSH laptopa juz na Pi (bez hasla). Pi ma tez eth0 192.168.137.5 (kabel).
- Wylacznik na telefon: `http://<ip-pi>:8000/stop` (`stop.html`, serwuje `web_control.py`). Jeden duzy STOP: zatrzask, jazda zablokowana
  (klawisze, tryby auto, sekwencje) do ODBLOKUJ na tej stronie, STOP idzie tez do ramienia. Zwykly HTTP, nie heartbeat: telefon na /stop
  nie trzyma robota przy zyciu. Pokazuje lacze (ms) i czy robot jedzie. Sprawdzone w przegladarce bez Xiao, 8 testow; NIE na Pi.
- Panel webowy ramienia `tools/arm_web.py` (port 8010): jog kazdego stawu o 1/5/10, HOME, chwytak, ruchy z `motions/`, STOP.
  Jazda + ramie w jednym miejscu: sekcja ramienia w panelu jazdy (:8000, `frontend.html`), glowny STOP zatrzymuje tez ramie.
  Dwa procesy na Pi: `web_control.py` i `tools/arm_web.py` (UI ramienia wspolne: `arm_panel.js`).
  Logika w `pinecone_bot/arm_panel.py` (kolejka, zakres z kalibracji, limit kroku), 22 testy; sprawdzony w przegladarce na atrapie (`--fake`).
  `--no-home`: bez HOME przy starcie (kamera siedzi teraz na ramieniu - HOME w nia uderzy); jog, chwytak i ruchy z `motions/` dzialaja,
  po pustym chwycie ramie zostaje w miejscu zamiast wracac do HOME.
- Pasy po kursie (`cfg.heading`, `pinecone_bot/heading.py`, RUNBOOK "Pasy po kursie"): obroty do kata z zyroskopu telefonu (phyphox, remote access), na prostej regulator P kursu, bezpieczniki -> pasy z czasu. W symulacji z poslizgiem 15% koniec wzorca 0.10 m od idealu (bez kursu 3.5 m). `--heading phyphox|odometry|none`. Telefon sprawdzony 2026-09-27: iPhone-hotspot, phyphox na `http://172.20.10.1` (iOS: port 80, nie 8080), Pi dostaje kurs, obrot recznie 90 st w lewo -> +90 (`sign` 1.0 dobry). Jazda po kursie NIE sprawdzona.
- `--dry-run --heading phyphox --source ~/pusty.png` na Pi (telefon obracany recznie): pelny obrot konczy sie na 358 st, prosta trzyma kurs, skret liczy kat; wolny obrot reczny (28 s) wlacza bezpiecznik -> pasy z czasu. Pusty obraz, bo prawdziwa kamera widziala 2 falszywe szyszki i mozg nie wchodzil w SEARCH.
- Skret hovera zmierzony zyroskopem (`tools/calibrate_turn.py`): ujemne b = w lewo (`steer_sign` +1 dobry); rusza od |b| 100-160 (160, gdy stal), 10 wyzej = +0.3..0.7 rad/s; `--response 160`: opoznienie 0.15-0.4 s, rozpedzanie 0.15 s, 0.43-1.08 rad/s przy tym samym b, wybieg 3-10 st. Stala tabela `xiao_steer_min/max` tego nie opisze.
- Petla obrotu na zyroskopie (`pinecone_bot/turn_loop.py`, `heading.rate_*`): PWM skretu z predkosci mierzonej telefonem. Symulator `--hover` (model z pomiarow): pasy 0.01-0.07 m od idealu w calym zmierzonym rozrzucie; bez petli robot sie nie obraca. Na robocie NIE sprawdzona.
- Ramie przez panel (`tools/arm_web.py --no-home`, API `/api/cmd`): jog wszystkich stawow i chwytaka dziala (2026-09-27).
- Zbieranie szyszek "na sztywno" z panelu (:8000), bez kodu: (1) sekcja ramienia "NAGRYWANIE RUCHU": ustaw stawami, "+ PUNKT" (chwytak z ostatniej
  komendy, wiec przed punktem zacisku "Zamknij chwytak"), "ZAPISZ do motions/" -> `motions/<nazwa>.json`; (2) sekcja "SEKWENCJA": kroki jazda
  (speed/steer/sekundy, bez limitu z suwaka) / ramie (ruch z motions/) / czekaj, "TEST TEGO KROKU", szkic w przegladarce, zapis do `sequences/<nazwa>.json`,
  odtwarzanie w trybie "sequence" (`pinecone_bot/sequence.py`, 12 testow; STOP/failsafe/zmiana trybu przerywa i zeruje jazde + STOP ramienia).
  Sprawdzone w przegladarce na atrapie ramienia (`--fake`) i bez Xiao; NIE na sprzecie.
- Polaczenie laptop -> Pi (hotspot albo kabel) i odpalenie obu paneli (:8000 jazda, :8010 ramie): poradnik `docs/PANEL.md`, sprawdzone 2026-09-26. Na Pi `pytest` 136 zielonych (bez `test_calibrate_target.py`), `./arm.sh status` czyta 6 przegubow.
- `motions/grasp_mid.json`: chwyt z `demo2_fixed.csv` (aktualna kalibracja). `home.json`, `drop_box.json` (placeholder).
- `tools/record_motion.py` (commit 40a75aa): ciagle nagranie ruchu ramienia prowadzonego reka (bez jazdy do HOME, kamera na ramieniu), probki 10 Hz, 'q'+Enter konczy i oddaje torque, zapis `motions/<name>.json` (waypointy co 0.25 s w tempie prowadzenia, pierwszy z dojazdem 1.5 s); odtwarzanie `tools/arm_play.py --motion <name>`. Testy `tests/test_record_motion.py` (3). Zastapilo dla operatora `tools/record_waypoints.py` (punkt po punkcie, uciazliwe) i legacy `record_demo.py` (jazda do HOME, stala liczba sekund).

## Nie dziala / nie sprawdzone

- Jog XYZ w panelu ramienia (`pinecone_bot/kinematics.py`, sekcja JOG XYZ): testy + atrapa, NIE sprawdzony na ramieniu. Najpierw ZERO URDF (ramie prosto poziomo do przodu), potem sprawdzic, czy GORA jedzie w gore (inaczej `arm.urdf_sign`).
- 2026-09-26: ROBOT WJECHAL W RAMIE I JE USZKODZIL (panel jazdy po hotspocie z duzym opoznieniem). Stan ramienia do oceny, serwa nie zasilac przed ogledzinami. Pi przestal odpowiadac (ping 100% strat).
- `tools/drive_calib.py` (branch pawel/drive-calib, draft PR): kalibracja jazdy bez miarki - droga z glebi RealSense
  do sciany, kat z phyphox, pytanie l/p -> xiao_pwm_*/xiao_steer_*, steer_sign, heading.sign. Testy 9, NA PI NIE
  uruchomione. Prawdopodobnie dubluje `base_test.py --measure` (pawel/base-calibration), `turn_loop.py`
  (frane/gyro-rate-loop) i jazde po mapie `zygzak.py` (frane/mapa-d435) - przed uzyciem zdecydowac, co zostaje.

- `pinecone_bot` NIE JECHAL jeszcze na sprzecie. Wszystko ponizej to pierwsze uruchomienie (docs/RUNBOOK.md).
- Nowy prog HSV (branch, commit 2ba7fc9) NIE jest jeszcze wpisany na Pi - do wypchniecia razem z blokada AWB/ekspozycji (`lock_auto`, sekcja "camera" configu, PR #30), ktora jest na masterze, ale NIE na Pi (`pinecone_bot/camera.py`/`config.py` na Pi sa starsze). Reka w kadrze ma podobny odcien co szyszka (bloby 9000-31500 px, szyszka max ~4000 px) - `max_area_px` 40000 tego nie odrzuca, warto zmniejszyc do ~8000 (niezmienione).
- 2026-09-27 popoludnie: Xiao ODPIETY - w jego USB siedzi leader SO-101 (`/dev/robot-leader`, nagrywanie ACT). Przed jazda:
  Xiao z powrotem, `pkill -f web_control.py; pkill -f lerobot`, `ls -l /dev/robot-*` ma pokazac `robot-drive`.
- `pawel/drive-calib` (eefa5a4, bez PR, nie uruchomiony): `tools/drive_calib.py` dubluje `calibrate_drive/turn` i zygzak
  (ten sam sie uczy predkosci ze zdjec). Decyzja: porzucic albo wziac tylko dopasowanie PWM/znakow.
- Kamera na robocie to D435 (sprawdzone pyrealsense2, fw 5.11.1.100) - pomiary glebi w HARDWARE 16-27 byly na D415.
- Mapa ogrodu (branch frane/mapa-d435): `tools/record_rgbd.py` na Pi (15 Hz, zero dziur) -> kopia na laptop ->
  `tools/rtabmap_build.py` (RTAB-Map 0.23.8 win64 w `C:/Users/pawel/tools/bin`) -> `python -m pinecone_bot.localize build`.
  ogrod1 i ogrod2: mapa rozpada sie na 5-9 kawalkow, w mapie ok. 120 poz z pierwszych 2/3 nagrania. Lokalizacja
  z jednego zdjecia: mediana 3-4 cm / 1 st, ale tylko 45-58% klatek, 90% bledow < 0.4 m. Ze startu przy plocie
  (ogrod1) NIE lapala - tego fragmentu nie bylo w mapie. ogrod2 na zywo niesprawdzona (Pi zajety `lerobot-record`).
- Zygzak po mapie `python -m pinecone_bot.zygzak` (kod na Pi w `~/mapa_test`, nie w `~/hackaton`): tylko dry-run do
  pierwszej lokalizacji (nie zlapal). NIE JECHAL. Szyszki w zygzaku niepodpiete (potrzebna poza "szukaj" ramienia).
- Pi padl w trakcie kopiowania nagrania (restart, uptime 3 min) - zasilanie. Kopia nagrania: `scp -r` przerwane zostawia dziury
  w srodku (kolejnosc plikow dowolna); `rtabmap_build.py` teraz to wykrywa.
- Kamera stoi za nisko: miejsce chwytu (17 cm przed kamera) jest w martwej strefie glebi (~31 cm). Trzeba przestawic.
- Panel ramienia (`tools/arm_web.py --no-home`) chodzi na Pi, jog NIE sprawdzony na ramieniu. Nagrywanie ruchu z panelu i sekwencje
  (jazda + ramie) tylko na atrapie; na Pi trzeba zrestartowac oba serwery (`web_control.py` woli `http://127.0.0.1:8010`, env `ROBOT_ARM_PANEL`).
- `shoulder_lift` stoi poza zakresem kalibracji (odczyt 127.7 st, zakres +-91.6; kamera na ramieniu). Panel blokuje jog tego stawu - trzeba go ustawic recznie albo sprawdzic kalibracje pod nowy montaz (NIE `lerobot calibrate`).
- Hotspot: ping do Pi skacze do 240 ms i gubi pakiety, heartbeat panelu jazdy (1 s) co chwile wpada w failsafe (robot staje na chwile).
- Chwyty `grasp_far`, `drop_box` nie nagrane (config ma na razie tylko `grasp_mid`).
- `motions/grasp_near.json` nagrany NA PI (`tools/record_motion.py`, 112 waypointow, 33 s; chwytak 34 -> 1.3; `shoulder_lift` od -42 st przy chwycie do 121.7 st w pozie spoczynkowej - POZA zakresem kalibracji +-91.6, ticki 1006..3089, homing_offset 1977). Plik jest tylko na Pi (NIE w repo). NIE odtworzony - przed pierwszym `tools/arm_play.py --motion grasp_near` sprawdzic odczytem Min/Max_Position_Limit z serwa, czy limit pozycji w EEPROM nie utnie celu (bark moglby skoczyc ~30 st do granicy na starcie).
- Jazda do przodu: mapowanie PWM -> m/s niezmierzone (`xiao_pwm_min/max`). `tools/calibrate_drive.py` (glebia do sciany przed i po jezdzie) raz odpalone z kamera patrzaca w sufit - wynik bez sensu; kamera ustawiona poziomo, pomiar do powtorzenia.
- Podjazd do szyszki na modelu hovera: regulator P w obrazie + tarcie + opoznienie telefonu oscyluje (1-5/5 zaleznie od parametrow). Pomysl: celowanie krokami (kat z obrazu, obrot o kat po zyroskopie, stop, patrz). Nie zaczete.
- `pinecone_bot/landmarks.py` na Pi to same zera (uszkodzony); oryginal w lokalnym commicie dafd481 (`pawel/base-calibration`).
- Bipropellant na plycie hovera: plyta jest przerobiona i niedostepna (2026-09-27), wiec hallotronow nie bedzie; kurs z telefonu zamiast nich. Stary test (nieaktualny):
  `python tools/bip_probe.py --port /dev/ttyAMA0` (nie rusza silnikow, sprawdza ASCII i protokol binarny na 3 baudach).
- WiFi na Pi DZIALA (wczesniej ten plik mowil, ze nie): eth0 192.168.137.5 (kabel) i wlan0 172.20.10.4 (hotspot "iPhone pawel", DHCP - adres moze sie zmienic). Kod na Pi nadal wchodzi przez `deploy/push_to_pi.sh` / scp (internet/`git pull` na Pi niesprawdzone).
- Panel zbiorczy (`tools/robot_panel.py --autostart`) NIE uruchomiony na Pi: RealSense przez `tools/vision_web.py`,
  temperatura/`get_throttled`, `/dev/robot-*` i zatrzymanie uslug SIGINT (Xiao dostaje `a0 b0`) sprawdzone tylko w kodzie.
  Tarcze stawow pokazuja odczyt wzgledem zakresu kalibracji, nie sylwetke ramienia (zera vs URDF niesprawdzone).
- Po restarcie Pi zadne panele nie wstaja same: `web_control.py` i `tools/arm_web.py` trzeba odpalac recznie, `robot-web.service` nie jest zainstalowany. Nadal nie odpalone w tej sesji.
- Zasilanie z akumulatora 12 V: issue #8, nie zaczete.
- Wylacznik /stop nie wdrozony na Pi (trzeba restartu `web_control.py`). Przy lagu hotspotu > 1-2 s STOP z telefonu tez dojdzie pozno:
  fizyczny wylacznik dalej w rece. Zatrzask nie blokuje panelu ramienia (:8010) - STOP ramienia idzie raz.
- Sciezka S w `web_control.py` (POKRYCIE): nawroty naprzemienne (L, P, L...) poprawione w kodzie, NIE jechane na sprzecie.
  Do nastrojenia na trawie: `cov_turn_seconds` (90 st), `cov_forward_seconds`, `cov_lane_seconds`.
- Polityki uczone (ML): decyzja frane 2026-09-27 - robimy lerobot ACT rownolegle (`docs/POLICIES_LEROBOT.md`, SETUP.md sekcja "lerobot do ACT").
  Laptop GOTOWY: torch 2.11 cu128 (CUDA na RTX 3070), lerobot 0.6.1 [phone,feetech,async], `so101.json` skopiowany z Pi.
  Pi GOTOWE: lerobot 0.6.1 + placo (IK), hebi-py/teleop (telefon), grpcio (async client), torchvision; importy i IK na URDF sprawdzone.
  `~/hackaton/examples/phone_to_so100/` na Pi (skrypty v0.6.1 + SO101 z STL, poza gitem). Teleop telefonem, druga kamera,
  JEST leader SO-101 -> `lerobot-record` z leaderem (SETUP.md "Wariant z leaderem"), telefon/placo tylko awaryjnie.
  2026-09-27: leader na Pi jako `/dev/robot-leader` (w miejscu kabla hovera - Pi nie ma wolnego USB), kalibracja leadera
  = KOPIA `so101.json` followera (decyzja frane) + gripper przez `tools/calibrate_joint.py`; czesc osi leadera odwrocona
  (dane dla ACT i tak poprawne: akcja = cel followera). Kamera D435 serial 030522070668 (nie 105422060821 z SETUP).
  Podzial na 2 etapy: ACT uczy sie TYLKO chwytu (HOME -> szyszka -> zamkniecie -> uniesienie, 15 s), wrzut do sloika
  robi nagrany `motions/drop_box.json` (tylko na Pi: tam i z powrotem, 15 s). Datasety na Pi `~/datasets/`:
  `so101_grasp` (7 ep.), `so101_grasp2` (50 ep., gotowy). Na Pi doinstalowane `lerobot[dataset]` z torch cpu przypietym
  (override), spadek napiecia uszkodzil `pyarrow`/`av` - przeinstalowane. `tools/act_pick.py` = etap 1 (`lerobot-rollout`
  z ACT, torque zostaje) + etap 2 (`arm_play drop_box --home-after`), testy 7, podglad kamery :8081; NIE uruchomiony (wagi sa, patrz nizej).
  Laptop pawel120 (Intel Arc, bez NVIDIA, bez venv) NIE nadaje sie do treningu - trening na laptopie z RTX 3070.
- ACT (2026-09-27 po poludniu): dataset `datasets/so101_grasp2` (50 epizodow, 22451 klatek, kamera wrist D435 serial
  030522070668, leader skalibrowany 12:30) NAGRANY na Pi przez druga sesje i wrzucony na master (wideo w Git LFS).
  Trening na laptopie (`tools/train_win.py`, batch 8, AMP, ~3 kroki/s po podpieciu zasilacza): loss 26 -> 1.24 przy
  kroku 2000 (l1 0.31). Checkpointy 1000/2000 na Pi w `~/models/act_so101_grasp2/<krok>/pretrained_model`, 1000 tez
  na masterze (`models/`, LFS). Rollout lokalnie na Pi (0.65 s na 100 akcji): komenda w SETUP.md "Rollout polityki
  ACT na Pi". NIE sprawdzone na robocie w chwili pisania.
  15:24: run2 padl przy 3528 (restart sesji); wznowienie z 3000 (`--config_path=.../003000/pretrained_model/
  train_config.json --resume=true`, bez `last`) doszlo do 4000 (loss 0.575 (l1 0.255)), checkpoint `C:/Users/frane/outputs/
  act_so101_grasp2_run2/checkpoints/004000`. 16:05: trening DOKONCZONY do 7000 (loss 0.301, l1 0.198); checkpointy
  5000/6000/7000 w tym samym katalogu na laptopie, NIE na Pi, NIE w LFS (limit 1 GB) - na Pi przez scp (SETUP.md).
  Pulapka: `import torch` pada w sandboxie narzedzia Claude Code (WinError 1114 shm.dll) - trening poza sandboxem,
  JEDEN proces naraz (dwie sesje naraz dzielily GPU i katalog, LOG 15:20). Checkpointy 2000/3000 w LFS tylko
  lokalnie (branch `frane/act-training`, push nie doszedl; limit LFS 1 GB).
- Zera stawow vs URDF i kamera na ramieniu: `tools/frame_check.py` (FK placo + werdykt operatora) i
  `tools/hand_eye_calib.py` (marker/collect/solve/predict, AX=XB) gotowe z testami (28), instrukcja `docs/ARM_FRAMES.md`.
  NIE uruchomione na sprzecie - do zrobienia przez sesje przy Pi. To warunek wstepny dla IK/GraspGenX.

- 2026-09-26 wieczor: chwytanie samym ramieniem (baza stoi), instrukcja: `docs/LIVE_GRASP.md`, kod `tools/live_grasp/`.
  Dziala: szyszki z glebi w HOME, nagrywanie ruchu reka, odtworzenie cyklu chwyt -> sloik. Nie dziala: autonomiczny chwyt
  (model z 8 niepewnych probek, blad ok. 6 cm). Nowy HOME w repo, NIE wgrany na Pi.

## Nastepne 3 kroki (w tej kolejnosci)

Dwa tory rownolegle. Tor mapa + zygzak (frane/mapa-d435, `docs/MAPA.md`):
1. Lokalizacja ze startu: `tools/arm_hold.py patrz`, zdjecie z miejsca startu zygzaka, `localize` na ogrod2. Nie lapie ->
   lepsza mapa (strojenie odometrii RTAB-Map na zewnatrz albo nagranie krotsze i wolniejsze, tylko pole zygzaka).
2. Zygzak `--dry-run`, potem `--real` na 2 pasach po 2 m (STOP z `~/mapa_test/tools/estop_server.py`, zna zygzak).
3. Poza ramienia "szukaj" (kamera 38 st w dol) i szyszki w zygzaku (detektor + podjazd z `brain.py`); merge frane/mapa-d435.

Tor ACT / ramie (master):
A. (ACT) wagi `pretrained_model` na Pi, `python tools/act_pick.py --policy <katalog> --skip-drop`
   z wylacznikiem w rece, potem bez `--skip-drop`.
0. (sesja przy Pi, rownolegle z ACT) `docs/ARM_FRAMES.md`: `tools/frame_check.py` z wylacznikiem, potem
   `tools/hand_eye_calib.py collect/solve` z markerem ArUco -> `camera_on_arm.json` do repo.
0. Chwytanie samym ramieniem: czyste uczenie (1 szyszka naraz, jeden styl chwytu, 12 pozycji), `fit.py`, test `pick.py`
   - dokladne kroki w `docs/LIVE_GRASP.md`. Swiatlo w pokoju konieczne.
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
