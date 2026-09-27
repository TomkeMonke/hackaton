# RUNBOOK - jak odpalic robota

Krok po kroku na dzien ze sprzetem. Stan projektu jest w `docs/STATUS.md`, setup w `docs/SETUP.md`,
pulapki sprzetowe w `docs/HARDWARE.md`. Ten plik opisuje stos `pinecone_bot`; stary panel webowy
do recznej jazdy odpala sie osobno (`python web_control.py`).

## Checklista na rano (pierwsze uruchomienie pinecone_bot na sprzecie)

Kolejnosc ma znaczenie: kazdy krok zapisuje cos, z czego korzysta nastepny.

- [ ] `git pull` na laptopie (master z PR #14 i #16), `deploy/push_to_pi.sh`, na Pi `python -m pytest tests -q`.
- [ ] **Kamera na maszt, wyzej i za ramie**, patrzy w dol. Zmierz miarka, ile cm przed osia kol chwytak
      zamyka sie w `grasp_mid` (i gdzie beda near/far), potem `python tools/camera_geometry.py --grasp-forward 0.30,0.35,0.40 --cam-forward -0.10`
      (tabela wysokosc x kat; komorki OK = chwyty w kadrze i poza martwa strefa glebi). Startowy typ: 0.45 m, 38 st, 10 cm za osia.
      Po zamontowaniu sprawdz w `rs_mjpeg_server.py`, ze widac szyszke lezaca w miejscu, gdzie `grasp_mid` ja podnosi.
- [ ] `python tools/snap_frames.py --out frames/ --every 0.5 --seconds 20` na prawdziwej trawie z szyszkami.
- [ ] `python tools/calibrate_hsv.py --source frames/`, klawisz `s` zapisuje prog do `pinecone_config.json`.
- [ ] `python tools/arm_play.py --motion grasp_mid --port /dev/robot-arm --dry-run`, potem bez `--dry-run`
      z szyszka w miejscu chwytu: czy chwyt trafia i czy po zacisku `gripper` czyta powyzej 6?
- [ ] `python tools/record_waypoints.py --name grasp_near ...`, `grasp_far`, `drop_box` (pojemnik zamontowany?).
- [ ] `python tools/calibrate_target.py`: szyszka dokladnie w miejscu kazdego chwytu, `s` na chwyt, `c` na kolumne, `w` zapis.
- [ ] `python tools/base_test.py --driver xiao --port /dev/robot-drive turn --seconds 2 --w 0.5`:
      robot ma skrecic W LEWO. Skreca w prawo? `cfg.control.steer_sign = -1`.
- [ ] `base_test forward --seconds 2 --v 0.15` i pomiar miarka -> `cfg.base.xiao_pwm_min/max`; rozstaw kol -> `wheel_base_m`.
- [ ] `python -m pinecone_bot.main --dry-run` (kamera prawdziwa, komendy tylko drukowane): czy `v`/`w` maja sens?
- [ ] `python -m pinecone_bot.main --real` z wylacznikiem w rece, jedna szyszka, potem piec.
- [ ] Wynik (co zadzialalo, co nie, liczby) do `docs/STATUS.md` i `docs/LOG.md`.
- [ ] Jesli jest czas: `python tools/bip_probe.py --port /dev/ttyAMA0` (przejsciowka USB-UART na zlacze plytki bocznej
      hovera; nie rusza silnikow). Odpowiada? Sekcja "Hoverboard" nizej mowi, co przestawic w configu.

## Idea w 5 zdaniach

1. Ramie jest "glupie": powtarza nagrane wczesniej ruchy (waypointy) po
   nazwie, bez IK, bez planowania trajektorii.
2. Baza jest "sprytna": ustawia szyszke w dokladnie tym miejscu obrazu, w
   ktorym nagrany chwyt ja podnosi - obracajac sie, az kolumna szyszki w
   obrazie rowna sie `cfg.cx`, i jadac, az wiersz szyszki rowna sie
   `grasp.target_row` - wylacznie na podstawie obrazu kolorowego (glebia nie
   jest do tego uzywana).
3. Cala logika to prosta maszyna stanow: SEARCH (szukaj obracajac sie) ->
   APPROACH (jedz/obracaj sie regulatorem P) -> ALIGN (potwierdz na stojaco)
   -> GRASP (zamknij chwytak) -> DROP (wrzuc do pojemnika) -> RETRY (probuj
   ponownie) -> z powrotem do SEARCH.
4. Zero uczenia maszynowego, zero LLM, zero promptow - kazda decyzja jest
   zakodowana w `pinecone_bot/brain.py` i da sie ja przeczytac linijka po
   linijce.
5. Kazda klatka i kazda decyzja trafia do pliku CSV (`pinecone_log.csv`),
   wiec kazdy nieudany przejazd da sie odtworzyc z logu, bez zgadywania.

## Struktura

| Plik / katalog                  | Co robi |
|----------------------------------|---------|
| `pinecone_bot/config.py`         | Jeden dataclass `Config` z cala konfiguracja (progi HSV, gainy regulatora, porty, geometria symulatora). Zapis/odczyt z `pinecone_config.json`. |
| `pinecone_bot/brain.py`          | Maszyna stanow + regulator P. Caly "mozg" robota, patrz docstring na gorze pliku. |
| `pinecone_bot/detector.py`       | Detektor szyszek: prog HSV -> maska -> kontury -> lista `Detection`. |
| `pinecone_bot/camera.py`         | Zrodla obrazu: `RealSenseCamera` (D415), `FileCamera` (pliki/katalog/wideo do strojenia na sucho). |
| `pinecone_bot/base.py`           | Sterowniki podwozia: `sim`, `xiao` (Seeed Xiao + H-bridge), `bipropellant` (UART do plyty hoverboarda). |
| `pinecone_bot/arm.py`            | Odtwarzacz ruchow ramienia: `sim`, `waypoints` (prawdziwe SO-101 przez lerobot), `subprocess` (skrypt zespolu). |
| `pinecone_bot/sim.py`            | Symulator geometrii kamery + swiata, do rozwijania calej petli bez sprzetu. |
| `pinecone_bot/main.py`           | CLI: `--sim`, `--dry-run`, `--real`. |
| `tools/snap_frames.py`           | Zapis klatek z kamery do PNG (material do strojenia detektora). |
| `tools/calibrate_hsv.py`         | Interaktywne strojenie progu HSV, zapis do configu. |
| `tools/calibrate_target.py`      | Pomiar `cfg.cx` i `grasp.target_row` dla kazdego nagranego chwytu. |
| `tools/record_waypoints.py`      | Nagrywanie ruchu ramienia (waypointy) recznym ustawianiem serw. |
| `tools/frame_check.py`           | Zera i znaki stawow lerobot vs URDF przez FK placo, werdykt operatora na staw (docs/ARM_FRAMES.md). |
| `tools/hand_eye_calib.py`        | Kalibracja reka-oko kamery na ramieniu (ArUco + calibrateHandEye) -> `camera_on_arm.json`. |
| `tools/arm_play.py`              | Odtworzenie jednego nagranego ruchu (do testu bez calej petli). |
| `tools/arm_web.py`               | Panel webowy ramienia na :8010 (jog stawow, HOME, chwytak, `motions/`, STOP); ta sama sekcja jest w panelu jazdy :8000. `--fake` bez sprzetu, `--no-home` bez HOME (kamera na ramieniu). |
| `tools/base_test.py`             | Reczny test podwozia: `forward` / `turn` / `square`, pomiar znaku skretu i mapowania PWM. |
| `motions/*.json`                 | Nagrane ruchy ramienia (`home`, `grasp_mid` - placeholder, `drop_box` - placeholder). |
| `tests/`                         | Testy jednostkowe `pinecone_bot/*` (bez sprzetu, bez kamery). |

## Na laptopie (bez sprzetu)

```
python -m venv .venv
.venv\Scripts\activate            # PowerShell: .venv\Scripts\Activate.ps1
python -m pip install -r requirements-pinecone.txt

python -m pinecone_bot.main --sim --show
python -m pytest tests -q
```

`--sim` uruchamia caly stos (kamera, detektor, regulator, maszyna stanow,
"ramie") na symulowanym swiecie z `pinecone_bot/sim.py` - dobre miejsce do
sprawdzenia zmian w `brain.py` bez czekania na sprzet. `--show` otwiera okno
podgladu (wymaga `opencv-python` z GUI, nie `-headless`).

## Na Raspberry Pi, w tej kolejnosci

1. **Zaleznosci.** `uv pip install --python .venv/bin/python -r requirements-pinecone.txt`
   w wenwie repo (lerobot i pyrealsense2 juz tam sa z `requirements-pi.txt`). Szczegoly
   pierwszej instalacji calego systemu: `deploy/setup_pi.sh`.
2. **Przemontowanie kamery.** Miejsce chwytu musi byc widoczne i dalej niz
   minimalna odleglosc pomiaru glebi D415: ok. 31 cm przy strumieniu koloru
   640x480, ok. 16 cm przy 424x240 (patrz `pinecone_bot/camera.py`, pomiar w
   `scan_cones.py`). Kamera ma stac wysoko, za ramieniem, patrzac w dol - nie
   nisko z przodu jak w starszych probach (tam martwa strefa glebi zjadala
   dystans chwytania).
3. **Zbierz klatki murawy i szyszek:**
   `python tools/snap_frames.py --out frames/ --every 0.5 --seconds 20`
4. **Strojenie progu HSV** i zapis do configu:
   `python tools/calibrate_hsv.py --source frames/`
   (klawisz `s` zapisuje `cfg.detector.hsv` i `min_area_px` do
   `pinecone_config.json`).
5. **Nagraj chwyty ramieniem:**
   ```
   python tools/record_waypoints.py --name grasp_near --port /dev/robot-arm
   python tools/record_waypoints.py --name grasp_mid --port /dev/robot-arm
   python tools/record_waypoints.py --name grasp_far --port /dev/robot-arm
   python tools/record_waypoints.py --name drop_box --port /dev/robot-arm
   ```
   `motions/grasp_mid.json` jest juz z aktualnej kalibracji (klatki z
   `legacy/arm_recordings/demo2_fixed.csv`, zacisk do 0), wiec na start
   wystarczy nagrac `grasp_near`, `grasp_far` i `drop_box`. Config na Pi
   (`pinecone_config.json`) ma na razie tylko `grasp_mid`; po nagraniu
   dopisz kolejne chwyty do listy `grasps` (target_row ustawi
   `calibrate_target.py`). Chwyt bez pliku ruchu jest pomijany przy starcie.
6. **Kalibracja celu** - poloz szyszke dokladnie tam, gdzie kazdy nagrany
   chwyt ja podnosi, i zmierz, gdzie ona wtedy lezy w obrazie:
   `python tools/calibrate_target.py`
   Dla kazdego chwytu (near/mid/far): wybierz go klawiszem `1`/`2`/`3`,
   poczekaj na ustabilizowana srednia, wcisnij `s`. Potem `c` zapisuje
   biezaca kolumne szyszki jako `cfg.cx` (srodek chwytaka w obrazie), `w`
   zapisuje wszystko do `pinecone_config.json`.
7. **Sprawdz podwozie** - znak skretu i mapowanie predkosci na PWM:
   ```
   python tools/base_test.py --driver xiao --port /dev/robot-drive forward --seconds 2 --v 0.15
   python tools/base_test.py --driver xiao --port /dev/robot-drive turn --seconds 2 --w 0.5
   python tools/base_test.py --driver xiao --port /dev/robot-drive square --side 1.0
   ```
   Wynik idzie recznie do `pinecone_config.json`: znak skretu ->
   `cfg.control.steer_sign`, zmierzone mapowanie PWM ->
   `cfg.base.xiao_pwm_min/xiao_pwm_max/xiao_steer_min/xiao_steer_max`,
   rozstaw kol z pomiaru fizycznego -> `cfg.base.wheel_base_m`.
   `base_test.py` niczego sam nie zapisuje - tylko pokazuje, co zmierzyc.
8. **Uruchomienie z prawdziwa kamera, bez ruchu** (drukuje komendy zamiast
   je wykonywac): `python -m pinecone_bot.main --dry-run --show`
   a po sprawdzeniu, ze regulator sie zachowuje sensownie:
   `python -m pinecone_bot.main --real`.
   Trzymaj wylacznik awaryjny w rece przy pierwszym `--real`.

Caly ten przebieg czyta i zapisuje jeden plik: `pinecone_config.json` w
katalogu repo (`pinecone_bot/config.py`, `Config.save`/`Config.load`; sciezke
mozna nadpisac zmienna `PINECONE_CONFIG` albo flaga `--config`). Kazde
narzedzie z `tools/` dotyka tylko tych pol configu, ktore mierzy - reszta
zostaje bez zmian.

## Hoverboard: xiao czy bipropellant

`cfg.base.driver` przelacza sterownik podwozia (`pinecone_bot/base.py`):

- **`xiao`** - dzisiejsza sciezka: Seeed Xiao RP2040 sterujacy H-bridge'em
  (`xiao_send_pwm.ino`), linie ASCII `a<speed> b<steer>\n` po USB-serial.
  Wymaga zmierzonego mapowania m/s -> PWM (`base_test.py`, patrz wyzej).

- **`bipropellant`** - alternatywa: bezposredni UART do plyty hoverboarda z
  firmwarem `bipropellant-hoverboard-firmware`, bez posredniczacego Xiao.
  Predkosci kol podaje sie wprost w mm/s, a odometria idzie z hallotronow
  na plycie (dokladniejsza niz nic, jak przy `xiao`, ktory odometrii nie ma).
  10-minutowy test, czy plyta w ogole mowi tym protokolem: podlacz
  przejsciowke USB-UART do zlacza na sideboard/mainboard hoverboarda,
  115200 bodow (jesli nie odpowiada, sprobuj 9600), wyslij `unlockASCII`,
  potem `?`. Jesli plyta odpowie, mozna przestawic `cfg.base.driver` na
  `bipropellant`.
  Zanim to pojedzie na sprzecie, zweryfikuj zalozenia opisane w komentarzach
  `pinecone_bot/base.py` (sekcja BIPROPELLANT):
  - znak skretu (`w` dodatnie ma skrecac w lewo, sprawdzic jak przy `xiao`);
  - kolejnosc indeksow kol w tablicach `[2]` (zakladane `[0]`=lewe,
    `[1]`=prawe - `BIP_LEFT`/`BIP_RIGHT`) i znak przyrostu odometrii halla
    dla kazdego kola (`BIP_HALL_SIGN`, zakladane, ze jazda do przodu
    zwieksza oba);
  - uklad ramki 0x09 (enable) / 0x0A (disable poweroff) - payload jako
    int32 LE, DO SPRAWDZENIA na konkretnym firmware zespolu;
  - predkosc portu (115200 vs 9600) i regula sumy kontrolnej ramki
    (`build_frame`: suma bajtow CI..CS musi wyjsc 0 mod 256);
  - `cfg.base.wheel_base_m` (rozstaw kol) - zmierzyc fizycznie, uzywane do
    przeliczenia (v, w) na predkosci kol w mm/s.

## Strojenie

Dwa gainy regulatora P w `cfg.control`:

- `kx` - rad/s obrotu na piksel bledu poziomego (`px - cfg.cx`). Za male:
  robot wolno centruje szyszke w kadrze. Za duze: oscyluje/przestrzela.
- `ky` - m/s do przodu na piksel bledu pionowego (`target_row - py`). Za
  male: wolny dojazd. Za duze: robot wjezdza za daleko, zanim regulator
  zdazy zahamowac.

Do tego `tol_x_px` / `tol_y_px` (tolerancja, w ktorej uznajemy "ustawiony")
i `settle_frames` (ile klatek z rzedu w tolerancji, zanim ALIGN potwierdzi
chwyt - za male drga na szumie detekcji, za duze wydluza kazde podejscie).

`pinecone_log.csv` (kolumny: `t, state, n_det, px, py, err_x, err_y, v, w,
collected, yaw_deg`; `yaw_deg` tylko z wlaczonym kursem, patrz nizej) pokazuje dokladnie, dlaczego chwyt sie nie udal: patrz na
`err_x`/`err_y` tuz przed przejsciem do stanu `GRASP` - jesli byly poza
tolerancja, ALIGN nie powinien byl puscic do GRASP (blad w kodzie), a jesli
byly w tolerancji, ale szyszka i tak nie trafila do chwytaka, to
`target_row` danego chwytu jest zle skalibrowany (wroc do
`tools/calibrate_target.py`).

## Szukanie szyszek (SEARCH): pasy jak kosiarka

Gdy nic nie widac, robot jedzie wzorcem z `cfg.control.search_pattern`:

- `lanes` (domyslne): najpierw pelny obrot w miejscu (kamera widzi ok. 1.5 m,
  wiec od razu lapie szyszki dookola), potem pasy jak kosiarka: prosto
  `lane_length_m`, obrot 90 st w lewo, prosto `lane_spacing_m`, znowu 90 st
  w lewo, prosto z powrotem, nastepny pas w prawo... Po `lane_count` pasach
  robot konczy (DONE). Robot MUSI startowac w rogu trawnika, przodem wzdluz
  dluzszego boku, tak by trawnik byl po jego lewej stronie.
- `spin_drive`: pelny obrot, kawalek prosto, od nowa. Bladzi losowo, konczy po
  `search_timeout_s` bez detekcji.

Czas wzorca biegnie tylko w SEARCH i nie zeruje sie po chwycie, wiec po
zebraniu szyszki robot wraca do pasow mniej wiecej tam, gdzie przerwal.
"Mniej wiecej", bo bez odometrii nie wie, gdzie stanal po podjezdzie.
Pasy sa liczone z czasu i zadanych predkosci (`search_drive_v`, `search_w`),
wiec na trawie trzeba je zmierzyc `tools/base_test.py` i wpisac. Z Xiao
(otwarte PWM) pasy beda krzywe; z bipropellantem (zamknieta petla predkosci,
odczyt halla) mozna je potem oprzec na odometrii. Na demo wystarczy
`lane_count` 2-3 i szyszki w zasiegu pierwszego obrotu.

### Pasy po kursie (zyroskop telefonu)

Bez dostepu do plyty hovera nie ma odometrii, a pasy z czasu psuje glownie
kat: obrot "przez X sekund" slizga sie o kilka stopni, a prosta jedzie lukiem.
Dlatego kurs bierzemy z zyroskopu telefonu (`cfg.heading`, `pinecone_bot/heading.py`):

- obrot konczy sie, gdy kurs dojdzie do celu (`tol_deg`), a nie po czasie;
- na prostej regulator P (`kp`) trzyma kurs pasa; po podjezdzie do szyszki
  robot najpierw obraca sie z powrotem na kurs pasa;
- dlugosc prostej nadal z czasu i `search_drive_v` (zyroskop nie mierzy drogi).
  Blad dlugosci skraca pasy, ale zostaja rownolegle.

W symulacji z poslizgiem obrotow 15% i znoszeniem 0.03 rad/s koniec wzorca
mija sie z idealem o 0.10 m z kursem i o 3.5 m bez (`tests/test_heading.py`).

Uruchomienie:
1. Telefon plasko na bazie, ekranem do gory. phyphox -> "Gyroscope (rotation
   rate)" -> menu -> "Allow remote access" -> start. Ekran nie moze zgasnac.
2. Na Pi: `python tools/phyphox_check.py`, obroc robota recznie o 90 st w lewo.
   Kurs ma urosnac o ~+90. Maleje -> `"heading": {"sign": -1.0}` w configu.
3. `python -m pinecone_bot.main --dry-run --heading phyphox`, potem `--real`.
   `--heading` nadpisuje `heading.source` z configu (`none` = pasy z czasu).

Bezpieczniki: brak kursu dluzej niz `lost_s` albo odcinek, ktory trwa ponad
3x dluzej niz powinien (zly znak kursu albo `steer_sign`, robot kreci sie
w kolko), przelaczaja na pasy z czasu od tego samego miejsca wzorca. Oba
widac w konsoli ("UWAGA: pasy po kursie wylaczone").

## Czego nie robic

- Nie dodawac IK do ramienia na tym etapie - `approach_and_grasp.py` (stary
  skrypt zespolu z IK) zostaje odlozony na bok; ten stos celowo uzywa tylko
  nagranych waypointow.
- Nie wprowadzac ROS ani SLAM - caly stan robota to pozycja w obrazie plus
  prosta maszyna stanow, nic wiecej nie jest potrzebne do zbierania szyszek.
- Nie dodawac uczonych polityk (ACT, imitation learning itp.), dopoki ta
  deterministyczna petla nie dziala niezawodnie na sprzecie - to jest linia
  bazowa, do ktorej kazde pozniejsze podejscie z ML powinno sie porownywac.
