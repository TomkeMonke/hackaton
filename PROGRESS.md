# Progress log

Wspólny log sesji na tym repo. Każda nowa sesja/agent dopisuje sekcję na
dole z datą, co zrobiła i w jakim stanie to zostawiła. Nie nadpisuj
cudzych wpisów.

## NASTĘPNY KROK (aktualne na 2026-09-25, koniec sesji "szyszki: detekcja + dystans")

Cel ogólny: pick-and-place dowolnych, nieoznaczonych obiektów z podłogi
ramieniem SO-101, z kamerą D415 na platformie robota, docelowo wszystko
na Raspberry Pi 5 (8 GB) zamiast Windows PC.

Stan:
- Ramię: skalibrowane (serwa), sterowanie działa fizycznie
  (`arm_control.py`: status/move/home/straight/open/close/gong/dance).
- **Wizja: DZIAŁA i jest zmierzona.** `detect_floor_objects.py` +
  `scan_cones.py` wykrywają szyszki po głębi i podają dystans w układzie
  robota (do przodu / w bok / kąt). Na torze: 5 szyszek, rozrzut
  0.0–1.4 mm przy 15–20 klatkach. Issue #4 zamknięte przez PR #5.
  `detect_object.py` (HSV) został, ale do szyszek nie nadaje się —
  brązu w obrazie nie ma, patrz pułapki.
- **Dojazd i chwyt: kod gotowy, ZERO testów na sprzęcie.**
  `drive_to_target.py` (PR #9) i `approach_and_grasp.py` (PR #10) czytają
  `cel.json` z wizji. Oba mają dry-run jako domyślny i odmawiają ruchu
  bez jawnego portu. Ani ramię, ani platforma nie były podłączone przy
  ich pisaniu.
- Kalibracja kamera→ramię: nadal tylko PLAN (Claude Doc
  https://claude.ai/code/artifact/44f74190-82ad-4315-a878-07329119a2a7).
  `approach_and_grasp.py` używa OSZACOWANIA z montażu i mówi o tym przy
  każdym uruchomieniu.

Najważniejsze dalej (w tej kolejności):
1. **Zmierzyć stałe jazdy**: `python drive_to_target.py --calibrate`,
   zmierzyć miarką przejazd i obrót, wpisać do `drive_calibration.json`
   i ustawić `"measured": true`. Dopóki tego nie ma, skrypt nie pojedzie
   do celu. Spodziewany błąd dojazdu po kalibracji: **5–8 cm**.
2. **Sprawdzić znaki i offsety przegubów lerobota** przed PIERWSZYM
   ruchem ramienia z planu (`arm_control.py status` w pozie z wypisu,
   poprawić `arm_camera_transform.json`). Przy złym znaku plan czyta się
   bez zarzutu i wysyła ramię w drugą stronę.
3. **Zmierzyć, ile chwytak sięga przed obiektyw kamery** i ustawić na tej
   podstawie `--standoff` — ta sama liczba w `drive_to_target.py` i
   `approach_and_grasp.py`, inaczej plan chwytu opisuje inne miejsce niż
   to, gdzie stanie platforma. Teraz w obu jest 0.12 m (placeholder).
4. Kalibracja kamera→baza (Kabsch) — po niej chwyt przestaje być
   przybliżeniem.
5. Pi 5: instalacja stosu, build `pyrealsense2` ze źródeł.

Rozważyć (propozycja z sesji dojazdu, nie decyzja): **drugi skan po
obrocie, przed jazdą**. Na 0.5 m kamera jeszcze widzi, więc korekta kąta
jest darmowa i zamyka pętlę tam, gdzie błąd open-loop jest największy.

Pytania do użytkownika na starcie następnej sesji (nie zgaduj):
- Czy stałe kalibracji jazdy zostały zmierzone? Jaki wyszedł realny błąd
  dojazdu na 0.5 m?
- Ile chwytak sięga przed obiektyw kamery (do `--standoff`)?
- Czy zespół zatwierdził plan kalibracji kamera→ramię? Komentarze?
- Czy ramię SO-101 stoi na tej samej platformie co kamera, i w jakiej
  odległości/orientacji od niej (do transformaty)?
- Czy wchodzimy w odłożone przypadki: inne obiekty na torze (liście,
  patyki), detekcja w trakcie jazdy, zasięg powyżej 1 m?
- Czy testy będą za dnia? O zmierzchu pokrycie głębią na sztucznej
  trawie spada z 87% do 31% (patrz pułapki).
- Czy jest ramię leader SO-101 (teleop / nagrywanie demonstracji)?
- Czy robimy plan B z uczeniem (ACT) — jest GPU do treningu?
- Pi 5: jaka rola (centralny kontroler wszystkiego czy tylko
  kamera+ramię)? Czy ma już system/sieć/zdalny dostęp? Czy repo jest na
  nim sklonowane? Jak podsystemy mają się komunikować (jeden proces vs
  serwisy po sieci)?

## Sprzęt

- **Robot-hoverboard**: 2x silnik hoverboardu, sterownik H-bridge (PWM+DIR)
  na `LEFT_PWM_PIN/LEFT_DIR_PIN/RIGHT_PWM_PIN/RIGHT_DIR_PIN` (D0-D3),
  Seeed Xiao RP2040 jako kontroler, USB-serial do PC (Windows, COM9 —
  numer portu może się zmienić po replugu).
- **Kamera**: Intel RealSense D415, USB.
- **Ramię**: SO-101 (TheRobotStudio/Hugging Face lerobot), serwa Feetech
  STS3215, URDF w `so101_urdf/so101_new_calib.urdf`. Kontroler przez
  adapter USB CH343 (VID:PID 1A86:55D3) — na tym PC **COM10**. Kalibracja
  serw zapisana poza repo:
  `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json`
  (id ramienia `so101`).
- Kamera D415: serial 105422060821, firmware 5.17.0.10. Montaż docelowy:
  platforma robota, 12 cm nad ziemią.
- Platforma obliczeniowa: Raspberry Pi 5 8 GB. Jetson Nano P3450 odrzucony
  (patrz pułapki).
- **Nowość (2026-09-25)**: doszło Raspberry Pi 5 — plan połączenia
  ramienia, kamery i robota-auta w jeden spójny system (prawdopodobnie
  Pi 5 jako centralny kontroler zamiast/obok Windows PC).

## Struktura repo

- `web_control.py` + `frontend.html` — panel webowy do sterowania
  robotem-hoverboardem (WASD, tryb "osemka", tryb "pokrycie"/lawnmower,
  nagrywanie i odtwarzanie sekwencji ruchów). Serwer: HTTP :8000,
  WebSocket :8765. Odpalenie: `python web_control.py`.
- `xiao_send_pwm.ino` — firmware na Xiao RP2040, protokół
  `a<speed> b<steer>\n` po serialu, watchdog 500ms, flagi `SWAP_LR` /
  `INVERT_DIR` do korekty montażu (patrz sekcja niżej).
- `keyboard_control.py`, `makarena.py`, `figure_eight.py` — starsze,
  samodzielne skrypty terminalowe (przed powstaniem panelu webowego);
  panel webowy jest teraz głównym sposobem sterowania.
- `recordings/*.json` — zapisane sekwencje ruchów z panelu webowego.
- `arm_control.py` — sterowanie ramieniem SO-101 przez lerobot.
  CLI: `python arm_control.py calibrate|status|home|straight|move
  joint=val ...|open|close|gong|dance`. Moduł do importu: `make_arm()`,
  `read_joint_positions()`, `move_to()`, `go_home()`.
- `detect_object.py` — detekcja obiektu przez RealSense + deprojekcja
  do współrzędnych 3D. Szuka po KOLORZE (próg HSV) — do szyszek się nie
  nadaje, patrz pułapki. `--white-balance` / `--exposure` zamrażają
  obraz, żeby progi dobrane raz trafiały następnego dnia.
- `detect_floor_objects.py` — detekcja DOWOLNYCH obiektów po GŁĘBI:
  płaszczyzna ziemi (RANSAC), co nad nią wystaje, bramka wymiarowa.
  Zwraca pozycję w układzie robota: do przodu / w bok / kąt. Preset
  `--target szyszka` nastrojony na zmierzonych szyszkach.
  `--mode depth|color|both`, `--laser-power`, `--open-kernel`.
  Do importu: `FloorObjectDetector`, `detect_objects(frames)`.
- `scan_cones.py` — skan na POSTOJU: mediana z N klatek + rozrzut w mm,
  zapis celu do `cel.json` (ignorowany przez gita, to artefakt).
- `drive_to_target.py` + `drive_calibration.json` — dojazd platformy do
  celu z `cel.json`. Obrót w miejscu, potem jazda prosto. Odmawia jazdy
  na niezmierzonej kalibracji; `--calibrate` robi przejazd wzorcowy.
- `approach_and_grasp.py` + `arm_camera_transform.json` — cel z
  `cel.json` na kąty przegubów (ikpy + URDF), sześć kroków chwytu.
  `--dry-run` domyślny.
- `sample_colors.py` — mierzy HSV celu i tła, do dobrania progów koloru
  pod aktualne światło zamiast zgadywania.
- `test_detect_gates.py`, `test_drive_plan.py`, `test_grasp_plan.py` —
  testy bez sprzętu, zwykły python (bez pytest): `python <plik>`.
- `so101_urdf/` — model URDF ramienia (do IK, np. przez `ikpy`).
- `constraints.txt` — pin `numpy==2.5.3` dla pip; `lerobot` na Python
  3.14 próbuje przebudować numpy ze źródła i pada (brak wheela + za
  stary GCC w systemie) — instalować z `--no-deps` i doinstalowywać
  brakujące moduły ręcznie, albo `-c constraints.txt --only-binary=:all:`.

## Ważne pułapki / lessons learned

- **Dwa mnożące się suwaki prędkości** (LIMIT PRĘDKOŚCI × prędkość
  trybu auto) potrafią zejść do wartości za niskiej żeby fizycznie
  ruszyć silniki (martwa strefa PWM). Przy "robot nic nie robi" zawsze
  sprawdź realny `pwm_speed`/`pwm_steer`, nie tylko czy komenda leci.
- **Sterowanie różnicowe**: `steer` musi być wyraźnie mniejsze niż
  `speed`, inaczej jedno koło idzie na minus i robot wiruje w miejscu
  zamiast jechać łukiem.
- **Robot montowany "do góry nogami"** względem oryginalnego założenia
  wymaga `SWAP_LR=true` i `INVERT_DIR=true` w firmware (zamiana L/R +
  odwrócenie kierunku obu silników).
- **COM port bywa niestabilny** po zawieszeniu USB CDC (Windows error
  31, "urządzenie nie działa") — zwykle pomaga fizyczny replug kabla,
  programowy disable/enable urządzenia wymaga uprawnień admina.
- **Nie rób ciężkiego re-renderu całego DOM co każdy tick WebSocketa**
  (było 30ms) — realne kliknięcia myszką/palcem w taki element się
  gubią, mimo że automatyzacja (precyzyjny klik) działa bez zarzutu.
  Rób diff/porównanie i przebudowuj tylko gdy dane faktycznie się
  zmieniły.
- Trzymaj `state.playback_name`/podobne pola trybu w spójności przy
  KAŻDEJ zmianie trybu (`set_mode`), nie tylko przy naturalnym
  zakończeniu — inaczej UI pokazuje duchy poprzedniego stanu.
- **lerobot 0.6.1: nie ma modułu `so101_follower`.** SO-100/101 scalone
  w `SOFollower`; import:
  `from lerobot.robots.so_follower.so_follower import SO101Follower` i
  `from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig`.
- **Brakujące zależności lerobot (instalacja `--no-deps`)**, w tej
  kolejności wychodziły: `huggingface_hub`, `feetech-servo-sdk` (tylko
  sdist — BEZ `--only-binary=:all:`, inaczej "No matching distribution"),
  `deepdiff` → `cachebox` (jest wheel cp314). Zawsze z `-c constraints.txt`.
- **Ramię niewidoczne jako COM** = kabel USB kontrolera niepodłączony albo
  brak zasilania serw. Porty Bluetooth COM3/5/7/8 to nie ramię.
- **"There is no status packet!" na magistrali Feetech przy szybkich
  pętlach ruchu.** Przyczyna: `max_relative_target` w configu sprawia, że
  KAŻDY `send_action` robi dodatkowy `sync_read` Present_Position — przy
  komendach co 0.25 s zapycha to bus. `sync_write` nie czeka na
  odpowiedź, więc błąd wychodzi dopiero później (np. w `disconnect`),
  co myli przy diagnozie. Rozwiązanie w pętlach: własne ograniczenie
  kroku w Pythonie + `arm.config.max_relative_target = None` na czas
  pętli (tak robi `dance()`/`gong()`).
- **Gwałtowne losowe ruchy = ochrona przeciążeniowa STS3215**, serwa
  przestają odpowiadać. Pomaga tylko odłączenie i ponowne podłączenie
  USB/zasilania kontrolera. Duże zakresy — tylko z małym krokiem na tick.
- **Zero stopni w lerobot ≠ zero w URDF (prawdopodobnie).** lerobot
  liczy `deg = (raw - (range_min+range_max)/2) * 360/4095`, czyli 0° to
  środek zakresu nagranego przy kalibracji serw, a nie poza zerowa URDF.
  Przed FK/IK trzeba zmierzyć offset i znak każdego przegubu (plan
  kalibracji, krok 1). Po `arm_control.py calibrate` kąty się zmieniają.
- **Jetson Nano P3450 nie nadaje się**: max JetPack 4.6 = Ubuntu 18.04 +
  Python 3.6, a lerobot wymaga Pythona 3.10+. Wybrane Raspberry Pi 5.
- **Kamera 12 cm nad ziemią**: D415 ma martwą strefę głębi — wg pamięci
  ok. 45 cm przy 1280×720, ok. 30 cm przy 640×480 (NIE zmierzone). Obiekty
  bliżej kamery nie dostaną XYZ. Sprawdzić przed ustaleniem zasięgu
  chwytania.
- **Bibliotek pod Python 3.14 Windows** (sprawdzone `pip download
  --only-binary`): są `mujoco`, `roboticstoolbox-python`, `open3d 0.20`,
  `ultralytics`, `torch`; NIE MA `pin` (pinocchio, więc wbudowane w lerobot
  `RobotKinematics`/placo nie działa), `pybullet`, `pyroki`.
- **`opencv-python` + `opencv-python-headless` zainstalowane razem** =
  `cv2.imshow` pada (`The function is not implemented. Rebuild the
  library with Windows, GTK+ 2.x or Cocoa support`), bo headless
  nadpisuje binaria GUI w tym samym namespace `cv2`. Fix: odinstalować
  oba i postawić od nowa tylko `opencv-python` (lerobot ciągnie
  `opencv-python-headless` jako zależność — konflikt nawracający przy
  `pip install lerobot`, sprawdzać po każdym takim instalu).
- **D415 znika z `rs.context().query_devices()` po nieczystym zamknięciu
  procesu** (crash przed `pipeline.stop()`, albo proces zabity przez
  `taskkill`) — Windows/librealsense zostawia uchwyt USB w złym stanie.
  Objaw: `RuntimeError: No device connected` albo `HResult 0x800703e3`
  przy `pipeline.start()`, mimo że kamera fizycznie podłączona. Fix:
  sprawdzić `tasklist | grep python` i dobić wiszące `python.exe`
  (zombie trzymają handle nawet gdy skrypt "już wyszedł" wg statusu
  procesu-wrappera); jeśli to nie pomoże — fizyczny replug USB. Zawsze
  zamykać stream przez `finally: pipeline.stop()`, nigdy Ctrl+C na
  goło/kill -9.
- **Martwa strefa D415 — ZMIERZONA, 640×480: 0.27 m.** Wcześniejszy wpis
  mówił "wg pamięci ok. 30 cm, NIE zmierzone" — teraz jest pomiar: ziemia
  jest mierzona od 0.27 m (dolne wiersze kadru) do ~1.1 m przy kamerze
  poziomo 10 cm nad podłożem. To jest powód, dla którego pozycję celu
  trzeba **zapamiętać z dystansu i dojechać z pamięci** — na dystansie
  chwytania szyszki już nie widać.
- **Lewa piąta kadru nie ma głębi.** Pokrycie wg kolumn: 0–64 px → 9.8%,
  64–128 px → 3.8%, środek → 25–34%. Prawy obiektyw nie widzi tego, co
  lewy łapie przy swojej lewej krawędzi, więc stereo nie ma czego
  dopasować. Podbicie lasera tego NIE naprawia (9.8% → 4.4%) — brakuje
  drugiego punktu widzenia, nie światła. Nie celować w obiekt lewą
  krawędzią kadru.
- **Sztuczna trawa o zmierzchu gubi głębię.** To samo miejsce: 87%
  pokrycia po południu, 31% o zmierzchu. Murawa nie ma własnej tekstury.
  Projektor IR stoi domyślnie na 150 z 360 — podbicie do 360 dało
  31.0% → 35.8%. Warto, ale nie ratuje sytuacji; testujcie za dnia.
- **Otwarcie morfologiczne jądrem 5×5 zjada małe obiekty.** Ta sama
  szyszka mierzy 2.2 cm wysokości przy progu 0.6 cm i 1.0 cm przy progu
  0.8 cm — nie dlatego, że zmierzono ją inaczej, tylko dlatego, że z
  cieńszego paska erozja zostawia sam dół. Przy celach 20–50 px jądro 3×3
  jest bezpieczniejsze. Objaw skrajny: klaster w ogóle nie powstaje, więc
  liczniki odrzuceń pokazują zera i "nie ma czego tłumaczyć".
- **Piksele bez głębi deprojektują się do (0,0,0)**, czyli do pozornej
  odległości równej offsetowi płaszczyzny. Domknięcie morfologiczne
  zalepia nimi dziury w obiekcie. Geometrię (centroid, wysokość,
  szerokość) liczyć WYŁĄCZNIE z pikseli faktycznie nad płaszczyzną,
  inaczej wysokości wychodzą absurdalne (61 cm przy limicie 40 cm), a
  szerokość chwytu skacze między 14 a 44 cm na nieruchomej scenie.
- **Offset dopasowanej płaszczyzny to zmierzona wysokość kamery** —
  porównanie go z wysokością montażu jest darmowym testem, czy RANSAC
  złapał ZIEMIĘ, a nie ścianę albo blat. Wyłapało przestawioną kamerę
  dwa razy w jednej sesji. Bez tego wszystkie dystanse liczą się od złej
  płaszczyzny i wyglądają wiarygodnie.
- **Kolor: w kadrze nie ma żadnego brązu.** Zmierzone HSV (OpenCV, 0–179)
  na szyszkach i murawie o zmierzchu: murawa H≈90 (cyjan), szyszki H≈148
  (fiolet), ogony rozkładów zachodzą na siebie. Rozdziela je JASNOŚĆ:
  szyszki V≈94, murawa V≈152. Próg na jasność działa, ale jest kruchy —
  słońce albo cień go przesuwa. Przed strojeniem progów HSV zamrozić
  balans bieli i ekspozycję (`detect_object.py --white-balance
  --exposure`), inaczej progi dobrane dziś nie trafią jutro.
- **Laptop z portami USB 2.0 Type-A cofa D415 do USB 2.1** i obcina tryby
  (848×480 tylko 10/6 Hz, koloru w tej rozdzielczości nie ma wcale).
  To nie jest wina kabla — na MSI Modern 14 B10MW oba porty Type-A są
  fizycznie 2.0, USB 3 jest tylko na Type-C. 640×480@30 działa i daje
  realnie 18–22 fps, co do skanu na postoju w zupełności wystarcza.

## Log sesji

### 2026-09-25 — sesja Claude (hoverboard control panel)

- Setup od zera: `makarena.py` (gamepad) → `keyboard_control.py`
  (WASD, bo brak pada) → `web_control.py` + `frontend.html` (pełny
  panel webowy, bo terminal był niewygodny).
- Napisany `xiao_send_pwm.ino` od zera (generyczny H-bridge PWM+DIR,
  różnicowy mix speed+steer, watchdog).
- Dodane tryby auto: "osemka" (figure-eight) i "pokrycie"
  (boustrophedon/lawnmower — jedzie rząd, obraca się ~180° w miejscu w
  dwóch krokach, jedzie rząd wstecz, powtarza).
- Dodany moduł nagrywania/odtwarzania sekwencji ruchów (zapis do
  `recordings/*.json`, przetrwa restart serwera).
- Wszystkie parametry (prędkości, czasy, siła skrętu) dostrajalne na
  żywo suwakami w przeglądarce, bez edycji kodu.
- Naprawione po drodze: skalowanie limitu prędkości nie obejmowało
  steer w manualu; race condition przy broadcastcie do wielu klientów;
  DOM re-render 30x/s gubił kliknięcia na liście nagrań;
  `playback_name` nie czyścił się przy zmianie trybu.
- Repo GitHub utworzone (`pawel120/hackaton`, prywatne), inne sesje
  dorzuciły równolegle `detect_object.py` i rozbudowany
  `arm_control.py` — scalone w jednym branchu `master`.
- Stan na koniec sesji: panel webowy działa end-to-end (potwierdzone
  fizycznym ruchem robota), COM9 bywa niestabilny po odłączeniu USB.
- **Nie zrobione / do zrobienia**: integracja ramię+kamera+auto w
  jeden system (właśnie doszło Raspberry Pi 5 do tego celu), IK dla
  SO-101 (ikpy + URDF, niesprawdzone końca), `lerobot` install na
  Python 3.14 wymaga obejścia (patrz `constraints.txt`).

### 2026-09-25 — sesja Claude (ramię SO-101 + kamera, planowanie pick-and-place)

- Dokończona instalacja lerobot na Python 3.14 (doinstalowane
  `huggingface_hub`, `feetech-servo-sdk`, `deepdiff`, `cachebox`); import
  `SO101Follower` działa (nowa ścieżka, patrz pułapki).
- Wykryte: D415 przez `pyrealsense2` OK; ramię na COM10 (CH343).
- Pobrany URDF SO-101 z TheRobotStudio/SO-ARM100 do
  `so101_urdf/so101_new_calib.urdf`; `ikpy` go ładuje (5 przegubów +
  `gripper_frame_joint` jako końcówka, chwytak poza łańcuchem IK).
- Napisany `arm_control.py` (CLI + funkcje do importu). Użytkownik zrobił
  kalibrację serw, ruch potwierdzony fizycznie (`move shoulder_pan=20`
  → odczyt 19.38°). `HOME_POSE` = poza zaraz po kalibracji.
- Dodane komendy demo: `straight` (wszystko 0°, chwytak otwarty),
  `gong` (powolny zamach + szybkie uderzenie jednym przegubem),
  `dance` (losowe ruchy ~90% zakresu, mały krok). Po drodze naprawione
  zapychanie magistrali (patrz pułapki); retry odczytu pozycji,
  `disconnect` nie wywala programu.
- Decyzje: kamera stała względem ramienia (eye-to-hand), najpierw na
  maszcie, potem ustalone: platforma robota 12 cm nad ziemią. Obiekty do
  podnoszenia nieoznaczone → detekcja po głębi zamiast po kolorze.
  Platforma: Raspberry Pi 5 8 GB zamiast Jetson Nano P3450.
- Research bibliotek (lerobot + ACT/SmolVLA, ikpy vs roboticstoolbox,
  mujoco, open3d, ultralytics; co ma wheele pod py3.14 — patrz pułapki).
  Rekomendacja: klasyczny pipeline (głębia → kalibracja → IK) jako
  główny, ACT (imitation learning) jako plan B.
- GitHub: issue #1 (detekcja + deprojekcja 3D, zrobione przez Tomka w
  PR #2 jako `detect_object.py`), issue #4 (detekcja dowolnych obiektów
  po głębi, open3d, przypisane TomkeMonke).
- Plan kalibracji kamera→ramię (marker ArUco na chwytaku, 15–20 póz,
  Kabsch, cel RMS < 10 mm) jako Claude Doc na 1 stronę A4 do
  zatwierdzenia przez zespół (link w "NASTĘPNY KROK").
- **Nie zrobione**: weryfikacja zer/kierunków przegubów lerobot vs URDF,
  FK/IK na prawdziwym ramieniu, marker i skrypt kalibracji kamera→ramię,
  pętla pick-and-place, cokolwiek na Pi 5. `dance` na pełnym zakresie
  nie było sprawdzone do końca po ostatniej poprawce (ostatnie
  uruchomienie padło w `go_home` na zgubionym pakiecie — od tego czasu
  jest retry, nieprzetestowany). `gong` i `straight` nieprzetestowane
  fizycznie (brak potwierdzenia od użytkownika).

### 2026-09-25 — sesja Claude (podgląd na żywo RealSense D415)

- Cel: prosty live preview (`rs_preview.py`) color+depth side-by-side
  z `pyrealsense2` + `cv2.imshow`, do wizualnej kontroli co widzi kamera.
  RealSense Viewer (osobna appka Intela) NIE jest zainstalowany i nie
  jest potrzebny — `pyrealsense2` ma librealsense wbudowane.
- Napisany `rs_preview.py`: color+depth align, colormapa JET, FPS co 30
  klatek w konsoli, wyjście `q`/ESC, `pipeline.stop()` w `finally`.
- Napotkane i naprawione po drodze (patrz pułapki): konflikt
  `opencv-python` / `opencv-python-headless` (reinstall samego
  `opencv-python`); D415 znikająca z enumeracji USB po zombie procesach
  pythona (`taskkill` wiszących `python.exe`).
- Potwierdzone: kamera wykrywana (`rs.context().query_devices()` → 1,
  D415, serial 105422060821), skrypt odpala się bez wyjątku po
  posprzątaniu procesów.
- **Nie zrobione / niepotwierdzone**: użytkownik nie potwierdził jeszcze
  wizualnie że okno faktycznie pokazuje obraz (proces działał bez
  crasha, ale brak feedbacku "widzę obraz" na koniec sesji) — na
  starcie następnej sesji zapytać czy `rs_preview.py` faktycznie
  pokazuje live podgląd, czy nadal łapie `No device connected` po
  replugu (jeśli tak, sprawdzić Menedżer Urządzeń / port USB, może hub
  zamiast bezpośredniego portu USB3).

### 2026-09-25 — sesja Claude (Raspberry Pi 5: architektura + setup zdalnego sterowania)

- Decyzje: Pi 5 jedzie na robocie, urządzenia po USB (D415 USB3, ramię
  CH343, Xiao), operator przez WiFi (panel webowy). Bluetooth odrzucony
  jako szkielet systemu (za mała przepustowość dla kamery, za duże
  opóźnienia dla magistrali Feetech). System: Raspberry Pi OS Lite 64-bit
  na pendrivie USB (brak karty SD), venv z `uv` + Python 3.12, bez Dockera
  na start, autostart przez systemd, hotspot WiFi z Pi na demo.
- Poradnik setupu Pi (Claude Doc):
  https://claude.ai/code/artifact/247e71b9-5b39-4ffb-8170-e355db9fd56b
  (instalacja, lerobot, RealSense ze źródeł jeśli brak wheela, udev
  `/dev/robot-arm` i `/dev/robot-drive`, IMU BNO085, systemd, hotspot,
  checklista, diagnostyka).
- Kod: porty ze zmiennych `ROBOT_DRIVE_PORT` / `ROBOT_ARM_PORT` (domyślnie
  `COM9` / `COM10`, Windows bez zmian); `web_control.py` nasłuchuje na
  `0.0.0.0` (`ROBOT_HOST`); **failsafe operatora**: brak wiadomości z
  przeglądarki przez 0,5 s (albo zero klientów) = natychmiastowy stop,
  tryb manual, koniec nagrywania/odtwarzania. Frontend wysyła ping co
  200 ms i puszcza klawisze przy `visibilitychange`. Nowy
  `requirements-pi.txt` (NIE używać `constraints.txt` na Pi).
- Failsafe sprawdzony lokalnie klientem websockets bez Xiao (ping → jedzie,
  brak pingu → `failsafe=True`, speed 0, manual). **Nie sprawdzony na
  fizycznym robocie.**
- GitHub: issue #7 (mapowanie ogrodu RealSense/RTAB-Map, odłożone, najpierw
  test na laptopie), issue #8 (zasilanie z akumulatora 12 V, osoba od
  elektroniki; baterie AA 1,5 V nie nadają się do zasilania Pi).
- Stan na koniec: pendrive Kingston DataTraveler 3.0 64 GB gotowy do
  wgrania systemu Imagerem. Na Pi nic jeszcze nie jest zainstalowane.
- Pułapka: przed failsafe'em zerwanie WiFi zostawiało robota jadącego z
  ostatnią komendą (watchdog Xiao chroni tylko łącze Pi–Xiao, nie
  operator–Pi). Tryby auto też jechały bez operatora.

### 2026-09-25 — sesja Claude (Tomek: wykrywanie szyszek po głębi + dystans, dojazd, plan chwytu)

Zamknięcie issue #4 i zbudowanie całego łańcucha od kamery do katów
przegubów. Cztery PR-y z forka `TomkeMonke/hackaton` (brak uprawnień do
pushu na `pawel120/hackaton`): **#5** wizja, **#9** platforma, **#10**
ramię, **#11** ten wpis.

**Zrobione:**
- `detect_floor_objects.py` — detekcja DOWOLNYCH obiektów po głębi:
  RANSAC płaszczyzny ziemi (+ douczenie SVD na inlierach), maska tego co
  nad nią wystaje, `cv2.connectedComponentsWithStats` zamiast DBSCAN.
  **Świadomie bez `open3d`**: głębia z D415 jest ZORGANIZOWANA (to obraz,
  sąsiedztwo pikseli już jest sąsiedztwem w przestrzeni), więc
  etykietowanie spójnych obszarów kosztuje ułamek ms, a DBSCAN po ~300
  tys. luźnych punktów setki ms na klatkę. Zmierzone **22.2 fps** end to
  end. Zero nowych zależności.
- **Preset `szyszka`** — bramka wymiarowa nastrojona na ZMIERZONYCH
  szyszkach na waszej murawie (nie na wymiarach z książki): 0.5–6 cm
  szerokości, 2–10 cm długości, 0.8–9 cm nad ziemią, wypełnienie ≥ 0.4.
  Kolorem się ich nie wyłowi — patrz pułapki.
- **Wyniki w układzie ROBOTA, nie kamery**: `forward_m`, `lateral_m`,
  `ground_distance_m`, `bearing_deg`, wyprowadzone z dopasowanej
  płaszczyzny, więc krzywo przykręcony uchwyt kompensuje się sam.
- `scan_cones.py` — skan na POSTOJU: mediana z N klatek + rozrzut w mm.
  Zapisuje `cel.json`. Na torze: 5 szyszek, każda widziana w 15/15
  klatkach, **rozrzut 0.0–1.4 mm**.
- `drive_to_target.py` + `approach_and_grasp.py` — dojazd i plan chwytu,
  oba czytają `cel.json`. Szczegóły w #9 i #10.
- Testy bez sprzętu: `test_detect_gates.py`, `test_drive_plan.py`,
  `test_grasp_plan.py` — razem 28, zwykły python, bez pytest.

**Decyzje:**
- **Postój-skan-zapamiętaj-jedź**, nie ciągłe śledzenie. Wymusza to
  martwa strefa 0.27 m (patrz pułapki) — przy chwytaniu szyszki nie widać.
- Zasięg skanu **0.3–1.0 m** (ustalony z zespołem). Zmierzone: na 1.06 m
  rozrzut to nadal ~1 mm, więc podniesienie do 1.2–1.5 m jest bezpieczne.
- Detekcja po GŁĘBI jako podstawowa, kolor jako opcja (`--mode
  depth|color|both`). Uzasadnienie w pułapkach.
- Wykrycia migoczące są odrzucane: skan zgłasza tylko cele widziane w
  ≥60% klatek. Dlatego podgląd na żywo pokazuje więcej niż skan.

**NIE dokończone / świadomie odłożone:**
- **Nic nie sprawdzone na sprzęcie poza kamerą.** Ani ramię, ani
  platforma nie były podłączone (zero portów COM na tej maszynie).
  Dojazd i chwyt są przetestowane wyłącznie w dry-run.
- **Kalibracja kamera→baza ramienia** — `approach_and_grasp.py` używa
  OSZACOWANIA z montażu. To wasz osobny task (plan w Claude Doc).
- **Stałe kalibracji jazdy niezmierzone** — `drive_to_target.py`
  ODMAWIA jazdy, dopóki ktoś ich nie zmierzy przez `--calibrate`.
- **Znaki/offsety przegubów lerobota niesprawdzone** — przy złym znaku
  plan czyta się bez zarzutu i wysyła ramię w drugą stronę.
- Odłożone na życzenie zespołu: inne obiekty na torze (liście, patyki),
  detekcja w trakcie jazdy, zasięg powyżej 1 m.
- Szyszka przy lewej krawędzi kadru nie jest wykrywana — patrz pułapki,
  to ograniczenie stereo, nie progów.
