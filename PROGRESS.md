# Progress log

Wspólny log sesji na tym repo. Każda nowa sesja/agent dopisuje sekcję na
dole z datą, co zrobiła i w jakim stanie to zostawiła. Nie nadpisuj
cudzych wpisów.

## NASTĘPNY KROK (aktualne na 2026-09-25, koniec sesji "ramię SO-101")

Cel ogólny: pick-and-place dowolnych, nieoznaczonych obiektów z podłogi
ramieniem SO-101, z kamerą D415 na platformie robota (12 cm nad ziemią),
docelowo wszystko na Raspberry Pi 5 (8 GB) zamiast Windows PC.

Stan:
- Ramię: skalibrowane (serwa), sterowanie działa fizycznie
  (`arm_control.py`: status/move/home/straight/open/close/gong/dance).
- Kamera: D415 działa; `detect_object.py` (Tomek) wykrywa po kolorze HSV.
- Detekcja dowolnych obiektów po głębi (płaszczyzna podłogi, open3d):
  issue #4, przypisane do TomkeMonke — w toku u niego.
- Kalibracja kamera→ramię: jest tylko PLAN (1 strona A4, Claude Doc
  "Plan kalibracji kamera–ramię SO-101",
  https://claude.ai/code/artifact/44f74190-82ad-4315-a878-07329119a2a7),
  czeka na zatwierdzenie przez zespół. Kodu jeszcze brak.

Najważniejsze dalej (w tej kolejności):
1. Po zatwierdzeniu planu: "Krok 1 / model przegubów" — `arm_model.py`
   z FK (ikpy + URDF) i tabelą offset/znak dla 5 przegubów, sprawdzenie
   czy zera lerobot == zera URDF (patrz pułapki). Bez tego FK/IK kłamią.
2. Marker ArUco na chwytaku + `calibrate_camera_arm.py` (Kabsch) +
   `camera_to_base.json`.
3. IK (ikpy albo roboticstoolbox-python) + pętla pick-and-place.
4. Pi 5: instalacja stosu, build `pyrealsense2` ze źródeł.

Pytania do użytkownika na starcie następnej sesji (nie zgaduj):
- Czy zespół zatwierdził plan kalibracji? Są komentarze w dokumencie?
- Czy ramię SO-101 też stoi na platformie robota, obok kamery?
- Jaki obszar podłogi (cm) ma być zasięgiem chwytania? Czy leży poza
  martwą strefą D415 (~30–45 cm od obiektywu, do zmierzenia)?
- Czy jest ramię leader SO-101 (teleop / nagrywanie demonstracji)?
- Czy robimy plan B z uczeniem (ACT) — jest GPU do treningu?
- Status issue #4 u Tomka.
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
  do współrzędnych 3D.
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
