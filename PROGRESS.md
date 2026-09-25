# Progress log

Wspólny log sesji na tym repo. Każda nowa sesja/agent dopisuje sekcję na
dole z datą, co zrobiła i w jakim stanie to zostawiła. Nie nadpisuj
cudzych wpisów.

## NASTĘPNY KROK (aktualne na 2026-09-25)

Doszło Raspberry Pi 5 — cel: połączyć ramię (SO-101), kamerę (RealSense
D415) i robota-auto (hoverboard) w jeden spójny system zamiast trzech
osobnych, niepowiązanych podsystemów na osobnym sprzęcie/sesjach.

Nierozstrzygnięte, do ustalenia z użytkownikiem na starcie nowej sesji
(nie zgaduj, zapytaj):
- Jaka rola Pi 5 — centralny kontroler dla wszystkich trzech
  podsystemów, czy tylko dla jednego (np. kamera+ramię), a auto zostaje
  na Windows PC?
- Czy Pi 5 ma już system/sieć/zdalny dostęp skonfigurowany?
- Czy repo jest już sklonowane na Pi 5?
- Jak podsystemy mają się komunikować — jeden proces, czy osobne
  serwisy gadające przez sieć (np. web_control.py analogicznie do
  hoverboarda, ale dla całego systemu)?
- Status faktycznej pracy `arm_control.py` i `detect_object.py` na
  prawdziwym sprzęcie nie był w tej sesji zweryfikowany fizycznie —
  sprawdź zanim założysz że działają.

## Sprzęt

- **Robot-hoverboard**: 2x silnik hoverboardu, sterownik H-bridge (PWM+DIR)
  na `LEFT_PWM_PIN/LEFT_DIR_PIN/RIGHT_PWM_PIN/RIGHT_DIR_PIN` (D0-D3),
  Seeed Xiao RP2040 jako kontroler, USB-serial do PC (Windows, COM9 —
  numer portu może się zmienić po replugu).
- **Kamera**: Intel RealSense D415, USB.
- **Ramię**: SO-101 (TheRobotStudio/Hugging Face lerobot), serwa Feetech
  STS3215, URDF w `so101_urdf/so101_new_calib.urdf`.
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
- `arm_control.py` — sterowanie ramieniem SO-101.
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
