# Setup

Dwie osobne sciezki: laptop (rozwoj, symulator, testy) i Raspberry Pi 5
(prawdziwy sprzet: kamera + ramie + hoverboard). Traps i wytlumaczenia
"dlaczego tak" - patrz docs/HARDWARE.md.

## Laptop (Windows/Linux/Mac)

Wymagany Python 3.12. Na tym laptopie z Windowsem Python zostal
zainstalowany przez `winget install Python.Python.3.12`, a git przez
Git for Windows (dlatego np. mDNS/`*.local` dziala z git-bash, a nie z
PowerShell - patrz nizej).

```
python -m venv .venv
```

Aktywacja venv:
- PowerShell: `.\.venv\Scripts\Activate.ps1`
- git-bash: `source .venv/Scripts/activate`
- Linux/Mac: `source .venv/bin/activate`

Instalacja zaleznosci (stos `pinecone_bot` - kamera + symulator +
testy, bez sprzetu ramienia/kol):

```
pip install -r requirements-pinecone.txt
```

Testy (60+ testow, bez sprzetu):

```
python -m pytest tests -q
```

Symulator (kamera pinhole + naped roznicowy, podglad w oknie):

```
python -m pinecone_bot.main --sim --show
```

Jesli zamiast `pinecone_bot` pracujesz nad starszym stosem
(`arm_control.py`, IK, detekcja po glebi) - dodatkowo potrzebny
`requirements-arm.txt` (ciagnie `lerobot` + `torch`, ciezkie i wolne):

```
pip install -r requirements-arm.txt -c constraints.txt
```

`constraints.txt` pinuje `numpy==2.5.3` - na Python 3.14 `lerobot` bez
tego probuje przebudowac numpy ze zrodla i pada (brak gotowego kola +
za stary GCC w systemie). Jesli `pip install lerobot` mimo to pada,
instaluj `--no-deps` i doinstaluj brakujace zaleznosci recznie (w
kolejnosci, w jakiej sie ujawniaja): `huggingface_hub`,
`feetech-servo-sdk` (tylko sdist - BEZ `--only-binary=:all:`, inaczej
"No matching distribution"), `deepdiff`, `cachebox`. Zawsze z
`-c constraints.txt`. Po kazdym `pip install lerobot` sprawdz, czy nie
wrocil konflikt `opencv-python` / `opencv-python-headless` (patrz
docs/HARDWARE.md, pulapka 25) - jesli tak, odinstaluj oba i zainstaluj
od nowa tylko `opencv-python`.

## lerobot do ACT: laptop = serwer, Pi = klient robota

Stan na 2026-09-27: laptop i Pi ZROBIONE, sprawdzone importami (na Pi tez IK placo
na naszym URDF, klient async, pyrealsense2, pinecone_bot). Teleop telefonem i
nagranie jeszcze nie odpalone. Tlo i decyzja: `docs/POLICIES_LEROBOT.md`.

### Wariant z leaderem SO-101 (MAMY leader, lezy w sali 435 D) - to jest sciezka glowna

Z leaderem telefon i placo sa zbedne: `lerobot-record --teleop.type=so101_leader`. Leader NIE
ma jeszcze pliku kalibracji (ani na Pi, ani na laptopie; `teleop_mirror.py` zakladal id
`so101_leader`, ale nigdy nie byl odpalony). Kroki, wszystkie interaktywnie w terminalu
operatora na Pi (`ssh -t robot@<ip>`), wylacznik w rece:

1. Leader ma TEN SAM kontroler CH343 (1a86:55d3) co follower, wiec stara regula udev dalaby
   obu nazwe `/dev/robot-arm`. Nowa `deploy/99-robot.rules` rozroznia po numerze seryjnym
   (follower = `5B41532803`, kazdy inny CH343 = `/dev/robot-leader`). Po wpieciu leadera:
   `ls -l /dev/robot-*` musi pokazac `robot-arm`, `robot-leader`, `robot-drive`.
2. Kalibracja TYLKO leadera (tworzy
   `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/so101_leader.json`;
   plik followera `robots/so_follower/so101.json` zostaje nietkniety). NIGDY nie podawac tu
   `--robot.*` (HARDWARE.md, pulapka 1):

   ```
   cd ~/hackaton && .venv/bin/lerobot-calibrate --teleop.type=so101_leader --teleop.port=/dev/robot-leader --teleop.id=so101_leader
   ```

3. Test teleopu bez kamer, ramie w wolnej przestrzeni, kamera na ramieniu zabezpieczona:

   ```
   .venv/bin/lerobot-teleoperate --robot.type=so101_follower --robot.port=/dev/robot-arm --robot.id=so101 \n     --teleop.type=so101_leader --teleop.port=/dev/robot-leader --teleop.id=so101_leader
   ```

   `lerobot-teleoperate` NIE kalibruje followera, jesli jego plik istnieje (sprawdza `is_calibrated`).
   Gdyby mimo to zapytal o kalibracje followera - przerwac Ctrl+C.
4. Nagranie datasetu lokalnie (bez wysylania na Hub; potem katalog `--dataset.root` kopiujemy
   `scp -r` na laptop do treningu). Kamera na ramieniu jako `wrist` (RealSense, serial
   105422060821), druga, statyczna kamera USB jako `top` (index z `lerobot-find-cameras`):

   ```
   .venv/bin/lerobot-record --robot.type=so101_follower --robot.port=/dev/robot-arm --robot.id=so101 \n     --robot.cameras="{ top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: intelrealsense, serial_number_or_name: 105422060821, width: 640, height: 480, fps: 30}}" \n     --teleop.type=so101_leader --teleop.port=/dev/robot-leader --teleop.id=so101_leader \n     --dataset.repo_id=local/so101_szyszki --dataset.root=/home/robot/datasets/so101_szyszki \n     --dataset.push_to_hub=false --dataset.num_episodes=50 --dataset.episode_time_s=30 --dataset.reset_time_s=10 \n     --dataset.single_task="Pick up the pine cone and put it in the box"
   ```

   Klawisze w trakcie: `n` nastepny epizod, `r` powtorz, `q` koniec. 5 pozycji szyszki x 10
   epizodow, kamery nieruchome, ten sam chwyt. Dodatkowe epizody: ta sama komenda z
   `--resume=true` i `num_episodes` = ile DOLOZYC.
5. Trening na laptopie: `lerobot-train --dataset.repo_id=local/so101_szyszki --dataset.root=<skopiowany katalog> --policy.type=act --policy.device=cuda --policy.push_to_hub=false`.

Ponizsze (telefon + placo) zostaje jako wariant awaryjny, gdyby leader byl niedostepny.

Podzial rol jak w async inference lerobota: laptop trenuje ACT na GPU i w czasie
jazdy jest `policy_server`; Pi obsluguje ramie, kamere i telefon (nagranie
datasetu, `robot_client`). Teleop telefonem liczy IK przez `placo`, ktore NIE ma
kola na Windows (tylko Linux/macOS) - dlatego nagrywanie idzie z Pi, nie z laptopa.

### Laptop (Windows, venv `.venv`, Python 3.12)

Torch z PyPI na Windows jest bez CUDA - najpierw torch z indeksu cu128, potem
lerobot; `lerobot` sciaga `opencv-python-headless`, ktory psuje GUI (HARDWARE.md,
pulapka 25), wiec na koncu opencv stawiamy od nowa:

```
python -m pip install "torch==2.11.0" "torchvision==0.26.0" --index-url https://download.pytorch.org/whl/cu128
python -m pip install "lerobot[phone,feetech,async]==0.6.1"
python -m pip uninstall -y opencv-python opencv-python-headless
python -m pip install "opencv-python>=4.9,<4.14"
python -c "import torch; print(torch.cuda.is_available())"     # True na RTX 3070
```

Uwagi: `lerobot==0.6.1` pinuje `numpy<2.3`, wiec numpy schodzi z 2.5.3 do 2.2.6
(na Python 3.12 sa kola, `constraints.txt` NIE jest tu potrzebny; testy 128/128
zielone po zmianie). Torch cu128 to ~2.6 GB - po hotspocie z telefonu ok. 1 h.
`from lerobot.model.kinematics import RobotKinematics` importuje sie, ale
utworzenie obiektu pada bez placo - to oczekiwane na Windows.

Plik kalibracji ramienia jest poza repo: skopiowany z Pi do
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json`
(ten sam `id=so101`; NIE uruchamiac `lerobot calibrate`, HARDWARE.md pulapka 1).

### Trening ACT na laptopie (Windows) - pulapki z 2026-09-27

- `lerobot-train` wymaga extras `dataset` i `training`: `pip install "lerobot[dataset,training]==0.6.1"`
  (bez tego `ImportError: 'datasets' is required`). Torch cu128 zostaje (pin torch<2.12 jest spelniony).
- Wideo datasetu (AV1) dekoduje `pyav` (`--dataset.video_backend=pyav`); `torchcodec` bez ffmpeg w PATH
  sypie traceback przy starcie, ale lerobot sam przechodzi na pyav - ignorowac.
- **Laptop na baterii = GPU 210 MHz / 20 W** (flagi power cap + thermal slowdown przy 51 st C), krok 1.6 s
  zamiast 0.14 s. Zasilacz MUSI byc podpiety; po podpieciu zegary wracaja same, bez restartu.
- Po zapisie checkpointu lerobot tworzy symlink `checkpoints/last`; Windows bez trybu deweloperskiego
  odmawia (`WinError 1314`) i trening PADA po pierwszym checkpoincie. Obejscie: `tools/train_win.py`
  (te same flagi co `lerobot-train`, pomija symlink). `--resume` wtedy nie dziala (wymaga `last`).
- `num_workers=4` na Windows bylo WOLNIEJSZE (2.8 s/krok) niz `num_workers=0` (0.33 s/krok, GPU 0.14 s,
  dane 0.19 s). Batch 8 + `--policy.use_amp=true`, ~3 kroki/s na RTX 3070; 4000 krokow = ~22 min.
- Dataset kopiowany z Pi w trakcie nagrywania ma uciety parquet ("Parquet magic bytes not found") -
  kopiowac dopiero, gdy `lerobot-record` na Pi sie skonczyl; `tar --exclude="tmp*"` przez ssh.
- Zabijanie procesow po tresci linii polecen (`wmic`/`Get-CimInstance ... CommandLine -like`) trafia tez
  wlasne skrypty bash, ktore te slowa zawieraja w heredocu - filtrowac po `Name -eq 'python.exe'`.

Komenda, ktora zadzialala (dataset w repo: `datasets/so101_grasp2`, po `git lfs pull`):

```
python tools/train_win.py --dataset.repo_id=local/so101_grasp2 --dataset.root=datasets/so101_grasp2 \n  --dataset.video_backend=pyav --policy.type=act --policy.device=cuda --policy.use_amp=true \n  --policy.push_to_hub=false --output_dir=outputs/train/act_so101_grasp2 --job_name=act_so101_grasp2 \n  --steps=7000 --batch_size=8 --num_workers=0 --log_freq=100 --save_freq=1000 --wandb.enable=false
```

Wagi na Pi: `tar -cf - -C outputs/train/act_so101_grasp2/checkpoints/<krok> pretrained_model | ssh robot@<ip>
'mkdir -p ~/models/act_so101_grasp2/<krok> && tar -xf - -C ~/models/act_so101_grasp2/<krok>'` (198 MB).
Na Pi `ACTPolicy.from_pretrained(...)` laduje sie 30 s (pierwszy raz sciaga resnet18 z torch hub - Pi
potrzebuje internetu), jedna paczka 100 akcji liczy sie ~0.65 s na CPU -> rollout lokalnie na Pi jest OK.
Kamera na Pi to D435, serial `030522070668` (NIE 105422060821 z HARDWARE.md), klucz w datasecie
`observation.images.wrist`, 640x480@30.

### Rollout polityki ACT na Pi (lokalnie, CPU)

Wagi w `~/models/act_so101_grasp2/<krok>/pretrained_model` (patrz wyzej). Ramie w wolnej przestrzeni,
szyszka jak przy nagraniach, wylacznik w rece, leader odpiety. W terminalu operatora:

```
ssh -t robot@172.20.10.4 "cd ~/hackaton && .venv/bin/lerobot-rollout --strategy.type=base \n  --policy.path=/home/robot/models/act_so101_grasp2/001000/pretrained_model --policy.device=cpu \n  --robot.type=so101_follower --robot.port=/dev/robot-arm --robot.id=so101 --robot.max_relative_target=20 \n  --robot.cameras=\"{ wrist: {type: intelrealsense, serial_number_or_name: 030522070668, width: 640, height: 480, fps: 30}}\" \n  --task='Pick up the pine cone' --duration=30"
```

Start ~30 s (ladowanie modelu), potem co ~3 s paczka 100 akcji (0.65 s liczenia) odtwarzana z 30 Hz.
`--robot.max_relative_target=20` ogranicza skok stawu na tick (bezpiecznik; za "gumowy" ruch -> 30).
Ruch od razu w zla strone = wylacznik i `docs/ARM_FRAMES.md` krok 1 (zera stawow). ACT ignoruje tekst
`--task`. Klucz kamery MUSI byc `wrist` (tak w datasecie). Alternatywa przy slabym CPU: policy server na
laptopie + `robot_client` na Pi (sekcja "Async" w docs/POLICIES_LEROBOT.md), ale hotspot ma 100-240 ms.

### Pi (venv `~/hackaton/.venv`, Python 3.12, `uv`)

Pi ma internet przez hotspot (PyPI odpowiada, ~0.3 MB/s), `uv` jest w
`~/.local/bin`. Dwie pulapki instalacji:

- Pi ma `torch 2.14.0+cpu` z indeksu CPU, a lerobot pinuje `torch<2.12`; bez
  override uv sciaga generyczny torch z PyPI, ktory na aarch64 ciagnie ~2 GB
  paczek CUDA + triton (LOG 2026-09-26). Override trzyma zainstalowany torch.
- `hebi-py` (aplikacja HEBI Mobile I/O na iPhone) istnieje tylko jako sdist ~92 MB
  na wersje; przy pelnym `lerobot[phone]` uv cofal sie po wersjach i sciagal
  kazda (2.11 -> 2.10.1 -> ... po 5-20 min sztuka). Rozwiazanie: extras bez
  `phone`, a jego skladniki (`teleop`, `fastapi`, `scipy`, `hebi-py`) podane wprost.

```
printf "torch==2.14.0+cpu\ntorchvision>=0.22\n" > ~/uv_overrides.txt
cd ~/hackaton && uv pip install --python .venv/bin/python \
  --index https://download.pytorch.org/whl/cpu --index-strategy unsafe-best-match \
  --override ~/uv_overrides.txt \
  "lerobot[feetech,async,kinematics]==0.6.1" "teleop>=0.1.0,<0.2.0" "fastapi<1.0" scipy "hebi-py>=2.8.0,<2.12"
```

Po instalacji na Pi: torch 2.14.0+cpu (bez zmian), torchvision 0.29.0+cpu, numpy 2.2.6,
placo 0.9.15, hebi-py 2.11.0, tylko `opencv-python-headless`. Placo przy ladowaniu URDF
ostrzega o samokolizjach w pozie neutralnej (siatki kolizyjne SO-101 nachodza na
siebie) - dla IK nieszkodliwe. Pierwsza proba padla na "network unreachable": hotspot
sie zrestartowal i laptop sam przeskoczyl na "hacker-bloc" - gdy Pi "znika", najpierw
`netsh wlan show interfaces` na laptopie.

`pip install lerobot` NIE zawiera katalogu `examples/`, a skrypty teleopu
telefonem zyja tylko tam. Na Pi odtworzony recznie jako
`~/hackaton/examples/phone_to_so100/` (poza gitem): `teleoperate.py`, `record.py`,
`replay.py`, `rollout.py`, `evaluate.py` z tagu lerobot v0.6.1 oraz katalog
`SO101/` z TheRobotStudio/SO-ARM100 (`Simulation/SO101`: URDF identyczny z naszym
`so101_urdf/so101_new_calib.urdf` plus 31 plikow STL, ktorych URDF wymaga, a
ktorych w repo nie ma). Skrypty maja w srodku port `/dev/tty.usbmodem...`,
`id` i kamery z docs - przed uruchomieniem podmienic na `/dev/robot-arm`,
`id="so101"` i nasza kamere (`use_degrees=True` zostaje).

## Raspberry Pi 5

### Jak sie polaczyc z Pi

- Kabel ethernet laptop<->Pi (adapter USB-Ethernet w laptopie). Pi ma
  **statyczne IP `192.168.137.5`** (ustawione przez `nmcli` na "Wired
  connection 1"), laptop `192.168.137.1` (Windows ICS - Internet
  Connection Sharing). ICS jest przypisane do KONKRETNEGO adaptera -
  jesli kabel wpiniesz w inny port (np. wbudowany Realtek zamiast
  adaptera USB-Ethernet), `192.168.137.5` nie odpowie. Przy zmianie
  portu przelacz ICS na ten adapter albo nadaj laptopowi recznie
  `192.168.137.1` na nowym porcie.
- WiFi na Pi DZIALA (wczesniej ten plik mowil, ze nie): Pi laczy sie z
  hotspotem "iPhone pawel" jako `wlan0`, adres `172.20.10.4` (DHCP -
  adres moze sie zmienic przy kolejnym polaczeniu). `ssh robot@172.20.10.4`,
  ping po WiFi 11-109 ms. `robot.local` (mDNS) dziala w git-bash tak samo
  jak dla adresu kablowego. Laptop moze SAM przelaczyc WiFi na inna znana
  siec (zdarzylo sie na "hacker-bloc", tylko IPv6) i zgubic polaczenie z
  Pi - sprawdz, do jakiej sieci laptop jest podlaczony, zanim szukasz
  problemu gdzie indziej (ten sam objaw daje odpiety kabel ethernet).
- `ssh robot@192.168.137.5` - user `robot`, haslo znasz (nie w tym
  pliku). `robot.local` (mDNS) dziala z git-bash, ale NIE z PowerShell -
  w PowerShell uzywaj IP wprost.
- Interaktywne narzedzia ramienia (np. `tools/record_motion.py`,
  `tools/arm_play.py`) odpalaj we WLASNYM terminalu operatora,
  interaktywnie: `ssh -t robot@<ip> "cd ~/hackaton && .venv/bin/python
  tools/..."`. Sesja tmux/nohup odpalona z nieinteraktywnego ssh (np. z
  sesji Claude) ginie po rozlaczeniu - patrz docs/HARDWARE.md.
- Sesje Claude nie moga kopiowac plikow kodu na Pi (blokada trybu auto
  "Remote Shell Writes") - config JSON idzie przez `python` heredoc po
  ssh, ale pliki `.py` musi skopiowac operator sam, np.
  `scp tools\record_motion.py robot@172.20.10.4:~/hackaton/tools/` (z
  cmd na laptopie).
- Pliki z laptopa na Pi: `scp plik.py robot@192.168.137.5:~/hackaton/`
  - **w PowerShell na laptopie, NIE z wnetrza sesji SSH na Pi** (Pi nie
  ma internetu, wiec `scp`/`git pull` z Pi w strone swiata nie zadziala
  - patrz nizej). Dla stosu `pinecone_bot` jest gotowy skrypt
  `deploy/push_to_pi.sh` (kopiuje `pinecone_bot/`, `tools/`, `motions/`,
  `tests/`, `requirements-pinecone.txt` i lokalny `pinecone_config.json`
  jesli istnieje; rsync gdy dostepny, inaczej `scp -r`):

  ```
  PI_HOST=robot@192.168.137.5 bash deploy/push_to_pi.sh
  ```

  **Uwaga:** `pinecone_config.json` jest sledzony w gicie i ten skrypt go
  NADPISUJE na Pi przy kazdym pushu - wartosci zmierzone na sprzecie
  musza wejsc do configu W REPO (commit/PR), inaczej gina przy nastepnym
  pushu (patrz docs/HARDWARE.md, ostatnia pulapka). Internet na Pi (i
  `git pull`) nie jest potwierdzony - wszystkie pliki nadal ida przez
  `scp`/`push_to_pi.sh` z laptopa.

### Instalacja od zera: `deploy/setup_pi.sh`

Uruchom NA Pi, jako zwykly user (NIE root/sudo), z katalogu glownego
repo:

```
bash deploy/setup_pi.sh
```

Bezpieczny do wielokrotnego uruchomienia - kazdy krok pomija to, co
juz jest zrobione. Robi po kolei:

1. Pakiety systemowe (`apt-get`: git, curl, build-essential, cmake,
   pkg-config, libusb, libssl, libudev, python3-dev, htop, tmux).
2. Dodaje uzytkownika do grupy `dialout` (dostep do portow
   szeregowych) i usuwa `modemmanager`, jesli jest zainstalowany (probuje
   odpytywac komendami AT nowe urzadzenia `ttyACM` i potrafi porwac
   Xiao/ramie).
3. Instaluje `deploy/99-robot.rules` do `/etc/udev/rules.d/` (stabilne
   nazwy `/dev/robot-arm` i `/dev/robot-drive`) plus regulki RealSense z
   repo IntelRealSense/librealsense, `udevadm control --reload-rules` +
   `trigger`.
4. Instaluje `uv` (menedzer Pythona), jesli go jeszcze nie ma.
5. Tworzy venv Python 3.12 (`uv venv --python 3.12 .venv`) i instaluje
   `requirements-pi.txt`. **To najdluzszy krok - `lerobot` ciagnie
   PyTorch, moze potrwac dlugo.** Na koniec odinstalowuje
   `opencv-python` (na Pi, ktore jest headless/bez ekranu, zostaje tylko
   `opencv-python-headless` z `requirements-pi.txt` - inaczej konflikt,
   patrz docs/HARDWARE.md pulapka 25).
6. `pyrealsense2` (bindingi Pythona do RealSense): najpierw probuje
   gotowego kola (`uv pip install pyrealsense2`). Jesli nie ma kola pod
   ta architekture/Pythona, wypisuje instrukcje budowy librealsense ze
   zrodel (patrz sekcja "Kamera RealSense D415" w starszym poradniku
   setupu) i konczy krok bez bledu - reszta stosu dziala bez kamery.
7. Autostart panelu webowego: wypelnia `deploy/robot-web.service`
   (User, WorkingDirectory) i instaluje jako `systemd` unit
   `robot-web.service`, `enable` + `restart`. Uruchamia
   `web_control.py` z portami `ROBOT_DRIVE_PORT=/dev/robot-drive`,
   `ROBOT_ARM_PORT=/dev/robot-arm`. `KillSignal=SIGINT`, zeby przy
   stopie zadzialaly bloki `finally` (stop silnikow, `pipeline.stop()`
   kamery).
8. Na koniec wypisuje adres panelu (`http://<hostname>.local:8000` albo
   po IP), jak podejrzec logi (`journalctl -u robot-web -f`), status
   (`systemctl status robot-web`), zawartosc `/dev/robot-*` i
   ostrzezenie, jesli brakuje pliku kalibracji ramienia (trzeba
   skopiowac z laptopa, patrz docs/HARDWARE.md - kalibracja jest poza
   repo).

**NIE uruchamiac `lerobot calibrate` na Pi** - nadpisze recznie
poprawiony offset barku, patrz docs/HARDWARE.md (pulapka 1).

### Ktory plik requirements czego dotyczy

- `requirements-pi.txt` - GLOWNY plik dla Pi. `lerobot==0.6.1`,
  `pyserial`, `websockets`, `ikpy`, `opencv-python-headless`. Uzywany
  przez `deploy/setup_pi.sh`. NIE uzywac tu `constraints.txt` - jest
  tylko na Windows/Python 3.14 (patrz nizej).
- `requirements-pinecone.txt` - zaleznosci nowego stosu `pinecone_bot/`
  (numpy, opencv-python, pyserial, pytest). Wystarcza do pracy na
  laptopie (symulator + testy, bez sprzetu). Czesc dla Pi (kamera,
  ramie) jest opisana w komentarzu w tym pliku jako "juz w
  `requirements-pi.txt`, nie duplikowac tutaj".
- `requirements-arm.txt` - starszy stos sterowania ramieniem
  (`arm_control.py` + IK na URDF z `so101_urdf/`): `lerobot>=0.6.1`,
  `ikpy>=4.1.0`, plus bazowy `requirements.txt`. Instalowac z
  `-c constraints.txt` - ciagnie `torch`, ciezka i dluga instalacja.
  Potrzebny tylko na maszynie, ktora faktycznie steruje ramieniem.
- `constraints.txt` - pin `numpy==2.5.3`, tylko dla Windows/Python 3.14
  (patrz wyzej, sekcja Laptop).

### `arm.sh` - skrot do sterowania ramieniem na Pi

Na Pi jest skrypt `arm.sh` opakowujacy `arm_control.py` (patrz jego
CLI): `./arm.sh status | home | straight | open | close | move
joint=wartosc ...`.

### Jak odpalac (w sesji SSH na Pi)

```bash
cd ~/hackaton && source .venv/bin/activate
./arm.sh status | home | straight | open | close | move shoulder_pan=10
python rs_mjpeg_server.py          # podglad na zywo: http://192.168.137.5:8080/ (Ctrl+C = stop)
python scan_cones.py --json cel.json
python approach_and_grasp.py --no-drive                       # dry-run planu
python approach_and_grasp.py --no-drive --port /dev/robot-arm # NA ZYWO
python drive_step.py --port /dev/robot-drive --speed 150 --duration 0.05   # jeden krok kolami
python auto_collect.py --drive-port /dev/robot-drive --arm-port /dev/robot-arm --dry-run
python record_demo.py --seconds 25 --out demo.csv   # nagranie recznego ruchu (torque off)
python replay_demo.py --port /dev/robot-arm [--dry-run]
```

Kamera na wylacznosc: zatrzymaj `rs_mjpeg_server.py` przed skanem
(jeden proces na raz trzyma urzadzenie D415).

Poradnik krok-po-kroku dla stosu `pinecone_bot` (kolejnosc narzedzi
`tools/snap_frames.py` -> `calibrate_hsv.py` -> `record_waypoints.py`
-> `calibrate_target.py` -> `base_test.py`, potem `--dry-run` / `--real`)
jest w `PINECONE_README.md` w korzeniu repo.

## Sprawdzenie, ze wszystko dziala

Na Pi, po `deploy/setup_pi.sh` i podlaczeniu sprzetu:

```
./arm.sh status
python rs_preview.py            # albo: python rs_mjpeg_server.py
python drive_step.py --port /dev/robot-drive --speed 120 --duration 0.05
python -m pytest tests -q
```

`./arm.sh status` powinno pokazac aktualne katy przegubow bez bledu
magistrali. `rs_preview.py`/`rs_mjpeg_server.py` powinno pokazac zywy
obraz koloru i glebi. `drive_step.py` z podanymi parametrami powinno
dac wyczuwalny, pojedynczy krok kolami do przodu (patrz
docs/HARDWARE.md pulapka 8 - krok jest nieliniowy, nie oczekuj
konkretnej odleglosci bez wlasnej kalibracji). `pytest` powinno przejsc
bez sprzetu (testy sa czysto programowe).
