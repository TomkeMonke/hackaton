# ACT krok po kroku: nagranie -> trening -> chwyt na robocie

Dwa etapy chwytu: ACT (siec) robi TYLKO chwyt szyszki (HOME -> szyszka -> zamkniecie -> uniesienie),
wrzut do sloika robi nagrany ruch `motions/drop_box.json` (na Pi). Laczy je `tools/act_pick.py`.

Adresy: Pi = `robot@172.20.10.4` (hotspot "iPhone pawel"; gdy nie odpowiada: `hostname -I` na Pi).
Kamera na ramieniu: D435, serial `030522070668`. Datasety na Pi: `~/datasets/`.

## 1. Pi: nagrywanie epizodow (leader)

Leader wpiety w Pi jako `/dev/robot-leader` (zamiast kabla hovera - Pi nie ma wolnego USB).
Sprawdz: `ls -l /dev/robot-*` -> `robot-arm` i `robot-leader`. Zasilanie Pi mocne (`vcgencmd get_throttled` = `0x0`),
inaczej kamera gubi klatki (`frame is too old`), a spadek napiecia potrafi uszkodzic pliki venv.

```
pkill -f lerobot; pkill -f arm_web.py; pkill -f web_control.py
cd ~/hackaton && SVT_LOG=1 .venv/bin/lerobot-record --robot.type=so101_follower --robot.port=/dev/robot-arm --robot.id=so101 --robot.cameras="{ wrist: {type: intelrealsense, serial_number_or_name: 030522070668, width: 640, height: 480, fps: 30}}" --teleop.type=so101_leader --teleop.port=/dev/robot-leader --teleop.id=so101_leader --dataset.repo_id=local/so101_grasp --dataset.root=/home/robot/datasets/so101_grasp2 --dataset.push_to_hub=false --dataset.num_episodes=50 --dataset.episode_time_s=15 --dataset.reset_time_s=5 --play_sounds=false --dataset.single_task="Grasp the pine cone and lift it"
```

- Dokladanie do istniejacego datasetu: ta sama komenda + `--resume=true`, `--dataset.num_episodes=` ile DOLOZYC.
- Ruchy tylko po linii `Recording episode N`. `Reset the environment` = przerwa: szyszka w nowe miejsce, leader do HOME.
- Klawisze: strzalka w prawo = koniec epizodu, w lewo = nagraj od nowa, Esc = koniec (zapisuje).
- 5 miejsc szyszki x 10 epizodow. Koniec epizodu = pozycja startowa `drop_box` (ramie z przodu, nisko, chwytak zamkniety).
- Warningi `libtorchcodec` i `Svt[...]` sa niegrozne (dekoder -> pyav, log kodera wideo).

Stan 2026-09-27 14:00: `so101_grasp2` = 50 ep., 22451 klatek, 30 fps - GOTOWY do pelnego treningu (2b).
`so101_grasp` 7 ep. (do probnego treningu 2a).

## 2. Laptop z RTX 3070 (Franek): trening

Raz: repo i venv wedlug `docs/SETUP.md` sekcja "lerobot do ACT" (torch cu128 + lerobot 0.6.1).

```
cd <repo>
git pull
.venv\Scripts\python -c "import torch; print(torch.cuda.is_available())"
```
Musi byc `True`.

### 2a. Probny trening (7 epizodow, sprawdza sam proces, ~15 min)

```
scp -r robot@172.20.10.4:/home/robot/datasets/so101_grasp C:/datasets/so101_grasp
.venv\Scripts\lerobot-train --dataset.repo_id=local/so101_grasp --dataset.root=C:/datasets/so101_grasp --policy.type=act --policy.device=cuda --policy.push_to_hub=false --steps=5000 --batch_size=8 --output_dir=outputs/act_test --job_name=act_test
```
`loss` w logu ma spadac. Wagi z 7 epizodow nie beda dobrze chwytac - to test procesu.

### 2b. Pelny trening (po 50 epizodach)

Najpierw na Pi nagrywanie zakonczone (Esc albo koniec 50), inaczej kopia bedzie niepelna.

```
scp -r robot@172.20.10.4:/home/robot/datasets/so101_grasp2 C:/datasets/so101_grasp2
.venv\Scripts\lerobot-train --dataset.repo_id=local/so101_grasp --dataset.root=C:/datasets/so101_grasp2 --policy.type=act --policy.device=cuda --policy.push_to_hub=false --steps=50000 --batch_size=8 --save_freq=10000 --output_dir=outputs/act_grasp --job_name=act_grasp
```
Na RTX 3070 kilka godzin; mozna testowac posrednie checkpointy (`outputs/act_grasp/checkpoints/<krok>/`).

### 2c. Wagi na Pi

```
ssh robot@172.20.10.4 "mkdir -p ~/models"
scp -r outputs/act_test/checkpoints/last/pretrained_model robot@172.20.10.4:/home/robot/models/act_grasp
```
(dla pelnego treningu `outputs/act_grasp/...` zamiast `outputs/act_test/...`)

## 3. Pi: chwyt na robocie (wylacznik w rece)

Nic innego nie moze trzymac portu ramienia: `pkill -f lerobot; pkill -f arm_web.py`.

```
cd ~/hackaton
.venv/bin/python tools/act_pick.py --policy ~/models/act_grasp --dry-run      # tylko komendy
.venv/bin/python tools/act_pick.py --policy ~/models/act_grasp --skip-drop    # sam chwyt ACT
.venv/bin/python tools/act_pick.py --policy ~/models/act_grasp                # chwyt + sloik + HOME
.venv/bin/python tools/act_pick.py --policy ~/models/act_grasp --repeat 3
```

Sam ruch do sloika bez sieci: `.venv/bin/python tools/arm_play.py --motion drop_box --port /dev/robot-arm --home-first`.

## Pulapki (2026-09-27)

- Leader ma kalibracje SKOPIOWANA z followera (decyzja), czesc osi odwrocona. Dane dla ACT i tak poprawne
  (akcja = cel followera). Naprawa: `drive_mode: 1` dla odwroconych stawow w
  `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/so101_leader.json`.
  NIGDY `lerobot-calibrate` z `--robot.*` (plik followera ma reczna poprawke barku).
- `lerobot[dataset]` na Pi instalowac z override torcha, inaczej uv sciagnie torch z CUDA:
  `printf 'torch==2.14.0+cpu\ntorchvision==0.29.0+cpu\n' > /tmp/ov.txt` i
  `~/.local/bin/uv pip install --python .venv/bin/python --override /tmp/ov.txt --index-strategy unsafe-best-match --extra-index-url https://download.pytorch.org/whl/cpu 'lerobot[dataset]==0.6.1'`.
- `Overload error` na `id_=6` przy koncu nagrania = gripper followera dociskal szyszke az zadzialalo zabezpieczenie.
  Dataset jest zapisany (finalize idzie przed disconnect). Kasowanie: otworzyc gripper, wylaczyc zasilanie serw na kilka s.
  Przy nagrywaniu zamykac leaderem tylko do oporu - ACT nauczy sie docisku z danych.
- Bus error przy imporcie = plik uszkodzony po spadku napiecia; `--reinstall --no-deps --no-cache` tego pakietu.
