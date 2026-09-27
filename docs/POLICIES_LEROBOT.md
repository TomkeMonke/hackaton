# Gotowe polityki lerobot do chwytania - co da sie wziac z polki

Stan na 2026-09-27 (research w sieci, nic nie uruchamiane). Cel: nie pisac polityki
od zera - sprawdzic, co z lerobot, DOT (IliaLarchenko) i MolmoAct (Allen AI) da sie
uzyc do podnoszenia szyszek ramieniem SO-101.

To jest sciezka ROWNOLEGLA do deterministycznej petli z `docs/RUNBOOK.md`
("Czego nie robic": ML dopiero, gdy petla bazowa dziala). Nie zastepuje jej.

## Nasze warunki brzegowe

| Co | U nas | Znaczenie dla polityk |
|---|---|---|
| Komputer na robocie | Raspberry Pi 5, bez GPU, `lerobot==0.6.1` | Zadna siec poza ACT nie policzy sie na Pi; potrzebny policy server na laptopie (async inference w lerobot) |
| Laptop (frane) | RTX 3070 Laptop 8 GB VRAM, 32 GB RAM | ACT: trening i inference OK. SmolVLA: inference OK, fine-tune ciasno. MolmoAct2 / Flux3: nie zmiesci sie |
| Kamery | jedna D415 NA RAMIENIU (wrist) | Gotowe checkpointy SO-101 chca 2 kamer (top + side lub top + wrist). Trzeba dolozyc statyczna kamere (zwykly USB webcam) |
| Teleoperacja | brak ramienia leader | Kazda polityka wymaga ~50 epizodow nagranych teleoperacja. Zamiennik: teleop telefonem (`lerobot[phone]`) albo klawiatura EE |
| Kalibracja | nowa konwencja lerobot (>= 0.5, zero w srodku zakresu), recznie poprawiony homing offset barku | Zgodna z aktualnymi checkpointami; NIE uruchamiac `lerobot calibrate` (HARDWARE.md) |

## Kandydaci

### 1. lerobot ACT (`--policy.type=act`) - REKOMENDACJA

- Jest w `lerobot==0.6.1` (to, co mamy na Pi). Najmniejsza siec (ResNet18 + maly
  transformer), lerobot poleca go jako pierwsza polityke.
- Trening na naszych ~50 epizodach: kilka godzin na RTX 3070 (docs mowia
  "several hours" na GPU), da sie tez w Colab / HF Jobs (`--job.target=a10g-small`).
- Inference: laptop jako policy server, Pi jako robot client (sekcja "Async" nizej).
- Gotowych wag do szyszek nie ma. Cudze checkpointy ACT dla SO-101 (np.
  `Histochemichael/act-so101-rack-centered-approach-2cam-25`) sa pod cudza scene,
  kamery i przedmiot - nie przeniosa sie.

### 2. SmolVLA (`lerobot/smolvla_base`, 450M) - plan B, jesli ACT nie generalizuje

- Model bazowy, docs wprost: "fine-tuning on your own data is required".
  Zalecane ~50 epizodow, 5 pozycji x 10 powtorzen; 25 epizodow bylo za malo.
- Fine-tune 20k krokow ~4 h na A100 (batch 64). Na 8 GB VRAM tylko maly batch;
  realnie Colab (notebook `training-smolvla.ipynb`) albo HF Jobs.
- Inference ~2 GB VRAM -> RTX 3070 wystarczy, `--inference.type=rtc` na wolnym sprzecie.
- Referencyjny dataset i scena: `lerobot/svla_so101_pickplace` (50 epizodow, klocek
  do przezroczystego pudelka, kamery `up` + `side`). Gotowe fine-tuny na HF
  (`jammyyang/smolvla_so101_pick_place`, `semi01/smolvla_official_so101_pickplace`,
  `nota-gmbh/so101_pick_place_pen_smolvla`) - jak wyzej, cudze sceny.

### 3. MolmoAct2 (Allen AI; w lerobot `--policy.type=molmoact2`) - "moloko sa gotowe", ale nie u nas

- Jedyny kandydat z deklarowanym zero-shot na SO-100/101: checkpoint
  `lerobot/MolmoAct2-SO100_101-LeRobot` (zadanie typu "pick up the red cube",
  kamery `cam0` + `cam1` = glowna + druga / wrist, `--rename_map`).
  Docs lerobot podaja gotowa komende `lerobot-rollout` (nizej).
- Koszt: 5B parametrow, plik `model.safetensors` 21.8 GB (fp32), w bf16 ~10 GB
  samych wag -> NIE zmiesci sie na RTX 3070 8 GB. Allen AI testowal na H100
  (~180 ms na akcje). Realnie: GPU >= 24 GB w chmurze + async inference po sieci
  (latencja hotspotu 100-240 ms, patrz STATUS) - watpliwe na hackatonie.
- Wymaga lerobot z `main` (w 0.6.1 nie ma `molmoact2`) i `uv sync --extra molmoact2`.
- Base `allenai/MolmoAct2` sam pisze: "foundation checkpoint for further fine-tuning
  rather than a ready-to-run policy". Fine-tune: 8x H100 w przykladach.
- Wniosek: sprawdzic tylko, jesli ktos da dostep do GPU 24 GB+; wtedy to jest
  najszybsza droga do "podnies szyszke" bez nagrywania danych.

### 4. FLUX 3 Action SO-101 (`black-forest-labs/flux-3-action-so101`) - odpada

7B "world action model", 2 kamery (scene + wrist), tylko lerobot `main`. Jeszcze
wiecej VRAM niz MolmoAct2.

### 5. DOT policy (IliaLarchenko/dot_policy) - odpada

- PR #739 do lerobot ("Adding DOT policy") zamkniety jako stale 2025-11-05, NIE
  zmergowany. Kod dziala tylko na forku `IliaLarchenko/lerobot`, branch
  `dot_new_config`, ze stara konfiguracja lerobot (przed zmiana kalibracji 0.5) -
  niekompatybilny z naszym 0.6.1 i nasza kalibracja.
- Gotowe checkpointy tylko symulacyjne (`dot_pusht_*`, `dot_bimanual_insert`).
  Na SO-ARM100 autor pokazal tylko demo "pick and drop" bez wag.
- Sam autor w LeHome 2026 (1. miejsce online) uzyl Pi0.5 + RL (`lehome_solution`),
  nie DOT. Zaleta DOT (14M parametrow, 20-25 predykcji/s na CPU M1) bylaby ciekawa
  dla Pi, ale wymagalaby portu do lerobot 0.6 - to jest pisanie od zera.

## Wspolny problem: dane, nie model

Wszystkie polityki poza MolmoAct2 wymagaja wlasnego `LeRobotDataset`: ~50 epizodow,
obraz z kamer + stan + akcje, nagranych TELEOPERACJA (reka operatora nie moze byc
w kadrze). `tools/record_motion.py` (prowadzenie reka) tego nie daje: nie zapisuje
klatek, a reka jest w obrazie.

Opcje bez leader arm:

1. **Telefon** (`pip install "lerobot[phone]"`): iOS -> app HEBI Mobile I/O (ARKit),
   Android -> WebXR (`teleop` z PyPI). IK przez placo na URDF - mamy
   `so101_urdf/so101_new_calib.urdf`, docs kaza go skopiowac do
   `examples/phone_to_so100/SO101/`. Skrypty `teleoperate.py`, `record.py`,
   `replay.py`, `evaluate.py` w `examples/phone_to_so100/`.
2. **Klawiatura EE** (`--teleop.type=keyboard_ee`, ten sam IK) - wolniejsze.
3. Pozyczyc leader SO-101 - najlepsza jakosc danych.

Druga kamera: statyczna, patrzaca na pole chwytu z gory/boku (`top`), plus nasza
wrist D415 jako `wrist`. Bez kamery statycznej polityka nie widzi szyszki, gdy
ramie stoi w HOME.

## Rekomendowana sciezka (jesli zespol decyduje: robimy ML rownolegle)

1. Laptop + ramie (bez Pi): `pip install "lerobot[phone]"` w `.venv`, skopiowac
   `so101.json` z Pi do `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`
   (HARDWARE.md), sprawdzic teleop telefonem. Wylacznik w rece.
2. Dolozyc kamere statyczna, nagrac 50 epizodow "podnies szyszke i wloz do pudelka",
   5 pozycji szyszki x 10, kamery nieruchome, ten sam chwyt.
3. `lerobot-train --policy.type=act` na RTX 3070 (albo `--job.target=a10g-small`).
4. `lerobot-rollout` na laptopie do testu przy stole; docelowo policy server na
   laptopie, `robot_client` na Pi.
5. Dopiero jesli ACT nie generalizuje po pozycji szyszki: fine-tune SmolVLA w Colab
   na tym samym datasecie.

## Komendy (z docs lerobot, do podstawienia portow i kamer)

Nagranie (leader zastapiony telefonem - patrz `examples/phone_to_so100/record.py`;
ponizej wersja z leaderem z docs):

```
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101 \
  --robot.cameras="{ top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, wrist: {type: intelrealsense, serial_number_or_name: 105422060821, width: 640, height: 480, fps: 30}}" \
  --dataset.repo_id=${HF_USER}/so101_szyszki --dataset.num_episodes=50 \
  --dataset.single_task="Pick up the pine cone and put it in the box"
```

Trening ACT:

```
lerobot-train --dataset.repo_id=${HF_USER}/so101_szyszki --policy.type=act \
  --output_dir=outputs/train/act_szyszki --job_name=act_szyszki \
  --policy.device=cuda --policy.repo_id=${HF_USER}/act_szyszki
```

Rollout (test przy stole, laptop):

```
lerobot-rollout --strategy.type=base --policy.path=${HF_USER}/act_szyszki \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101 \
  --robot.cameras="{ ... jak przy nagraniu ... }" \
  --task="Pick up the pine cone and put it in the box" --duration=60
```

Async (laptop = server, Pi = client; `pip install "lerobot[async]"` po obu stronach):

```
python -m lerobot.async_inference.policy_server --host=0.0.0.0 --port=8080
python -m lerobot.async_inference.robot_client --server_address=<ip_laptopa>:8080 \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101 \
  --robot.cameras="{ ... }" --policy_type=act --pretrained_name_or_path=${HF_USER}/act_szyszki \
  --policy_device=cuda --actions_per_chunk=50 --chunk_size_threshold=0.5
```

MolmoAct2 zero-shot (tylko z GPU >= 24 GB, lerobot `main`):

```
lerobot-rollout --policy.path=lerobot/MolmoAct2-SO100_101-LeRobot \
  --rename_map='{"observation.images.top": "observation.images.cam0", "observation.images.wrist": "observation.images.cam1"}' \
  --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=so101 \
  --robot.cameras="{ top: {...}, wrist: {...} }" --task="pick up the pine cone" --duration=30
```

## Zrodla

- https://huggingface.co/docs/lerobot/il_robots (record / train / rollout)
- https://huggingface.co/docs/lerobot/en/smolvla, https://huggingface.co/lerobot/smolvla_base
- https://huggingface.co/docs/lerobot/async (policy server / robot client)
- https://huggingface.co/docs/lerobot/main/en/phone_teleop
- https://huggingface.co/docs/lerobot/main/en/molmoact2, https://huggingface.co/lerobot/MolmoAct2-SO100_101-LeRobot,
  https://allenai.org/blog/molmoact2
- https://huggingface.co/black-forest-labs/flux-3-action-so101
- https://github.com/IliaLarchenko/dot_policy, https://github.com/huggingface/lerobot/pull/739,
  https://github.com/IliaLarchenko/lehome_solution
- https://huggingface.co/datasets/lerobot/svla_so101_pickplace
