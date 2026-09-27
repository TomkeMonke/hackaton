# GraspGenX (NVlabs) - notatka

**Stan na:** 2026-09-27. Zrodlo: README repo (nic nie odpalone u nas).
Repo: https://github.com/NVlabs/GraspGenX, poprzednik: https://github.com/NVlabs/GraspGen.
Paper: "GraspGen-X: Cross-Embodiment 6-DOF Diffusion-based Grasping", CVPR 2026.

## Co to jest

Generator chwytow 6-DOF dla dowolnego chwytaka. Poprzednik GraspGen wymagal osobnego modelu na
kazdy chwytak. GraspGenX to jeden model warunkowany opisem chwytaka, wiec dziala tez na chwytakach,
ktorych nie widzial w treningu, bez ponownego treningu.

## Wejscie / wyjscie

- **Wejscie:** chmura punktow obiektu (posegmentowana z kamery glebi) albo cala scena z segmentacja;
  alternatywnie mesh (`.obj`, `.stl`, `.ply`, USD).
- **Wyjscie:** lista poz chwytaka 6-DOF (pozycja + orientacja) z wynikiem; dla sceny opcjonalnie
  odfiltrowane chwyty kolidujace z otoczeniem.

## Jak dziala

- Model dyfuzyjny generuje kandydatow, drugi model (dyskryminator) ich ocenia.
- Tryb domyslny **GraspMoE**: chwyty z dyfuzji + proste chwyty z bounding boxa obiektu, wszystko
  oceniane dyskryminatorem.
- Opis chwytaka dla modelu: przestrzen przemiatana przez palce (otwarty i polotwarty), stany stawow
  otwarty/zamkniety, typ (`parallel_2f`, `revolute_2f`, `revolute_3f`).
- Trening: ponad 2 mld symulowanych chwytow, 32 proceduralne chwytaki z 6 rodzin, 8000+ obiektow.

## Wlasny chwytak

1. `uv run python scripts/gripper_config_wizard.py --urdf <gripper.urdf> --name <nazwa>` - kreator
   w przegladarce.
2. 6 krokow: baza (Z+ = kierunek podejscia, X+ = kierunek zamykania), poza otwarta, przestrzen
   przemiatana otwarta i polotwarta, podglad animacji, zapis.
3. Wynik w `assets/x_grippers/<nazwa>/`: `config.json`, URDF + meshe, `vis_mesh.obj`.
4. Sprawdzenie: `scripts/vis_gripper_desc.py`.

Gotowe chwytaki m.in.: `robotiq_2f_85`, `robotiq_2f_140`, `franka_panda`, `inspire_hand`,
`barrett_hand`, `unitree_g1`. Pelna lista: `scripts/list_grippers.py`.

## Instalacja i licencja

- `uv sync` do inferencji, Docker do treningu; Python 3.10 / 3.11.
- Checkpointy i opisy chwytakow pobieraja sie same z HuggingFace przy pierwszym uzyciu.
- Katalogi: `graspgenx/` (biblioteka), `end2end/` (pelny pick-and-place: cuRobo + symulacja
  Newton/MuJoCo), `mcp/` i `client-server/` (model jako osobny serwer).
- Licencja: kod Apache 2.0, wagi NVIDIA Open Model License (GraspGen mial bardziej restrykcyjna
  licencje badawcza NVIDIA).
- Niewydane: pelny dataset treningowy.
- Sprzet: README nie podaje wymagan GPU ani szybkosci. GraspGen: PyTorch + CUDA, testy na
  V100/A100/H100/L40s, ok. 20 Hz bez TensorRT.

## Znana slabosc

Issue #8 (wybieranie z pudla): uzytecznych chwytow ok. 4 na 400. Model wyglada na trenowany na
obiektach lezacych swobodnie na stole, a wynik dyskryminatora prawie nie przewiduje, ktory chwyt
zadziala. Brak odpowiedzi maintainerow.

## Co to znaczy dla naszego robota

- **Brak SO-101 w pudelku.** Dokumentacja nie wspomina SO-101 ani LeRobot. Mamy URDF w
  `so101_urdf/`, wiec kreator w teorii zadziala. Chwytak SO-101 ma jedna szczeke stala i jedna
  obrotowa - pewnie typ `revolute_2f`, niesprawdzone.
- **Pi 5 tego nie uciagnie.** Brak GPU z CUDA - model musialby chodzic na laptopie z karta NVIDIA
  i wysylac chwyty na Pi przez `client-server/`.
- **Potrzebne IK.** Model daje tylko poze chwytaka. Nasze ramie odtwarza nagrane ruchy, wiec
  trzeba by wrocic do IK (stare podejscie na `ikpy` w `legacy/ik_approach/`).
- **Dla nas raczej przesada.** Szyszka lezy na plaskiej trawie i chwytamy z gory, wiec obecne
  "baza ustawia szyszke w stalym miejscu obrazu + jeden nagrany ruch" (patrz `docs/STACK.md`)
  jest na hackaton duzo prostsze.
