# ARM_FRAMES - zera stawow vs URDF i kamera na ramieniu (instrukcja dla sesji przy Pi)

Stan na 2026-09-27. Dwa narzedzia, oba do odpalenia NA Pi (ramie i kamera sa
wpiete w Pi), interaktywnie w terminalu operatora, z wylacznikiem w rece.
Napisane i przetestowane na laptopie bez sprzetu (28 testow); NA SPRZECIE
JESZCZE NIE URUCHOMIONE - pierwszy przebieg to tez test narzedzia.

Po co: zanim ktokolwiek liczy IK (placo, GraspGenX) trzeba wiedziec, czy
"stopnie" lerobota (kalibracja so101.json z recznie poprawionym barkiem) to
katy, ktorych oczekuje `so101_new_calib.urdf` (docs/HARDWARE.md, pulapka 12),
a potem gdzie wzgledem chwytaka siedzi kamera. Bez tego kazda poza z kamery
trafia w zle miejsce.

## 0. Przygotowanie (sesja przy Pi)

1. Branch: `git fetch && git checkout claude/lerobot-policy-setup-2a250e` (albo master po merge).
2. Kod na Pi: `PI_HOST=robot@172.20.10.4 bash deploy/push_to_pi.sh` (kopiuje `tools/`,
   `tests/`, `pinecone_bot/`; UWAGA: nadpisuje `pinecone_config.json` na Pi - wartosci
   z Pi maja byc w repo). Jesli laptop nie widzi Pi, najpierw `netsh wlan show interfaces`:
   laptop musi byc na hotspocie "iPhone pawel", sam przeskakuje na "hacker-bloc".
3. Na Pi jest juz: lerobot 0.6.1 + placo (IK/FK), pyrealsense2, URDF z siatkami w
   `~/hackaton/examples/phone_to_so100/SO101/` (narzedzia same go znajduja; zapas:
   `so101_urdf/so101_new_calib.urdf`). Sprawdzenie bez sprzetu:

   ```
   ssh -t robot@172.20.10.4 "cd ~/hackaton && .venv/bin/python tools/frame_check.py --fake"
   ```

   Ma wypisac FK dla pozy teraz/URDF zero/HOME i przejsc przez stawy na atrapie.
4. Kamera siedzi na ramieniu: zanim ramie ruszy, upewnij sie, ze kabel USB kamery ma luz
   na caly zakres ruchu i ze HOME nie uderza w kamere (STATUS: `--no-home` w panelu).
5. `shoulder_lift` bywa poza zakresem kalibracji w pozie spoczynkowej (STATUS). Zacznij
   od pozy, w ktorej wszystkie stawy sa w zakresie (`./arm.sh status`), inaczej lerobot
   przytnie ruch i "zmierzony ruch stawu" bedzie mniejszy od zadanego.

## 1. Zera i znaki stawow: `tools/frame_check.py`

```
ssh -t robot@172.20.10.4 "cd ~/hackaton && .venv/bin/python tools/frame_check.py --delta 15"
```

Przebieg: odczyt stawow, FK (placo) -> polozenie koncowki `gripper_frame_link` w
ukladzie podstawy ramienia [cm]. Potem dla `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll` po kolei: Enter = ruch tylko tego stawu o +15 st, narzedzie
wypisuje, co MODEL przewiduje ("koncowka: +X 8.1 cm, -Z 2.3 cm (w dol)"), Ty patrzysz na
ramie i odpowiadasz `t` (tak, tak sie ruszyla) / `n` (nie) / wpisujesz notatke. Ramie wraca
i idzie nastepny staw. `s` pomija staw, `q` konczy. Raport: `frame_check.json` (wklej do
LOG i issue).

Jak czytac osie: Z to gora/dol (pewne). Ktora os to "przod" ustalasz sam na pierwszym
stawie: `shoulder_pan` przy wyprostowanym ramieniu przesuwa koncowke w bok, wiec os, ktora
model podaje dla pan, to os boczna (Y), a pozostala pozioma to przod (X). Zapisz to w LOG.

Interpretacja wyniku:
- wszystkie `t`: zera i znaki zgodne, IK z placo na tym URDF mozna probowac od razu (krok 2).
- ktorys `n`: staw ma zly znak albo zle zero wzgledem URDF. Poprawka idzie w warstwie
  miedzy lerobotem a kinematyka (offset i znak na staw; przyklad z lerobot backwardcomp:
  `shoulder_lift' = -(shoulder_lift - 90)`, `elbow_flex' = elbow_flex - 90`), NIGDY w pliku
  kalibracji serw i NIGDY przez `lerobot calibrate`. Zeby odroznic "zly znak" od "zle zero":
  zly znak = ruch dokladnie przeciwny do przewidywanego; zle zero = kierunek podobny, ale
  wielkosc/skladowe sie nie zgadzaja (koncowka "obraca sie wokol zlego punktu").
- `zmierzony ruch stawu` duzo mniejszy niz 15 st: staw uderzyl w limit kalibracji, nie w
  URDF - zacznij z innej pozy.

## 2. Kamera na ramieniu (reka-oko): `tools/hand_eye_calib.py`

Szukamy stalej T_gripper_cam. Potem `T_base_cam = FK(stawy) @ T_gripper_cam` daje pozycje
i kierunek patrzenia kamery dla dowolnych stawow ("jak rusza sie kamera, gdy rusza sie
ramie"). Klasyczne AX = XB (Park-Martin w numpy; `cv2.calibrateHandEye` tylko jako kontrola
krzyzowa, bo OpenCV 5.0 juz tej funkcji nie ma), zero ML.

1. Marker: na laptopie `python tools/hand_eye_calib.py marker --out aruco_marker.png`
   (ArUco DICT_4X4_50, id 0). Wydrukuj, przyklej na cos sztywnego, zmierz linijka bok
   CZARNEGO kwadratu (np. 0.052 m). Poloz nieruchomo na stole tam, gdzie zwykle leza szyszki.
2. Probki (torque OFF, ramie prowadzisz reka jak w `record_motion.py`):

   ```
   ssh -t robot@172.20.10.4 "cd ~/hackaton && .venv/bin/python tools/hand_eye_calib.py collect --marker-len 0.052 --save-frames frames_he"
   ```

   Enter = probka (stawy + poza markera w kamerze), `q` = koniec. Potrzeba 12-20 probek,
   marker caly w kadrze, i przede wszystkim ROZNE ORIENTACJE kamery (krec nadgarstkiem,
   lokciem, patrz na marker z lewa/prawa/z gory), nie same przesuniecia - inaczej uklad
   jest zle uwarunkowany. "marker NIE wykryty" = popraw kadr, probka nie liczy sie.
   Wynik: `hand_eye_samples.json` (+ `frames_he/*.png` do debugowania).
3. Rozwiazanie (na Pi, bo FK wymaga placo):

   ```
   ssh -t robot@172.20.10.4 "cd ~/hackaton && .venv/bin/python tools/hand_eye_calib.py solve"
   ```

   Wypisuje T_gripper_cam (przesuniecie [m], rpy [st]) i residua: rozrzut polozenia
   markera w bazie po wszystkich probkach. RMS < 10 mm i < 2 st = dobrze. Duze residua =
   za malo roznych orientacji, zly `--marker-len` albo zle zera stawow (wroc do kroku 1).
   Wynik: `camera_on_arm.json`.
4. Sprawdzenie na zywo: `... hand_eye_calib.py predict` (czyta stawy z ramienia) wypisuje,
   gdzie w bazie jest kamera i wzdluz jakiego wektora patrzy. Porownaj z linijka.

Zdrowy rozsadek do porownania: kamera jest kilka cm od ramki chwytaka, wiec
`translation_m` rzedu 0.02-0.10 na skladowa. Metr = cos poszlo zle.

## 3. Co z tym dalej

- `camera_on_arm.json` + FK to brakujaca "kalibracja kamera -> ramie" z LOG 25.09.
  Punkt z detektora (piksel + glebia) -> deprojekcja intrinsics -> punkt w kamerze ->
  `T_base_cam @ p` = punkt w bazie ramienia -> IK placo. To fundament pod GraspGenX
  (pozy chwytu w kamerze -> w bazie -> filtr osiagalnosci 5-DOF -> IK).
- Pliki wynikowe (`frame_check.json`, `hand_eye_samples.json`, `camera_on_arm.json`) NIE
  sa w .gitignore celowo: `camera_on_arm.json` ma trafic do repo (jak `pinecone_config.json`),
  pozostale wkleic do LOG/issue i skasowac.
- Koniec sesji: STATUS "Dziala"/"Nie dziala", wpis w LOG z tabela werdyktow z
  `frame_check.py` i residuami z `solve`.
