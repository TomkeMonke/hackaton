# Chwytanie szyszki samym ramieniem - jak kontynuowac (stan 2026-09-26 wieczor)

Cel: robot stoi (baza sie nie rusza), ramie SO-101 z kamera D435 na przedramieniu widzi szyszke
w pozycji HOME, samo ja chwyta, wrzuca do sloika i wraca do HOME. Deterministycznie: model z demonstracji
reka (regresja), bez ML/LLM w petli.

Kod: `tools/live_grasp/` (branch `pawel/arm-home-cam`). Skrypty ida na Pi przez `ssh ... python -`
(stdin), nic nie jest kopiowane na Pi. Uruchamiasz je z laptopa (git-bash, katalog repo)
przez `bash tools/live_grasp/pi.sh <skrypt> [VAR=plik] [--bg nazwa]`.

## Co jest zrobione i co wiemy

- HOME = poza `arm_control.HOME_POSE` (pan -5.45, lift 88.3, elbow 7.56, wrist -87.87, roll 88.88).
  UWAGA: nowy HOME jest w repo, ale NIE wgrany na Pi (`arm_control.py`, `pinecone_bot/arm.py`, `motions/home.json`).
  Skrypty w `tools/live_grasp/` maja HOME wpisany na sztywno, wiec dzialaja bez tego.
- Wykrywanie szyszek w HOME z glebi dziala (plaszczyzna ziemi RANSAC, szyszka = wybrzuszenie 1.2-8 cm,
  chwytak wyciety maska koloru niebieskiego, kabel serwa wyciety prostokatem). Po zmroku kolor nie dziala:
  maska chwytaka i wykrywanie reki potrzebuja swiatla w pokoju.
- Ruch do sloika i z powrotem: nagrany reka, dziala przy odtwarzaniu (`data/jar_cycle.json`,
  `pick.py` ma go jako `JAR_UP` / `JAR_BACK`). Sloik musi stac tam, gdzie przy nagraniu (podstawa ok. -22 st).
- Model "szyszka w kamerze -> stawy chwytu" (`data/grasp_model.json`, 8 probek) jest ZA SLABY:
  blad ok. 6 cm, w probach szczeki trafialy obok szyszki albo ja spychaly. Powody: polowa etykiet
  z sesji 1-2 niepewna (znikalo kilka szyszek naraz), rozne style chwytu (bark 61-92 st).
- Pulapki sprzetu: serwa maja niskie P (16) - bez korekty calkujacej zostaje 2-4 st bledu (funkcja `go()`
  w skryptach ja robi); podstawa ma w EEPROM zakres tylko +-23 st (1786-2308); bark stoi przy
  Max_Position_Limit (88.4 st); serwo chwytaka przy scisku do 0 potrafi zniknac z magistrali - trzymac cel 3-4
  ponizej odczytu; Pi na powerbanku raz sie zrestartowal (zasilanie).
- Kamera na przedramieniu: gdy szczeki sa nad szyszka, zaslaniaja ja - korekta ze zdjec przy samym chwycie
  nie dziala (probowane: `grasp_servo.py`, `servo.py`, `attempt.py`). Wyjscie: dobry model z czystych danych
  (ponizej) albo kamera na maszt (STATUS, krok 1).

## Nastepny krok: czyste uczenie (ok. 15 min) -> model -> test

Warunki: swiatlo w pokoju, laptop i Pi w tej samej sieci (hotspot), sloik w tym samym miejscu.

1. Sprawdz, ze Pi odpowiada i co widzi kamera:
   ```
   ssh robot@robot.local uptime
   bash tools/live_grasp/pi.sh snap_only.py
   ```
   Zdjecie: `tools/live_grasp/out/snap_only.jpg` (lewo kolor, prawo wysokosc nad ziemia 0-6 cm).
   Przed uczeniem nic nie moze trzymac kamery ani ramienia (`ssh robot@robot.local 'pgrep -af python'`).

2. Uczenie w tle na Pi (12 chwytow, max 20 min):
   ```
   bash tools/live_grasp/pi.sh teach_clean.py --bg teach
   ssh robot@robot.local tail -f /tmp/teach.err
   ```
   Przebieg jednej rundy (program mowi w logu, co robi):
   - robot w HOME czeka, az zobaczy DOKLADNIE 1 szyszke i zadnej reki (2 zgodne zdjecia);
   - szeroko otwiera chwytak i wylacza torque = "twoja kolej";
   - chwytasz ja reka, ZAWSZE tym samym stylem: bark nisko jak w HOME, pracuja podstawa, lokiec,
     nadgarstek, szczeki schodza z gory. Zamkniecie chwytaka (ponizej 25, stabilnie 0.5 s, ramie > 5 st od HOME) = zapis;
   - wrzucasz do sloika, otwierasz chwytak, puszczasz ramie; po 5 s robot wraca do HOME;
   - kladziesz nastepna szyszke w NOWYM miejscu (blisko/daleko, lewo/srodek/prawo), reka poza kadr.

3. Pobierz dane i dopasuj model (laptop):
   ```
   ssh robot@robot.local cat /tmp/teach.jsonl > tools/live_grasp/data/teach_session3_full.jsonl
   python tools/live_grasp/fit.py tools/live_grasp/data/teach_session3_full.jsonl tools/live_grasp/data/grasp_model.json
   ```
   Patrz na wiersz "blad zostaw-jedna [st]": podstawa < 2 st, bark/lokiec/nadgarstek < 4 st = mozna testowac.
   Wiekszy blad = wiecej probek w tych samych warunkach albo sprawdz zdjecia HOME (pole `img` w jsonl).
   Do repo wrzucaj wersje bez zdjec:
   ```
   python -c "import json,sys;[print(json.dumps({k:v for k,v in json.loads(l).items() if k!='img'})) for l in open(sys.argv[1])]" tools/live_grasp/data/teach_session3_full.jsonl > tools/live_grasp/data/teach_session3.jsonl
   ```

4. Test: jedna szyszka w polu, ktore bylo w uczeniu (program odmawia poza nim):
   ```
   bash tools/live_grasp/pi.sh pick.py MODEL=data/grasp_model.json
   ```
   Log: wybrana szyszka, poza chwytu, odczyt chwytaka (>= 6 = trzyma), sloik, HOME. Zdjecie: `out/pick.jpg`.
   Seria z poprawka po kazdej porazce (szyszka przepchnieta -> cel przesuwa sie o 70% przepchniecia):
   ```
   bash tools/live_grasp/pi.sh pick_loop.py MODEL=data/grasp_model.json
   ```

## Inne narzedzia w katalogu

- `rec.py` - nagranie ruchu reka (torque off, stawy 30 Hz + zdjecie co 0.5 s): `REC_SECONDS` w env na Pi
  nie jest przekazywany przez pi.sh - domyslnie 120 s. W tle: `pi.sh rec.py --bg rec`, potem
  `ssh robot@robot.local cat /tmp/rec.out > plik`. Przyklad: `data/rec_demo_120s.jsonl` (9 cykli chwyt -> sloik).
- `replay.py` - odtworzenie nagranego cyklu (tempo x0.5): `pi.sh replay.py CYCLE=data/jar_cycle.json`.
  Szyszka musi lezec tam, gdzie przy nagraniu.
- `step.py` - jeden ruch + zdjecie: `pi.sh step.py ARGS=plik.json`, plik np. `{"rel": {"shoulder_pan": 3}, "sec": 1.0}`.
- `snap_only.py` - zdjecie + pozycje stawow, bez ruchu ramienia.
- Eksperymenty, ktore NIE zadzialaly (do wgladu): `attempt.py` (celowanie w pozycji "nad" z koloru+glebi),
  `servo.py` (serwo 3D z Jakobianem, zle mierzona koncowka szczeki), `grasp_servo.py` (korekta w pozie chwytu -
  szczeki zaslaniaja szyszke).

## Bezpieczenstwo

- Przy kazdym uruchomieniu z ruchem: czlowiek przy robocie, reka przy wylaczniku.
- Przed `teach_clean.py` i `rec.py` torque jest wylaczany - ramie jest luzne, trzymaj je.
- Nie uruchamiaj `lerobot calibrate`. Nie zmieniaj limitow pozycji w EEPROM serw bez decyzji zespolu
  (podstawa +-23 st to obecny limit, stare wartosci 1786/2308).
