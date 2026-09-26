# live_grasp - chwytanie szyszki samym ramieniem, na zywo (eksperyment 2026-09-26)

Skrypty wysylane na Pi przez `ssh ... python -` (stdin), bez kopiowania plikow na Pi.
Stan roboczy, NIE gotowe narzedzie. Wynik dnia: 1 chwyt udany po uczeniu reka + recznym celowaniu,
automat 0/4 prob z glebi (szczeki zamykaly sie nad szyszka albo ja spychaly).

- `percept.py` - glebia: plaszczyzna ziemi (RANSAC), wysokosc nad ziemia, szyszki = wybrzuszenia 1.2-8 cm.
- `attempt.py` - jedna proba: pozycja "nad", celowanie podstawa + barkiem (szyszka do punktu `align_x/align_y`
  na obrazie), zejscie ruchem z uczenia (+`goff_*`), zacisk, sprawdzenie chwytaka, HOME. Szyszki z glebi + HSV
  (kolor widzi w cieniu stereo szczeki, glebia nie). Stan w `state.json`.
- `run.sh N` - odpala `attempt.py` na Pi, zapisuje log i zdjecie do `out/`.
- `step.py` - pojedynczy ruch + zdjecie (ARGS: {"rel"|"abs": {...}, "sec": 1.0}).
- `servo.py` - proba serwa 3D z glebi (Jakobian + Broyden): NIE dziala, koncowka szczeki z mapy wysokosci byla
  mierzona zle. Zostawione do wgladu.

Lekcje (szczegoly w docs/LOG.md 2026-09-26):
- serwa maja niskie P (16): bez korekty calkujacej zostaje 2-4 st bledu; bark stoi na Max_Position_Limit (88.4).
- kamera siedzi na przedramieniu: szczeki przesuwaja sie w obrazie przy ruchu nadgarstka; w pozycji "nad" stoja stale.
- po zmroku kolor jest bezuzyteczny (ekspozycja max 100 ms i tak ciemno); glebia (laser 360) dziala.
- punkt celowania z jednego udanego chwytu nie przenosi sie na inne szyszki - nastepny krok: uczyc reka 2-3 przyklady
  z zapisem zdjecia z pozycji "nad" i brac punkt celowania z nich.
