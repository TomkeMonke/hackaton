# Prezentacja: Pinecone Robot - How It Works

Online: https://tomkemonke.github.io/hackaton/docs/presentation/

34 slajdy po angielsku, do nauki: jak robot widzi, mierzy, jezdzi i uczy sie chwytac.

| Czesc | Slajdy |
|---|---|
| Intro | misja, sprzet (schemat polaczen), mapa kodu w repo |
| 01 See | HSV, detekcja krok po kroku na prawdziwej klatce, morfologia, dlaczego kolor jest kruchy |
| 02 Measure | kamera stereo (Z = f x b / d), mediana glebi, RANSAC, martwa strefa 0.31 m |
| 03 Drive | regulator P w obrazie (z przykladem liczbowym), maszyna stanow, kurs z telefonu, wypadek z ramieniem |
| 04 Grab | 3 proby przed AI, imitation learning, dataset, jeden chwyt klatka po klatce, ACT w srodku, chunking, CVAE, ResNet18, trening, ograniczenia |
| 05 Lessons | stan, lekcje, slowniczek, literatura |

Liczby sa z repo (STATUS, LOG, configi), z wag modelu (`models/act_so101_grasp2/007000`, liczba parametrow)
i z datasetu `datasets/so101_grasp2` (klatki z kamery, trajektorie stawow). Obrazki w `img/`:
klatki z wideo datasetu, maski z detektora `pinecone_bot/detector.py`, siatka 50 pierwszych klatek
z ramkami z progu z `pinecone_config.json`.

## Pliki

- `index.html` - gotowa prezentacja, jeden plik + `img/`. Strzalki / spacja / klik = nastepny slajd,
  `#12` w adresie = slajd 12.
- `src/deck.json` - kolejnosc slajdow, sekcje, fonty.
- `src/slides/<id>.html` - jeden slajd = jeden plik (format Slides z claude.ai, notatki prelegenta w `<aside>`).
- `src/assets.json` - mapa `/_blob/<id>` (obrazek w claude.ai) -> `img/<plik>`.
- `build.py` - sklada `src/` w `index.html`: `python docs/presentation/build.py`.
