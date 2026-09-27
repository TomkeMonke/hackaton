# MAPA - mapa ogrodu z kamery D435, lokalizacja i zygzak po mapie

Stan na 2026-09-27. Branch `frane/mapa-d435`. Szczegoly prob: `docs/LOG.md` (wpisy 2026-09-25 RTAB-Map i
2026-09-27 mapa ogrodu). Pulapki: `docs/HARDWARE.md` 40-42.

## Co to jest

"Lidar" w rozmowach = glebia z kamery RealSense. Lidara nie ma. Kamera na robocie to **D435** (fw 5.11.1.100,
USB 3.2), siedzi na ramieniu SO-101.

Idea: raz nagrac teren kamera, zlozyc z tego mape (RTAB-Map, offline na laptopie), a potem robot staje, robi
jedno zdjecie w pozie "patrz" (kamera poziomo na sciany/plot) i dostaje swoje (x, y, kurs) w mapie. Miedzy
zdjeciami kurs z zyroskopu telefonu (phyphox), droga z czasu. Tak jedzie zygzak (pasy jak kosiarka).
Deterministycznie, bez ML i bez ROS.

## Gdzie sa pliki (NIE w repo - za duze)

| co | laptop frane | Pi |
|---|---|---|
| nagrania (rgb/, depth/, calib/, stamps.txt) | `C:/Users/pawel/mapy/ogrod1`, `ogrod2` | `~/mapy/ogrod1`, `~/mapy/ogrod2` |
| mapa RTAB-Map (`map.db`, `map_cloud.ply`, `map_poses.txt`, `map_preview.png`) | w katalogu nagrania | - |
| mapa do lokalizacji (`map_features.npz`, 1-3 MB) | w katalogu nagrania | `~/mapy/<nazwa>/map_features.npz` |
| RTAB-Map 0.23.8 win64 | `C:/Users/pawel/tools/bin` | - |
| kod do testow na Pi (kopia brancha, nie `~/hackaton`) | - | `~/mapa_test` |

Nagranie ~5 MB/s (10 min = 3 GB). `map_features.npz` wystarczy do jazdy - reszta tylko do budowy mapy.

## Kolejnosc (komendy)

Na Pi, kamera w pozie "patrz" (teleop i panel ramienia zamkniete):

```
python tools/arm_hold.py patrz --config ~/hackaton/pinecone_config.json     # trzyma do Ctrl+C
python tools/record_rgbd.py --out ~/mapy/ogrod3                               # Ctrl+C konczy
```

Jechac WOLNO (obrot < 30 st/s - narzedzie ostrzega), kazde miejsce 2 razy, na koniec wrocic na start.

Na laptopie:

```
scp -r robot@172.20.10.4:mapy/ogrod3 C:/Users/pawel/mapy/        # zerwane scp zostawia dziury w srodku!
python tools/rtabmap_build.py C:/Users/pawel/mapy/ogrod3          # ~8 min, pisze ile kawalkow mapy
python -m pinecone_bot.localize build C:/Users/pawel/mapy/ogrod3  # -> map_features.npz
python -m pinecone_bot.localize eval C:/Users/pawel/mapy/ogrod3   # dokladnosc na klatkach spoza mapy
scp C:/Users/pawel/mapy/ogrod3/map_features.npz robot@172.20.10.4:mapy/ogrod3/
```

Na Pi, jazda (najpierw `--dry-run`; STOP: `tools/estop_server.py` z tego brancha - zna zygzak):

```
python -m pinecone_bot.zygzak --dry-run --config ~/hackaton/pinecone_config.json --map ~/mapy/ogrod3/map_features.npz --lanes 2 --length 2 --spacing 0.6 --first-turn right
```

`--first-turn right` z lewego dolnego rogu pola, `left` z prawego. Zygzak sam ustawia ramie w pozie "patrz"
(`motions/patrz.json`) i trzyma ja do konca (`--no-arm` wylacza).

## Kod

- `tools/record_rgbd.py` - nagranie na Pi w formacie RTAB-Map (kolor + glebia wyrownana, 15 Hz), ostrzezenia.
- `tools/rtabmap_build.py` - mapa jedna komenda (RTAB-Map CLI), sprawdza komplet plikow, podglad z gory.
- `pinecone_bot/localize.py` - lokalizacja z jednego zdjecia: ORB + PnP do klatek kluczowych mapy (punkty 3D z glebi do 6 m).
- `pinecone_bot/zygzak.py` - plan pasow, obrot na zyroskopie, stop + zdjecie co `nav.look_every_m`, nauka predkosci.
- `tools/arm_hold.py`, `motions/patrz.json` - poza kamery do nagrywania i lokalizacji.
- Config: sekcja `nav` w `pinecone_config.json`.

## Wyniki i co nie dziala

- ogrod1 (329 s), ogrod2 (235 s): odometria gubi sie rzadko, ale RTAB-Map sklada mape z 5-9 kawalkow i do
  mapy wchodzi ok. 2/3 nagrania. Reszta terenu jest poza mapa.
- Lokalizacja (laptop 0.12-0.18 s, Pi ok. 0.5 s): mediana bledu 3-4 cm i 1 st; lapie 45-58% klatek; 90% bledow < 0.4 m.
- Ze startu przy plocie (ogrod1) lokalizacja NIE zlapala - tego fragmentu nie bylo w mapie. ogrod2 na zywo niesprawdzona.
- Zygzak w symulacji dziala (robot 0.6x wolniejszy od zadanego trafia w punkty < 0.3 m). Na robocie NIE JECHAL.
- Szyszki w zygzaku niepodpiete: potrzebna poza "szukaj" (kamera w dol) i przelaczanie patrz/szukaj.
- GPS z telefonu odrzucony: 3-10 m bledu przy polu 5 x 4 m.

## Pulapki (skrot)

- Kamera na wylacznosc jednego procesu: `rs_mjpeg_server.py`, `lerobot-record`, zygzak - jeden naraz.
- RealSense pamieta zamrozona ekspozycje po poprzednim procesie (`lock_auto`) - bez blokady `camera.py` odmraza.
- RTAB-Map win64 pada z 0xC0000135 bez `msvcr110.dll`/`msvcp110.dll` (x64) w katalogu bin.
- `dataRecorder` odtwarza nagranie w czasie rzeczywistym - import trwa tyle, ile nagranie.
- Pi pada przy slabym zasilaniu - kopiowanie nagrania po kablu (192.168.137.5) jest pewniejsze niz po hotspocie.
