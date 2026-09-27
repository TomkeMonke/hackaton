# Robot zbierajacy szyszki

Robot na bazie hoverboardu z ramieniem SO-101 i kamera RealSense D415, sterowany z Raspberry Pi 5.
Jezdzi po trawniku, znajduje szyszki, chwyta je i wrzuca do pojemnika na sobie.
Sterowanie jest deterministyczne (`pinecone_bot/`): ramie odtwarza nagrane ruchy, baza ustawia szyszke
w miejscu chwytu na podstawie obrazu, calosc spina prosta maszyna stanow. Bez ML, bez LLM w petli.

## Zacznij tutaj (kazda sesja)

1. `git checkout master && git pull`
2. Przeczytaj [docs/STATUS.md](docs/STATUS.md): co dziala, co nie, nastepne 3 kroki, kto ma robota. Jeden ekran.
3. Wez zadanie z tablicy Projects (link w STATUS) albo zaloz issue. Branch `<nick>/<opis>`.
4. Zasady pracy (branch, PR, review, koniec sesji): [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).

## Pierwszy raz? Setup

[docs/SETUP.md](docs/SETUP.md): laptop (Python 3.12, venv, testy, symulacja) i Raspberry Pi (ssh, IP, venv, udev, lerobot).

Na laptopie, bez sprzetu:

```
python -m venv .venv && .venv\Scripts\activate       # Linux/Mac: source .venv/bin/activate
pip install -r requirements-pinecone.txt
python -m pytest tests -q
python -m pinecone_bot.main --sim --show
```

## Odpalanie robota (dzien ze sprzetem)

[docs/RUNBOOK.md](docs/RUNBOOK.md): checklista na rano i kolejnosc narzedzi na Pi:
przestaw kamere -> `snap_frames` -> `calibrate_hsv` -> `record_waypoints` -> `calibrate_target` ->
`base_test` -> `--dry-run` -> `--real`. Tam tez strojenie i przelaczenie hovera na bipropellant.

Zanim dotkniesz sprzetu: [docs/HARDWARE.md](docs/HARDWARE.md) (pulapki: kalibracja barku, martwa strefa kamery,
nieliniowe kola, port szeregowy).

## Struktura repo

| Sciezka | Co to |
|---|---|
| `pinecone_bot/` | glowny stos: config, detektor, kamera, sterowniki bazy (Xiao, bipropellant, sim), ramie, maszyna stanow, symulator, `main.py` |
| `tools/` | narzedzia: `camera_geometry` (gdzie zamontowac kamere), `snap_frames`, `calibrate_hsv`, `calibrate_target`, `record_waypoints`, `arm_play`, `base_test`, `bip_probe` (czy hover gada bipropellantem) |
| `motions/` | nagrane ruchy ramienia (`grasp_mid.json`, `home.json`, `drop_box.json`) |
| `tests/` | testy bez sprzetu (`python -m pytest tests -q`), odpalane tez w GitHub Actions |
| `pinecone_config.json` | jedyny plik konfiguracji robota; wypelniaja go narzedzia z `tools/` |
| `arm_control.py`, `arm.sh` | sterowanie ramieniem przez lerobot (CLI i modul dla `pinecone_bot/arm.py`) |
| `web_control.py`, `frontend.html`, `recordings/` | panel webowy do jazdy hoverboardem (WASD, osemka, pokrycie), nagrania z panelu |
| `xiao_send_pwm.ino` | firmware Xiao RP2040: `a<speed> b<steer>` po USB, watchdog 500 ms |
| `detect_floor_objects.py`, `scan_cones.py`, `test_detect_gates.py` | detekcja obiektow z glebi (plaszczyzna podlogi), skan szyszek z dystansem |
| `rs_preview.py`, `rs_snapshot.py`, `rs_mjpeg_server.py` | podglad i zrzuty z RealSense |
| `drive_step.py`, `auto_collect.py` | pojedynczy krok kolami, stary podjazd krokowy (do recznych testow) |
| `bag_to_rtabmap.py`, `render_map_preview.py` | mapa ogrodu RTAB-Map (issue #7, odlozone) |
| `so101_urdf/` | model URDF ramienia |
| `deploy/` | `setup_pi.sh` (instalacja na Pi), `push_to_pi.sh` (wyslanie kodu), udev, autostart panelu |
| `docs/` | STATUS (stan), SETUP, PANEL, RUNBOOK, HARDWARE, LOG (dziennik sesji), CONTRIBUTING (zasady) |
| `legacy/` | odlozone: IK i slepy podjazd, nagrania sprzed naprawy barku, stare skrypty jazdy ([legacy/README.md](legacy/README.md)) |
| `requirements-pinecone.txt` | zaleznosci laptopa (numpy, opencv, pyserial, pytest); Pi: `requirements-pi.txt` |

## Dokumentacja

- [docs/STATUS.md](docs/STATUS.md): stan projektu, aktualizowany w kazdym PR.
- [docs/LOG.md](docs/LOG.md): dziennik sesji, tylko dopisywanie.
- [docs/RUNBOOK.md](docs/RUNBOOK.md): jak odpalic robota krok po kroku.
- [docs/PANEL.md](docs/PANEL.md): polaczenie z Pi i odpalenie panelu jazdy + ramienia.
- [docs/SETUP.md](docs/SETUP.md): instalacja laptop / Pi.
- [docs/HARDWARE.md](docs/HARDWARE.md): sprzet i pulapki.
- [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md): jak pracujemy.
- [docs/WORKFLOW_CLAUDE.md](docs/WORKFLOW_CLAUDE.md) i [AGENTS.md](AGENTS.md): jak pracowac z Claude tanio, kiedy wlaczac agentow.
- Issues: [#7 mapa ogrodu](https://github.com/pawel120/hackaton/issues/7), [#8 zasilanie 12 V](https://github.com/pawel120/hackaton/issues/8).
