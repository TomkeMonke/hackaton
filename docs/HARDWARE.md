Aktualizuj ten plik, gdy odkryjesz nowa pulapke. Historia sesji jest w
docs/LOG.md.

# Sprzet robota

Platforma: podwozie-hoverboard + ramie SO-101 na wierzchu, docelowo
sterowane z Raspberry Pi 5 (zamiast Windows PC).

- **Raspberry Pi 5 (8 GB)** - centralny kontroler. Jedzie na robocie,
  urzadzenia podpiete po USB: kamera D415 (USB3), ramie (adapter CH343),
  Xiao (USB-serial). System: Raspberry Pi OS Lite 64-bit na pendrivie USB
  (brak karty SD) - **nie wyciagac pendrive'a z dzialajacego Pi**, to dysk
  systemowy.
- **Seeed Xiao RP2040** - kontroler hoverboardu, firmware
  `xiao_send_pwm.ino`. Protokol po USB-serial: `a<speed> b<steer>\n`,
  watchdog 500 ms (brak komend = stop). Steruje H-bridge (PWM+DIR) na
  pinach `LEFT_PWM_PIN/LEFT_DIR_PIN/RIGHT_PWM_PIN/RIGHT_DIR_PIN` (D0-D3).
  Flagi w firmware `SWAP_LR` / `INVERT_DIR` korygujace montaz "do gory
  nogami" wzgledem oryginalnego zalozenia (zamiana L/R + odwrocenie
  kierunku obu silnikow) - patrz pulapka 3 nizej.
- **Hoverboard** - 2x silnik + wlasny sterownik H-bridge, napedzany przez
  Xiao jak wyzej.
- **Ramie SO-101** (TheRobotStudio/Hugging Face lerobot), serwa Feetech
  STS3215, URDF w `so101_urdf/so101_new_calib.urdf`. Kontroler przez
  adapter USB CH343 (VID:PID `1a86:55d3`). Kalibracja serw zapisana POZA
  repo: `~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json`
  (id ramienia `so101`) - na laptopie i osobno na Pi, trzeba kopiowac
  recznie (nie jest w gicie).
- **Kamera Intel RealSense D415** - USB. Serial 105422060821, firmware
  5.17.0.10. Montaz docelowy: platforma robota, 12 cm nad ziemia.
- **udev / stabilne nazwy portow na Pi** (`deploy/99-robot.rules`):
  `/dev/robot-arm` = kontroler ramienia (CH343, `1a86:55d3`),
  `/dev/robot-drive` = Xiao (`2e8a:000a`). Na Windows nazwy portow to
  zwykle COM (numer zmienia sie po replugu - patrz pulapka 4).
- Odrzucona platforma obliczeniowa: **Jetson Nano P3450** - patrz
  pulapka "Jetson Nano" nizej.

## Zasilanie

Krotko: zasilanie 12 V z akumulatora jest osobnym, nierozwiazanym
tematem (GitHub issue #8, przypisana osoba od elektroniki). Baterie AA
1,5 V NIE nadaja sie do zasilania Pi. Pi 5 + D415 na slabym zasilaniu
(np. slaby powerbank/zasilacz) znika z sieci - na dobrym powerbanku
dziala stabilnie. Szczegoly i decyzje - patrz issue #8.

## Pulapki (przeczytaj zanim dotkniesz)

1. **NIE uruchamiac `lerobot calibrate`.** Nadpisuje recznie poprawiony
   offset barku (id2, patrz pulapka 3) i zakres znow przejdzie przez
   zero enkodera - ramie znowu bedzie jechalo "naokolo" zamiast prosto
   do celu. Backup pliku kalibracji sprzed poprawki jest na Pi:
   `~/so101.json.bak-204746`.

2. **lerobot adresuje serwa PO NAZWIE, ze sztywnej listy w kodzie**
   (`lerobot/robots/so_follower/so_follower.py`: `shoulder_lift`=id2,
   `elbow_flex`=id3). Pole `"id"` w pliku `so101.json` jest IGNOROWANE.
   Proba "naprawy" przez zamiane wpisow `shoulder_lift`/`elbow_flex` w
   pliku kalibracji zamienia tylko offsety/zakresy miedzy przegubami -
   kazdy z dwoch przegubow zostaje znormalizowany zakresem tego
   drugiego, wiec kierunek dalej wychodzi zly. To byla bledna diagnoza w
   jednej z sesji ("zamiana ID serw") - nie powtarzac. Nie naprawiac
   mapowania przez edycje JSON-a.

3. **Fizyczny zakres barku (id2, `shoulder_lift`) przechodzil przez zero
   enkodera** (Present 4095->0). Serwo w trybie pozycji nie przejdzie
   przez 0, wiec do celu po drugiej stronie jechalo "naokolo", w
   podloge (widac w nagraniu: -82 -> -151 -> +139 -> 128 st). Naprawa
   (bez recznej kalibracji, policzona z nagrania demonstracji):
   `fix_shoulder_offset.py` ustawia `Homing_Offset` i limity tak, zeby
   srodek zakresu byl daleko od zera enkodera (id2: offset 1977, limity
   1006..3089; id3 przywrocone do wartosci sprzed sesji). Zapisuje do
   EEPROM serw i do `so101.json`. `Homing_Offset` na Feetech ma zakres
   +-2047 - wieksze przesuniecia licz modulo 4096.

4. **Ramie montowane "do gory nogami"** wzgledem oryginalnego zalozenia
   projektu wymaga w firmware Xiao `SWAP_LR=true` i `INVERT_DIR=true`
   (zamiana L/R + odwrocenie kierunku obu silnikow). Bez tego robot
   jedzie w zla strone mimo poprawnych komend z wyzszej warstwy.

5. **Sterowanie roznicowe kolami**: `steer` musi byc wyraznie mniejsze
   niz `speed`, inaczej jedno kolo idzie na minus i robot wiruje w
   miejscu zamiast jechac lukiem.

6. **Dwa mnozace sie suwaki predkosci** (limit predkosci x predkosc
   trybu auto) potrafia zejsc do wartosci za niskiej, zeby fizycznie
   ruszyc silniki (martwa strefa PWM). Przy "robot nic nie robi" zawsze
   sprawdz realny `pwm_speed`/`pwm_steer`, nie tylko czy komenda w ogole
   leci.

7. **COM port bywa niestabilny** po zawieszeniu USB CDC (Windows error
   31, "urzadzenie nie dziala") - zwykle pomaga fizyczny replug kabla;
   programowy disable/enable urzadzenia wymaga uprawnien admina.

8. **Krok kol (`drive_step.py`) jest mocno NIELINIOWY**, nie licz go
   proporcjonalnie do czasu. Zmierzone przy `--speed 150`: 0.05 s (jeden
   tick 80 ms) ~ 2.9 cm do przodu, ale 0.1 s ~ 12.5 cm (jeden pomiar,
   nie 2x wiecej). Do tylu 0.05 s dawalo od 0.4 cm do kilku cm. Przy
   jezdzie robot lekko znosi w bok (zdarzylo sie 1.8 cm na jednym
   kroku). Kalibrowac pomiarem (suwmiarka/kamera), nie zakladac
   liniowosci.

9. **Ramie niewidoczne jako port COM** = kabel USB kontrolera
   niepodlaczony albo brak zasilania serw. Porty Bluetooth COM3/5/7/8 to
   NIE ramie - nie mylic przy wyborze portu.

10. **"There is no status packet!" na magistrali Feetech przy szybkich
    petlach ruchu.** Przyczyna: `max_relative_target` w configu sprawia,
    ze KAZDY `send_action` robi dodatkowy `sync_read` Present_Position -
    przy komendach co ~0.25 s zapycha to bus. `sync_write` nie czeka na
    odpowiedz, wiec blad wychodzi dopiero pozniej (np. w `disconnect`),
    co myli przy diagnozie. Rozwiazanie w petlach: wlasne ograniczenie
    kroku w Pythonie + `arm.config.max_relative_target = None` na czas
    petli (tak robi `dance()`/`gong()`/`replay_csv.py`). Interpolowane
    ruchy (`move_to` z wieloma krokami) tez zapychaja magistrale przy
    duzych katach - bezposrednie `send_action` z
    `max_relative_target=None` + powtorki sa niezawodne.

11. **Gwaltowne losowe ruchy = ochrona przeciazeniowa STS3215** -
    serwa przestaja odpowiadac. Pomaga tylko fizyczne odlaczenie i
    ponowne podlaczenie USB/zasilania kontrolera. Duze zakresy ruchu -
    tylko z malym krokiem na tick.

12. **Zero stopni w lerobot to NIE zero w URDF (prawdopodobnie).**
    lerobot liczy `deg = (raw - (range_min+range_max)/2) * 360/4095`,
    czyli 0 st to SRODEK zakresu nagranego przy kalibracji serw, a nie
    pozycja zerowa URDF. Przed FK/IK trzeba zmierzyc offset i znak
    kazdego przegubu. Po `arm_control.py calibrate` katy sie zmieniaja.

13. **Kazde `connect()` na chwile wylacza torque** (`configure()`) -
    ramie moze opasc pod grawitacja miedzy wywolaniami. Rob cala
    sekwencje ruchow w jednym polaczeniu, nie rozlaczaj i laczaj w
    petli.

14. **Przed zmiana rejestrow serwa** (np. `Homing_Offset`) wywolaj
    `bus.disable_torque()` - zdejmuje tez `Lock`, inaczej zapis do
    EEPROM nie wejdzie.

15. **`record_demo.py` najpierw wysyla ramie do home.** Przy zlej
    kalibracji home nie dojezdza tam gdzie trzeba i nagranie startuje z
    innej pozy niz zamierzona - jesli poczatek nagrania wyglada na
    "opadanie" ramienia, odtwarzaj replay od pozniejszego znacznika
    czasu, nie od 0 s.

16. **Punkt zamkniecia szczek chwytaka jest ok. 17 cm PRZED kamera**
    (zmierzone suwmiarka). D415 widzi glebie dopiero od ok. 0.27-0.31 m
    (patrz pulapka 17), wiec w chwili faktycznego chwytu szyszka jest
    ZAWSZE w martwej strefie glebi - ostatni odcinek podjazdu jedzie sie
    "na slepo", z zapamietanej wczesniej pozycji. Nie da sie tego
    zobaczyc "na zywo" w momencie chwytu: przy pozie chwytu przedramie
    ramienia zaslania kamere (jest ok. 0.31 m przed obiektywem, czyli
    tez w martwej strefie). Konsekwencja projektowa: schemat
    postoj-skan-zapamietaj-jedz, nie ciagle sledzenie celu.

17. **Martwa strefa glebi D415 - ZMIERZONA: 0.27 m przy 640x480**
    (poziomo, kamera 10 cm nad podlozem). Ziemia jest mierzona od ok.
    0.27 m (dolne wiersze kadru) do ok. 1.1 m. Wczesniejszy szacunek "ok.
    30 cm, niezmierzone" - uzywaj tej zmierzonej wartosci. Przy 1280x720
    strefa jest wieksza (szacunek pamieciowy, niezmierzony dokladnie:
    ok. 45 cm).

18. **Niskie pokrycie glebia przy ziemi to GEOMETRIA (martwa strefa),
    NIE ciemnosc/zmierzch.** Pomiar wiersz po wierszu: 78.7% pokrycia
    przy medianie 0.39 m, 74.6% przy 0.33 m, 47.2% przy 0.27 m, tylko
    2.4% nizej. Rozstrzygajace: `gain` sensora glebi stoi na 16 z
    zakresu 16-248 (auto-ekspozycja juz na minimum wzmocnienia - sensor
    ma NADMIAR swiatla, nie niedobor). Nie trzeba czekac na inna pore
    dnia. Podbicie mocy projektora IR pomaga trocha (np. 31.0% ->
    35.8%), ale to nie jest glowny czynnik.

19. **Lewa piata kadru D415 (kolumny 0-64 px) prawie nie ma glebi**
    (pokrycie ~9.8% vs ~25-34% w srodku, ~3.8% w kolumnach 64-128 px).
    Przyczyna: prawy obiektyw stereo nie widzi tego, co lewy lapie przy
    swojej lewej krawedzi - stereo nie ma czego dopasowac. Podbicie
    lasera tego NIE naprawia (9.8% -> 4.4%, jeszcze gorzej) - brakuje
    drugiego punktu widzenia, nie swiatla. Nie celowac w obiekt lewa
    krawedzia kadru.

20. **Filtry glebi (spatial + temporal) poprawiaja pokrycie i
    POGARSZAJA detekcje malych obiektow.** Zmierzone parami na tych
    samych klatkach: filtry daja 22.30% -> 24.44% pokrycia w pasie
    0.3-1.0 m (pewna poprawa), ale na tej samej nieruchomej scenie ze
    szyszkami spada liczba wykrytych obiektow (5/5 bez filtrow w
    4 przebiegach na 4, 4/5 z filtrami w 2 na 4) - i zawsze gubiona byla
    NAJBLIZSZA szyszka. Wygladzanie zjada male obiekty. `hole_filling_filter`
    odpada z zalozenia - zmysla glebie z sasiadow zamiast uzupelniac ja
    pomiarami, a detekcja "obiekt czy nie" decyduje dokladnie na tych
    danych.

21. **Otwarcie morfologiczne jadrem 5x5 zjada male obiekty.** Ta sama
    szyszka mierzy 2.2 cm wysokosci przy progu 0.6 cm i tylko 1.0 cm
    przy progu 0.8 cm - nie dlatego, ze zmierzono ja inaczej, tylko
    dlatego, ze z cienszego paska erozja zostawia sam dol. Przy celach
    20-50 px jadro 3x3 jest bezpieczniejsze. Objaw skrajny: klaster w
    ogole nie powstaje, wiec liczniki odrzucen pokazuja same zera i
    "nie ma czego tlumaczyc".

22. **Piksele bez glebi deprojektuja sie do (0,0,0)**, czyli do
    pozornej odleglosci rownej offsetowi plaszczyzny. Domkniecie
    morfologiczne zalepia nimi dziury w masce obiektu. Geometrie
    (centroid, wysokosc, szerokosc) licz WYLACZNIE z pikseli faktycznie
    nad plaszczyzna ziemi, inaczej wysokosci wychodza absurdalne (np.
    61 cm przy limicie 40 cm), a szerokosc chwytu skacze miedzy 14 a
    44 cm na tej samej, nieruchomej scenie.

23. **Offset dopasowanej plaszczyzny ziemi (RANSAC) to zmierzona
    wysokosc kamery** - porownanie go z wysokoscia montazu to darmowy
    test, czy RANSAC zlapal faktycznie ZIEMIE, a nie sciane albo blat.
    Wylapalo dwa razy w jednej sesji przestawiona kamere. Bez tej
    kontroli wszystkie dystanse licza sie od zlej plaszczyzny i mimo to
    wygladaja wiarygodnie.

24. **W kadrze na tym torze nie ma zadnego "brazu"** - detekcja szyszek
    po kolorze (HSV) sie nie nadaje. Zmierzone HSV (OpenCV, zakres
    0-179) o zmierzchu: murawa H~90 (cyjan), szyszki H~148 (fiolet), a
    ogony rozkladow zachodza na siebie. Rozdziela je JASNOSC: szyszki
    V~94, murawa V~152. Prog na jasnosc dziala, ale jest kruchy - slonce
    albo cien go przesuwa. Przed strojeniem progow HSV zamroz balans
    bieli i ekspozycje (`detect_object.py --white-balance --exposure`),
    inaczej progi dobrane dzis nie trafiaja jutro. Domyslna detekcja
    idzie po GLEBI (dowolny ksztalt), kolor jest tylko opcja.

25. **`opencv-python` + `opencv-python-headless` zainstalowane razem
    psuja GUI.** `cv2.imshow` pada ("The function is not implemented.
    Rebuild the library with Windows, GTK+ 2.x or Cocoa support"), bo
    headless nadpisuje binaria GUI w tym samym namespace `cv2`. Fix:
    odinstalowac oba i postawic od nowa tylko `opencv-python` (na
    maszynie z ekranem). `lerobot` ciagnie `opencv-python-headless` jako
    zaleznosc - ten konflikt wraca po kazdym `pip install lerobot`,
    sprawdzac za kazdym razem. Na Pi (headless, bez ekranu) jest
    odwrotnie: trzymac TYLKO `opencv-python-headless`
    (`deploy/setup_pi.sh` odinstalowuje `opencv-python` po instalacji
    `requirements-pi.txt`).

26. **D415 znika z `rs.context().query_devices()` po nieczystym
    zamknieciu procesu** (crash przed `pipeline.stop()`, albo proces
    zabity przez `taskkill`/`kill -9`) - Windows/librealsense zostawia
    uchwyt USB w zlym stanie. Objaw: `RuntimeError: No device connected`
    albo `HResult 0x800703e3` przy `pipeline.start()`, mimo ze kamera
    fizycznie podlaczona. Fix: sprawdzic zawisle procesy Pythona
    (`tasklist | grep python` / `ps aux | grep python`) i dobic je -
    zombie trzymaja uchwyt USB nawet gdy proces "juz wyszedl" wedlug
    wrappera. Fizyczny replug USB jako ostatecznosc. Zawsze zamykaj
    stream przez `finally: pipeline.stop()`, nigdy golym Ctrl+C/kill -9.

27. **Laptop z portami USB 2.0 Type-A cofa D415 do trybu USB 2.1** i
    obcina tryby (848x480 dziala tylko na 10/6 Hz, koloru w tej
    rozdzielczosci nie ma wcale). To nie jest wina kabla - sprawdzone,
    ze porty Type-A sa fizycznie USB 2.0, USB 3 jest tylko na Type-C.
    640x480@30 dziala i daje realnie 18-22 fps, co w zupelnosci
    wystarcza do skanu na postoju.

28. **Jetson Nano P3450 odrzucony jako platforma.** Max JetPack 4.6 =
    Ubuntu 18.04 + Python 3.6, a lerobot wymaga Pythona 3.10+. Dlatego
    wybrane Raspberry Pi 5 (8 GB).

29. **Pinowanie lerobot / numpy** (`constraints.txt`, `numpy==2.5.3`):
    `lerobot` na Python 3.14 (laptop Windows) probuje przebudowac numpy
    ze zrodla i pada - brak gotowego kola (wheel) dla tej wersji Pythona
    oraz za stary kompilator GCC w systemie do budowy ze zrodel.
    Instalowac `lerobot` z `--no-deps` i doinstalowywac brakujace
    zaleznosci recznie (patrz docs/SETUP.md), zawsze z
    `-c constraints.txt --only-binary=:all:` gdzie sie da. Na Pi (Python
    3.12, `requirements-pi.txt`) tego problemu nie ma - NIE uzywac tam
    `constraints.txt`, jest tylko na Windows/Python 3.14.

30. **Siec `hacker-bloc` (demo) miala tylko IPv6, GitHub tylko IPv4** -
    przez to nie dzialal ani `git pull` na Pi, ani ICS z laptopa dla Pi.
    Hotspot z telefonu dawal IPv4 i dzialal. Zobacz tez docs/SETUP.md
    (sekcja Pi) po biezacy sposob laczenia z Pi.

31. **`pkill -f nazwa` przez SSH potrafi zabic sam siebie**, gdy szukana
    nazwa wystepuje tez w tresci samej komendy `pkill` (proces konczy
    sie z kodem 255, myli przy diagnozie). Zamiast tego: `fuser -k
    port/tcp` albo zabij po konkretnym PID.

32. **Nie rob ciezkiego re-renderu calego DOM przy kazdym ticku
    WebSocketa** (bylo co 30 ms) - realne klikniecia myszka/palcem w
    liscie/przyciskach zaczynaja sie gubic, mimo ze zautomatyzowany
    (precyzyjny) klik dalej dziala bez zarzutu. Rob diff/porownanie
    stanu i przebudowuj DOM tylko wtedy, gdy dane faktycznie sie
    zmienily (dotyczy panelu `web_control.py` / `frontend.html`).

33. **Trzymaj pola trybu (np. `state.playback_name`) w spojnosci przy
    KAZDEJ zmianie trybu** (`set_mode`), nie tylko przy naturalnym
    zakonczeniu odtwarzania - inaczej UI panelu pokazuje "duchy"
    poprzedniego stanu.

34. **`pinecone_config.json` jest w gicie, a `deploy/push_to_pi.sh` go
    NADPISUJE na Pi przy kazdym pushu.** Wartosc zmierzona i wpisana
    recznie na sprzecie (np. nowy prog HSV) przetrwa tylko do
    nastepnego pushu, jesli nie trafi do repo (commit/PR) - inaczej push
    przywraca stara wersje z brancha/mastera i pomiar przepada. Tak
    stracono raz wpisany na Pi prog HSV i sekcje "camera". Wartosci
    zmierzone na sprzecie wpisywac do configu W REPO, nie tylko lokalnie
    na Pi.

35. **Sesja tmux odpalona z NIEINTERAKTYWNEGO polaczenia ssh na Pi
    ginie po rozlaczeniu ssh** - nawet z `nohup`/`setsid` przezyla tylko
    jedno rozlaczenie, przy drugim zniknela. Interaktywne narzedzia
    ramienia (np. `tools/record_motion.py`, `tools/arm_play.py`)
    odpalac we WLASNYM terminalu operatora, interaktywnie:
    `ssh -t robot@<ip> "cd ~/hackaton && .venv/bin/python tools/..."`,
    nie z automatycznej (nieinteraktywnej) sesji.

36. **phyphox na iPhonie slucha na porcie 80, nie 8080** (8080 to Android;
    serwer `GCDWebServer`). iPhone-hotspot ma adres 172.20.10.1, wiec
    `heading.phyphox_url` = `http://172.20.10.1`. Serwer znika, gdy phyphox
    pojdzie w tlo albo zgasnie ekran (`Connection refused`) - Blokada
    automatyczna ekranu: Nigdy, phyphox na wierzchu. Pomiar startuje sam
    (`/control?cmd=start`). Test: `python tools/phyphox_check.py`.

37. **Skret hovera: martwa strefa zalezy od tego, czy robot stal.** Zmierzone
    zyroskopem telefonu (`tools/calibrate_turn.py`): z miejsca rusza dopiero od
    |b| ~160, a juz krecacy sie kreci od ~100; 10 jednostek wyzej to +0.3..0.7
    rad/s, przy tym samym b 160 raz 0.43, raz 1.08 rad/s. Stala tabela
    `xiao_steer_min/max` tego nie opisze - obrot po zyroskopie
    (`pinecone_bot/turn_loop.py`). `xiao_steer_min` 60 z configu nie rusza
    robota wcale.

38. **Przed pomiarem kamera sprawdz, gdzie patrzy.** Kamera siedzi na ramieniu;
    raz patrzyla w sufit i `calibrate_drive.py` "zmierzyl" 2 cm, choc robot
    przejechal 40 cm. Narzedzie zapisuje teraz `frames/calibrate_drive.jpg` i
    ostrzega, gdy glebia sie nie zmienia.

39. **`./arm.sh move` po ruchu wylacza moment w serwach** (lerobot
    `disconnect`), ramie opada. Do trzymania pozycji (np. kamera poziomo) -
    `tools/arm_web.py --no-home` i jog przez panel albo `POST /api/cmd`
    `{"cmd":"jog","joint":"wrist_flex","step":5}`.
