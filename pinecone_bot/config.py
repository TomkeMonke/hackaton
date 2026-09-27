"""
Konfiguracja robota. Wszystko, co trzeba zmierzyc lub nastroic na sprzecie, jest tutaj.

Plik JSON (domyslnie pinecone_config.json w katalogu repo) nadpisuje wartosci domyslne.
Wartosci domyslne sa dobrane pod symulator; na Pi trzeba je zmierzyc narzedziami z tools/.
"""
from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field


@dataclass
class HsvRange:
    """
    Prog HSV w skali OpenCV (H 0..179, S 0..255, V 0..255). Domyslnie 'brazowe'.
    lo[0] > hi[0] oznacza zakres H przechodzacy przez 180 (np. 140..15 = czerwien/braz
    z obu koncow skali) - detektor sumuje wtedy dwa przedzialy.
    """
    lo: tuple = (5, 60, 20)
    hi: tuple = (25, 255, 200)


@dataclass
class Grasp:
    """
    Jeden nagrany chwyt. target_row to wiersz obrazu (py), w ktorym lezala szyszka,
    gdy chwyt byl nagrywany. Zapisuje go tools/calibrate_target.py.
    forward_m jest potrzebne tylko symulatorowi (gdzie fizycznie jest ten punkt).
    """
    name: str
    target_row: float
    forward_m: float = 0.0


@dataclass
class ControlConfig:
    """Regulator P podjazdu. Dwie liczby do strojenia na sprzecie: kx i ky."""
    kx: float = 0.004      # rad/s na piksel bledu poziomego
    ky: float = 0.0025     # m/s na piksel bledu pionowego
    steer_sign: float = 1.0  # -1 jesli robot skreca w zla strone
    v_max: float = 0.25    # m/s do przodu
    v_min: float = -0.10   # m/s do tylu (ujemne = cofanie)
    w_max: float = 0.6     # rad/s
    deadband_px: float = 3.0
    tol_x_px: float = 8.0
    tol_y_px: float = 8.0
    settle_frames: int = 3     # ile klatek z rzedu w tolerancji, zeby uznac 'ustawiony'
    lost_timeout_s: float = 2.0    # tyle bez detekcji w APPROACH -> SEARCH
    lost_hold_s: float = 0.25      # po utracie dokoncz ostatni obrot tylko przez tyle sekund (inaczej przestrzal)
    far_row_frac: float = 0.35     # ostatnia detekcja powyzej tej czesci obrazu = 'daleko'
    far_creep_v: float = 0.10      # 'daleko' i zgubiona: pelznij do przodu, az wejdzie w kadr
    search_w: float = 0.35     # rad/s obrotu w SEARCH
    search_pattern: str = "lanes"  # lanes = pelny obrot, potem pasy jak kosiarka; spin_drive = obrot + kawalek prosto
    lane_length_m: float = 3.0     # dlugosc pasa (dlugosc trawnika w kierunku startu)
    lane_count: int = 4            # tyle pasow i koniec (DONE); szerokosc trawnika / lane_spacing_m
    lane_spacing_m: float = 1.0    # odstep miedzy pasami; kamera widzi ok. +-0.8 m na 1.5 m, wiec 1.0 m ma zapas
    search_drive_m: float = 0.8   # po pelnym obrocie bez detekcji przejedz tyle do przodu i obroc sie znowu
    search_drive_v: float = 0.15
    search_timeout_s: float = 90.0  # tyle sekund bez detekcji konczy prace (DONE)
    retries: int = 2
    retry_back_s: float = 1.2  # ile sekund cofac przed ponowna proba
    retry_back_v: float = -0.08
    loop_hz: float = 15.0
    track_max_jump_px: float = 120.0  # w APPROACH trzymaj sie celu; skok wiekszy = to inna szyszka


@dataclass
class DetectorConfig:
    hsv: HsvRange = field(default_factory=HsvRange)
    min_area_px: int = 60
    max_area_px: int = 40000
    blur_ksize: int = 5
    morph_ksize: int = 5
    border_px: int = 3             # blob dotykajacy gornej lub bocznej krawedzi = uciety obiekt, ignoruj
    grasp_zone_rows: float = 40.0  # +- tyle wierszy wokol target_row = 'szyszka wciaz w strefie chwytu'


@dataclass
class BaseConfig:
    driver: str = "sim"            # sim | xiao | bipropellant
    port: str = "/dev/robot-drive"
    baud: int = 115200
    wheel_base_m: float = 0.50     # rozstaw kol, do przeliczenia (v, w) na kola
    # xiao: mapowanie m/s -> PWM. Nieliniowe, zmierzyc tools/base_test.py.
    xiao_pwm_min: int = 80         # ponizej tego kola nie ruszaja (tarcie statyczne)
    xiao_pwm_max: int = 260        # PWM odpowiadajace v_max
    xiao_steer_min: int = 60
    xiao_steer_max: int = 220      # PWM skretu odpowiadajace w_max
    xiao_period_s: float = 0.08    # < 500 ms watchdog na Xiao
    # bipropellant
    bip_period_s: float = 0.05
    bip_hall_every: int = 4        # co ile ramek pytac o halla


@dataclass
class ArmConfig:
    driver: str = "sim"            # sim | waypoints | subprocess
    port: str = "/dev/robot-arm"
    arm_id: str = "so101"
    motions_dir: str = "motions"
    subprocess_cmd: str = "python legacy/arm_recordings/replay_demo.py --port {port}"
    empty_gripper_below: float = 6.0  # odczyt gripper.pos po zamknieciu ponizej tego = pusty chwytak
    # Jog XYZ w panelu (pinecone_bot/kinematics.py): kat URDF = znak * kat lerobot + offset.
    # Offsety wpisuje przycisk "ZERO URDF" w panelu (ramie wyprostowane poziomo do przodu).
    urdf_path: str = "so101_urdf/so101_new_calib.urdf"
    urdf_sign: dict = field(default_factory=dict)        # {joint: +1/-1}, brak = +1
    urdf_offset_deg: dict = field(default_factory=dict)  # {joint: st}, brak = 0


@dataclass
class CameraConfig:
    """Start kamery RealSense (kolor)."""
    # Klatki odrzucane po starcie (30 fps). Auto white balance D4xx ustala sie ~1 s;
    # przy 15 klatkach (0.5 s) trawa byla jeszcze zielona i prog HSV nie lapal szyszek.
    warmup_frames: int = 45
    # True: po rozgrzewce zamroz AWB i auto-ekspozycje na biezacych wartosciach,
    # zeby kolory nie plywaly. Minus: przy duzej zmianie swiatla obraz za ciemny/jasny.
    lock_auto: bool = False


@dataclass
class HeadingConfig:
    """
    Zrodlo kursu (kata obrotu robota) dla pasow w SEARCH. Bez niego pasy ida z czasu i predkosci zadanych.
    none:     brak, pasy z czasu (jak dotad)
    phyphox:  zyroskop telefonu przyklejonego plasko do bazy, aplikacja phyphox z "Allow remote access"
    odometry: kat z base.odometry() (bipropellant z hallotronow; w symulacji prawdziwy kat)
    """
    source: str = "none"
    # iPhone jako hotspot ma zawsze 172.20.10.1; port i nazwy buforow jak w eksperymencie
    # phyphox "Gyroscope (rotation rate)"
    phyphox_url: str = "http://172.20.10.1:8080"
    gyro_buffer: str = "gyrZ"
    time_buffer: str = "gyr_time"
    sign: float = 1.0          # -1, jesli obrot w lewo daje ujemny kurs (telefon ekranem w dol)
    poll_hz: float = 20.0
    stale_s: float = 1.0       # tyle bez nowych probek = kurs nieznany
    lost_s: float = 3.0        # tyle bez kursu w SEARCH -> dalej pasy z czasu
    kp: float = 1.5            # rad/s na radian bledu kursu
    w_min: float = 0.08        # rad/s; ponizej tego kola nie ruszaja, a obrot ma dojsc do celu
    tol_deg: float = 3.0       # obrot uznany za skonczony
    align_deg: float = 15.0    # na prostej: blad wiekszy -> najpierw obrot w miejscu


@dataclass
class SimConfig:
    """Geometria kamery i swiata dla symulatora (i do zgrubnej kalibracji na sucho)."""
    cam_height_m: float = 0.45
    cam_pitch_deg: float = 38.0
    cam_forward_m: float = -0.10   # kamera 10 cm ZA srodkiem osi kol (na maszcie z tylu)
    fx: float = 600.0
    fy: float = 600.0
    cone_radius_m: float = 0.03
    noise_px: float = 1.0
    field_m: float = 2.2
    n_cones: int = 5
    seed: int = 1
    # niedoskonalosci napedu (domyslnie idealny): obrot faktyczny = zadany * turn_gain (poslizg),
    # a przy jezdzie do przodu robot sam skreca z drift_w rad/s (rozne silniki przy tym samym PWM)
    turn_gain: float = 1.0
    drift_w: float = 0.0


@dataclass
class Config:
    image_w: int = 640
    image_h: int = 480
    cx: float = 320.0   # kolumna 'srodek chwytaka' w obrazie; zapisuje calibrate_target.py
    grasps: list = field(default_factory=lambda: [
        Grasp("grasp_near", 400.0, 0.28),
        Grasp("grasp_mid", 360.0, 0.33),
        Grasp("grasp_far", 325.0, 0.38),
    ])
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    base: BaseConfig = field(default_factory=BaseConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    heading: HeadingConfig = field(default_factory=HeadingConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    log_csv: str = "pinecone_log.csv"

    # --- IO ---------------------------------------------------------------
    @staticmethod
    def default_path() -> str:
        return os.environ.get("PINECONE_CONFIG", "pinecone_config.json")

    @classmethod
    def load(cls, path: str | None = None) -> "Config":
        path = path or cls.default_path()
        cfg = cls()
        if not os.path.exists(path):
            return cfg
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        return _merge(cfg, data)

    def save(self, path: str | None = None) -> str:
        path = path or self.default_path()
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(asdict(self), fh, indent=2, ensure_ascii=False)
            fh.write("\n")
        return path

    def grasp_for_row(self, py: float) -> Grasp:
        """Najblizszy nagrany chwyt dla wiersza obrazu, w ktorym lezy szyszka."""
        return min(self.grasps, key=lambda g: abs(g.target_row - py))

    def nearest_target_row(self, py: float) -> float:
        return self.grasp_for_row(py).target_row


def _merge(obj, data: dict):
    """Rekurencyjnie wpisz slownik z JSON w dataclass (nieznane klucze ignoruje)."""
    for key, value in data.items():
        if not hasattr(obj, key):
            continue
        current = getattr(obj, key)
        if key == "grasps":
            setattr(obj, key, [Grasp(**g) if isinstance(g, dict) else g for g in value])
        elif hasattr(current, "__dataclass_fields__") and isinstance(value, dict):
            _merge(current, value)
        elif isinstance(current, tuple) and isinstance(value, list):
            setattr(obj, key, tuple(value))
        else:
            setattr(obj, key, value)
    return obj
