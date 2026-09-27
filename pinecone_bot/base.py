"""
Sterowniki podwozia (base) - wspolny interfejs dla maszyny stanow.

Trzy implementacje wybierane przez cfg.base.driver:
  sim          - kinematyka roznicowa calkowana w advance(dt) (symulator)
  xiao         - Seeed Xiao z xiao_send_pwm.ino, linie ASCII "a<speed> b<steer>\\n"
  bipropellant - UART do plyty hoverboarda z bipropellant-hoverboard-firmware

Konwencja predkosci: v [m/s] do przodu, w [rad/s] dodatnie = skret W LEWO
(przeciwnie do wskazowek zegara, standard ROS). Sterowniki same przeliczaja
to na znak, ktorego oczekuje sprzet.

Wszystkie sterowniki sprzetowe maja watek w tle, ktory powtarza ostatnia
komende (watchdogi w firmware zatrzymuja kola po ~500 ms ciszy). stop()
zeruje predkosci ale trzyma lacze; close() zatrzymuje watek i zamyka port.
"""
from __future__ import annotations

import math
import struct
import threading
import time
from typing import Callable, Optional, Protocol

from .config import Config


class Base(Protocol):
    def set_speed(self, v_mps: float, w_radps: float) -> None: ...
    def stop(self) -> None: ...
    def odometry(self) -> tuple[float, float, float] | None: ...
    def close(self) -> None: ...


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _clamp_cmd(cfg: Config, v: float, w: float) -> tuple[float, float]:
    c = cfg.control
    return _clamp(float(v), c.v_min, c.v_max), _clamp(float(w), -c.w_max, c.w_max)


def _default_serial_factory(port: str, baud: int):
    import serial  # pyserial, importowane leniwie zeby testy/sim nie wymagaly go
    return serial.Serial(port, baud, timeout=0.2)


# ---------------------------------------------------------------------------
# SIM
# ---------------------------------------------------------------------------
class SimBase:
    """
    Podwozie symulowane. Pozycja: x, y [m], theta [rad] (0 = os X, rosnie w lewo).
    advance(dt) calkuje ruch o dt sekund. Jesli podano clock (funkcja zwracajaca
    sekundy), set_speed()/odometry() same dociagaja czas z zegara.
    """

    def __init__(self, cfg: Config, clock: Optional[Callable[[], float]] = None):
        self.cfg = cfg
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self._clock = clock
        self._last_t = clock() if clock is not None else None

    def _sync(self) -> None:
        if self._clock is None:
            return
        now = self._clock()
        dt = now - self._last_t
        self._last_t = now
        if dt > 0:
            self.advance(dt)

    def advance(self, dt: float) -> None:
        """Dokladne calkowanie luku (dla w=0 linia prosta)."""
        if dt <= 0:
            return
        v, w = self.v, self.w
        if abs(w) < 1e-9:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            return
        th0 = self.theta
        th1 = th0 + w * dt
        r = v / w
        self.x += r * (math.sin(th1) - math.sin(th0))
        self.y -= r * (math.cos(th1) - math.cos(th0))
        self.theta = th1

    def set_speed(self, v_mps: float, w_radps: float) -> None:
        self._sync()
        self.v, self.w = _clamp_cmd(self.cfg, v_mps, w_radps)

    def stop(self) -> None:
        self.set_speed(0.0, 0.0)

    def odometry(self) -> tuple[float, float, float] | None:
        self._sync()
        return (self.x, self.y, self.theta)

    def close(self) -> None:
        self.stop()


# ---------------------------------------------------------------------------
# XIAO (xiao_send_pwm.ino)
# ---------------------------------------------------------------------------
XIAO_SPEED_LIMIT = 500   # MAX_PWM w firmware
XIAO_STEER_LIMIT = 400   # MAX_STEER w firmware
XIAO_EPS = 1e-4          # ponizej tego |v| lub |w| wysylamy 0 (bez offsetu tarcia)


def xiao_map(value: float, full_scale: float, pwm_min: int, pwm_max: int, limit: int) -> int:
    """
    m/s (lub rad/s) -> PWM z offsetem tarcia statycznego:
    pwm = sign(value) * (pwm_min + |value|/full_scale * (pwm_max - pwm_min)).
    """
    if abs(value) < XIAO_EPS or full_scale <= 0:
        return 0
    frac = min(abs(value) / full_scale, 1.0)
    mag = pwm_min + frac * (pwm_max - pwm_min)
    pwm = int(round(math.copysign(mag, value)))
    return int(_clamp(pwm, -limit, limit))


class XiaoBase:
    """
    Protokol: "a<speed> b<steer>\\n", speed -500..500 (+ = przod),
    steer -400..400 (+ = W PRAWO wg drive_step.py). Nasze w dodatnie = w lewo,
    wiec steer = -w_mapped. Watek w tle powtarza komende co cfg.base.xiao_period_s.
    serial_factory(port, baud) pozwala wstrzyknac atrape z write()/close().
    """

    ZERO_LINE = b"a0 b0\n"

    def __init__(self, cfg: Config, serial_factory=None, settle_s: float = 0.3):
        self.cfg = cfg
        factory = serial_factory or _default_serial_factory
        self._ser = factory(cfg.base.port, cfg.base.baud)
        if settle_s > 0:
            time.sleep(settle_s)  # Xiao resetuje sie przy otwarciu portu USB-CDC
        self.v = 0.0
        self.w = 0.0
        self.speed_pwm = 0
        self.steer_pwm = 0
        self.last_error: Optional[BaseException] = None
        self._line = self.ZERO_LINE
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="xiao-tx", daemon=True)
        self._thread.start()

    # -- mapowanie ---------------------------------------------------------
    def pwm_for(self, v: float, w: float) -> tuple[int, int]:
        b, c = self.cfg.base, self.cfg.control
        speed = xiao_map(v, c.v_max, b.xiao_pwm_min, b.xiao_pwm_max, XIAO_SPEED_LIMIT)
        steer = -xiao_map(w, c.w_max, b.xiao_steer_min, b.xiao_steer_max, XIAO_STEER_LIMIT)
        return speed, steer

    @staticmethod
    def command_line(speed_pwm: int, steer_pwm: int) -> bytes:
        return ("a%d b%d\n" % (int(speed_pwm), int(steer_pwm))).encode("ascii")

    # -- interfejs Base ----------------------------------------------------
    def set_speed(self, v_mps: float, w_radps: float) -> None:
        self.v, self.w = _clamp_cmd(self.cfg, v_mps, w_radps)
        self.speed_pwm, self.steer_pwm = self.pwm_for(self.v, self.w)
        line = self.command_line(self.speed_pwm, self.steer_pwm)
        with self._lock:
            self._line = line
            self._write_locked(line)

    def set_raw(self, speed_pwm: int, steer_pwm: int) -> None:
        """PWM wprost, z pominieciem mapowania m/s -> PWM (petla obrotu na zyroskopie, turn_loop.py)."""
        speed_pwm = int(_clamp(speed_pwm, -XIAO_SPEED_LIMIT, XIAO_SPEED_LIMIT))
        steer_pwm = int(_clamp(steer_pwm, -XIAO_STEER_LIMIT, XIAO_STEER_LIMIT))
        self.speed_pwm, self.steer_pwm = speed_pwm, steer_pwm
        line = self.command_line(speed_pwm, steer_pwm)
        with self._lock:
            self._line = line
            self._write_locked(line)

    def stop(self) -> None:
        self.set_speed(0.0, 0.0)  # wysyla "a0 b0\n" od razu, watek dalej powtarza

    def odometry(self) -> tuple[float, float, float] | None:
        return None

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        with self._lock:
            self._line = self.ZERO_LINE
            self._write_locked(self.ZERO_LINE)
            self._write_locked(self.ZERO_LINE)
        try:
            self._ser.close()
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    # -- wnetrze -----------------------------------------------------------
    def _write_locked(self, line: bytes) -> None:
        try:
            self._ser.write(line)
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    def _loop(self) -> None:
        period = max(0.01, float(self.cfg.base.xiao_period_s))
        while not self._stop_evt.wait(period):
            with self._lock:
                self._write_locked(self._line)


# ---------------------------------------------------------------------------
# BIPROPELLANT (bipropellant-hoverboard-firmware, protokol binarny)
# ---------------------------------------------------------------------------
# Ramka: SOM(1) CI(1) LEN(1) DATA(LEN) CS(1); CS = (-(CI + LEN + sum(DATA))) & 0xFF
# czyli suma bajtow od CI do CS wlacznie == 0 (mod 256). SOM nie wchodzi do sumy
# (wiki "Protocol Defn"). UWAGA: nowszy master bipropellant-protocol ma inny
# uklad (SOM cmd CI len ... + COBS/R) - sprawdzic wersje submodulu protocol
# w firmware zespolu.
BIP_SOM_ACK = 0x02      # odbiorca musi odpowiedziec 'A'
BIP_SOM_NOACK = 0x04    # bez potwierdzenia, do komend okresowych
BIP_CMD_WRITE = ord("W")
BIP_CMD_READ = ord("R")
BIP_CMD_READ_RESPONSE = ord("r")
BIP_CMD_WRITE_RESPONSE = ord("w")
BIP_CMD_ACK = ord("A")
BIP_CMD_NACK = ord("N")

BIP_CODE_HALL = 0x02              # HALL_DATA_STRUCT[2], tylko odczyt
BIP_CODE_SPEED = 0x03             # SPEED_DATA, zapis wlacza silniki + tryb predkosci
BIP_CODE_ENABLE = 0x09            # enable motors
BIP_CODE_DISABLE_POWEROFF = 0x0A  # disable poweroff timer

# Payloady 0x09 / 0x0A. W firmware sa to zmienne typu uint8_t (enable,
# disablepoweroff); zapis kopiuje min(len_param, len_msg) bajtow, wiec int32 LE
# z wartoscia 1 daje 0x01 w pierwszym bajcie. Jesli protocol.c zespolu robi
# to inaczej, zmienic tylko te stale (np. na b"\x01"). DO SPRAWDZENIA.
BIP_ENABLE_PAYLOAD = struct.pack("<i", 1)
BIP_DISABLE_ENABLE_PAYLOAD = struct.pack("<i", 0)
BIP_DISABLE_POWEROFF_PAYLOAD = struct.pack("<i", 1)
BIP_DISABLE_MOTORS_ON_CLOSE = True

# SPEED_DATA: int32 wanted_speed_mm_per_sec[2], speed_max_power, speed_min_power,
# speed_minimum_speed, speed_diff_mm_per_sec[2], speed_power_demand[2]
BIP_SPEED_STRUCT = struct.Struct("<9i")
# HALL_DATA_STRUCT: HallPosn, HallSpeed, HallPosnMultiplier(float), HallPosn_lastread,
# HallPosn_mm, HallPosn_mm_lastread, HallSpeed_mm_per_s, HallTimeDiff(u32), HallSkipped(u32)
BIP_HALL_STRUCT = struct.Struct("<iifiiiiII")
BIP_HALL_MM_INDEX = 4  # pole HallPosn_mm w krotce z BIP_HALL_STRUCT

# Indeksy kol w tablicach [2]. DO WERYFIKACJI NA SPRZECIE: zakladamy [0]=lewe,
# [1]=prawe. Jesli robot skreca w zla strone, zamienic te dwie stale.
BIP_LEFT = 0
BIP_RIGHT = 1
# Znak przyrostu HallPosn_mm dla kazdego kola (hoverboardy czesto maja jedno
# kolo odwrocone). DO WERYFIKACJI: jazda do przodu ma zwiekszac oba.
BIP_HALL_SIGN = (1.0, 1.0)

# Wartosci awaryjne, gdy firmware nie odpowie na odczyt 0x03 - ZGADYWANE,
# przy pierwszej okazji odczytac prawdziwe z plyty (base.speed_limits).
BIP_DEFAULT_MAX_POWER = 600
BIP_DEFAULT_MIN_POWER = -600
BIP_DEFAULT_MINIMUM_SPEED = 30

BIP_MAX_FRAME = 3 + 255 + 1
BIP_RX_LIMIT = 4 * BIP_MAX_FRAME  # powyzej tego wyrzucamy bajty z bufora


def build_frame(som: int, ci: int, data: bytes) -> bytes:
    data = bytes(data)
    if len(data) > 255:
        raise ValueError("payload za dlugi: %d" % len(data))
    ci &= 0xFF
    cs = (-(ci + len(data) + sum(data))) & 0xFF
    return bytes([som & 0xFF, ci, len(data)]) + data + bytes([cs])


def parse_frames(buffer: bytes) -> tuple[list[tuple[int, int, bytes]], bytes]:
    """
    Wyciaga kompletne, poprawne ramki z bufora. Zwraca ([(som, ci, data), ...], reszta),
    gdzie reszta to niekompletny ogon (od ostatniego kandydata na SOM) do doklejenia
    przy nastepnym odczycie. Smieci przed SOM i ramki ze zla suma sa pomijane.
    """
    buf = bytes(buffer)
    frames: list[tuple[int, int, bytes]] = []
    i = 0
    n = len(buf)
    while i < n:
        som = buf[i]
        if som != BIP_SOM_ACK and som != BIP_SOM_NOACK:
            i += 1
            continue
        if i + 3 > n:
            break  # brak CI/LEN
        ci = buf[i + 1]
        length = buf[i + 2]
        end = i + 3 + length + 1
        if end > n:
            break  # ramka niekompletna, czekamy na reszte
        data = buf[i + 3:i + 3 + length]
        cs = buf[end - 1]
        if (ci + length + sum(data) + cs) & 0xFF == 0:
            frames.append((som, ci, data))
            i = end
        else:
            i += 1  # falszywy SOM albo uszkodzona ramka - szukamy dalej
    return frames, buf[i:]


class BipropellantBase:
    """
    Sterownik plyty hoverboarda. Watek w tle co cfg.base.bip_period_s wysyla
    SPEED_DATA (SOM 0x04), co cfg.base.bip_hall_every ramek pyta o HALL_DATA
    i parsuje odpowiedzi -> odometria roznicowa z HallPosn_mm obu kol.

    serial_factory(port, baud) -> obiekt z write(), read(n), close() i najlepiej
    in_waiting. autostart=False i connect_timeout_s=0 przydaja sie w testach:
    wtedy wolamy tick()/poll() recznie.
    """

    def __init__(self, cfg: Config, serial_factory=None, connect_timeout_s: float = 0.5,
                 autostart: bool = True):
        self.cfg = cfg
        factory = serial_factory or _default_serial_factory
        self._ser = factory(cfg.base.port, cfg.base.baud)
        self.wheel_base_m = float(cfg.base.wheel_base_m)
        self.v = 0.0
        self.w = 0.0
        self.last_error: Optional[BaseException] = None
        self._ci = 0
        self._lock = threading.Lock()
        self._rx = b""
        # pelna struktura SPEED_DATA; [2:5] podmieniamy odczytem z firmware
        self._speed_fields = [0, 0, BIP_DEFAULT_MAX_POWER, BIP_DEFAULT_MIN_POWER,
                              BIP_DEFAULT_MINIMUM_SPEED, 0, 0, 0, 0]
        self.speed_limits_source = "default"
        self.frames_sent = 0
        self.hall_responses = 0
        self.hall_raw: Optional[tuple[tuple, tuple]] = None
        self._last_mm: Optional[tuple[float, float]] = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._stop_evt = threading.Event()
        # osobna blokada na sam zapis do portu: stop()/close() z glownego watku i tick() z bip-io
        # nie moga przeplatac bajtow ramek (ramka stop musi dojsc w calosci)
        self._io_lock = threading.Lock()
        self._thread = threading.Thread(target=self._loop, name="bip-io", daemon=True)
        self._connect(connect_timeout_s)
        if autostart:
            self._thread.start()

    # -- wlasciwosci -------------------------------------------------------
    @property
    def speed_limits(self) -> tuple[int, int, int]:
        """(speed_max_power, speed_min_power, speed_minimum_speed) uzywane w zapisie."""
        with self._lock:
            return (self._speed_fields[2], self._speed_fields[3], self._speed_fields[4])

    def wheel_speeds_mm(self, v: float, w: float) -> tuple[int, int]:
        half = self.wheel_base_m / 2.0
        left = int(round((v - w * half) * 1000.0))
        right = int(round((v + w * half) * 1000.0))
        return left, right

    # -- interfejs Base ----------------------------------------------------
    def set_speed(self, v_mps: float, w_radps: float) -> None:
        v, w = _clamp_cmd(self.cfg, v_mps, w_radps)
        left, right = self.wheel_speeds_mm(v, w)
        with self._lock:
            self.v, self.w = v, w
            self._speed_fields[BIP_LEFT] = left
            self._speed_fields[BIP_RIGHT] = right

    def stop(self) -> None:
        self.set_speed(0.0, 0.0)
        self._send(BIP_SOM_NOACK, self.speed_payload())

    def odometry(self) -> tuple[float, float, float] | None:
        with self._lock:
            if self._last_mm is None:
                return None  # jeszcze zadnej odpowiedzi halla
            return (self._x, self._y, self._theta)

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        self.set_speed(0.0, 0.0)
        self._send(BIP_SOM_NOACK, self.speed_payload())
        self._send(BIP_SOM_NOACK, self.speed_payload())
        if BIP_DISABLE_MOTORS_ON_CLOSE:
            self._send(BIP_SOM_ACK, bytes([BIP_CMD_WRITE, BIP_CODE_ENABLE]) + BIP_DISABLE_ENABLE_PAYLOAD)
        try:
            self._ser.close()
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    # -- ramki -------------------------------------------------------------
    def speed_payload(self) -> bytes:
        with self._lock:
            return bytes([BIP_CMD_WRITE, BIP_CODE_SPEED]) + BIP_SPEED_STRUCT.pack(*self._speed_fields)

    def speed_frame(self) -> bytes:
        """Kompletna ramka zapisu SPEED_DATA (do testow i podgladu), nie zuzywa CI."""
        with self._lock:
            ci = (self._ci + 1) & 0xFF
        return build_frame(BIP_SOM_NOACK, ci, self.speed_payload())

    def _next_ci(self) -> int:
        with self._lock:
            self._ci = (self._ci + 1) & 0xFF
            return self._ci

    def _send(self, som: int, data: bytes) -> None:
        frame = build_frame(som, self._next_ci(), data)
        try:
            with self._io_lock:
                self._ser.write(frame)
            self.frames_sent += 1
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    def _send_ack(self, ci: int) -> None:
        try:
            with self._io_lock:
                self._ser.write(build_frame(BIP_SOM_NOACK, ci, bytes([BIP_CMD_ACK])))
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc

    # -- polaczenie --------------------------------------------------------
    def _connect(self, timeout_s: float) -> None:
        self._send(BIP_SOM_ACK, bytes([BIP_CMD_WRITE, BIP_CODE_DISABLE_POWEROFF]) + BIP_DISABLE_POWEROFF_PAYLOAD)
        self._send(BIP_SOM_ACK, bytes([BIP_CMD_WRITE, BIP_CODE_ENABLE]) + BIP_ENABLE_PAYLOAD)
        self._send(BIP_SOM_ACK, bytes([BIP_CMD_READ, BIP_CODE_SPEED]))
        deadline = time.monotonic() + max(0.0, timeout_s)
        while True:
            self.poll()
            if self.speed_limits_source == "firmware" or time.monotonic() >= deadline:
                break
            time.sleep(0.01)

    # -- odbior ------------------------------------------------------------
    def _read_available(self) -> bytes:
        try:
            waiting = getattr(self._ser, "in_waiting", None)
            if waiting is None:
                return self._ser.read(BIP_MAX_FRAME) or b""
            if waiting <= 0:
                return b""
            return self._ser.read(int(waiting)) or b""
        except Exception as exc:  # noqa: BLE001
            self.last_error = exc
            return b""

    def poll(self) -> int:
        """Jeden cykl odbioru: czyta co jest w porcie i obsluguje ramki. Zwraca ich liczbe."""
        return self.process_incoming(self._read_available())

    def process_incoming(self, chunk: bytes) -> int:
        self._rx += bytes(chunk)
        frames, rest = parse_frames(self._rx)
        if len(rest) > BIP_RX_LIMIT:
            rest = rest[-BIP_MAX_FRAME:]
        self._rx = rest
        for som, ci, data in frames:
            self.handle_frame(som, ci, data)
        return len(frames)

    def handle_frame(self, som: int, ci: int, data: bytes) -> None:
        if som == BIP_SOM_ACK:
            self._send_ack(ci)
        if len(data) < 2:
            return  # 'A', 'N' itp. - ignorujemy
        cmd, code, payload = data[0], data[1], data[2:]
        if cmd != BIP_CMD_READ_RESPONSE:
            return
        if code == BIP_CODE_SPEED and len(payload) >= BIP_SPEED_STRUCT.size:
            fields = BIP_SPEED_STRUCT.unpack_from(payload)
            with self._lock:
                self._speed_fields[2:5] = list(fields[2:5])
            self.speed_limits_source = "firmware"
        elif code == BIP_CODE_HALL and len(payload) >= 2 * BIP_HALL_STRUCT.size:
            h0 = BIP_HALL_STRUCT.unpack_from(payload, 0)
            h1 = BIP_HALL_STRUCT.unpack_from(payload, BIP_HALL_STRUCT.size)
            self._integrate_hall((h0, h1))

    def _integrate_hall(self, halls: tuple[tuple, tuple]) -> None:
        self.hall_raw = halls
        self.hall_responses += 1
        mm_l = float(halls[BIP_LEFT][BIP_HALL_MM_INDEX]) * BIP_HALL_SIGN[BIP_LEFT]
        mm_r = float(halls[BIP_RIGHT][BIP_HALL_MM_INDEX]) * BIP_HALL_SIGN[BIP_RIGHT]
        with self._lock:
            if self._last_mm is not None:
                d_l = (mm_l - self._last_mm[0]) / 1000.0
                d_r = (mm_r - self._last_mm[1]) / 1000.0
                d = (d_l + d_r) / 2.0
                dth = (d_r - d_l) / self.wheel_base_m if self.wheel_base_m > 0 else 0.0
                mid = self._theta + dth / 2.0
                self._x += d * math.cos(mid)
                self._y += d * math.sin(mid)
                self._theta += dth
            self._last_mm = (mm_l, mm_r)

    # -- petla -------------------------------------------------------------
    def tick(self) -> None:
        """Jeden okres: wyslij SPEED_DATA, co N-ty raz zapytaj o halla, odbierz."""
        self._send(BIP_SOM_NOACK, self.speed_payload())
        every = max(1, int(self.cfg.base.bip_hall_every))
        if self.frames_sent % every == 0:
            self._send(BIP_SOM_NOACK, bytes([BIP_CMD_READ, BIP_CODE_HALL]))
        self.poll()

    def _loop(self) -> None:
        period = max(0.01, float(self.cfg.base.bip_period_s))
        while not self._stop_evt.wait(period):
            self.tick()


# ---------------------------------------------------------------------------
def make_base(cfg: Config, **kw) -> Base:
    driver = str(cfg.base.driver).lower()
    if driver == "sim":
        return SimBase(cfg, **kw)
    if driver == "xiao":
        return XiaoBase(cfg, **kw)
    if driver == "bipropellant":
        return BipropellantBase(cfg, **kw)
    raise ValueError("nieznany cfg.base.driver: %r (sim | xiao | bipropellant)" % cfg.base.driver)
