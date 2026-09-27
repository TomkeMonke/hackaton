"""Teleoperacja przez siec: leader SO-101 na laptopie, follower na Pi, pozycje stawow po UDP.

Po co: Pi nie ma wolnego USB (leader siedzial w miejscu kabla Xiao). Leader wpinamy do laptopa,
laptop czyta go lerobot-em (ta sama normalizacja co lerobot-teleoperate/lerobot-record)
i wysyla cele stawow po WiFi. Pi zadaje je followerowi.

Na Pi (nic innego nie moze trzymac portu ramienia: pkill -f arm_web.py; pkill -f lerobot):
    .venv/bin/python tools/teleop_net.py server
    .venv/bin/python tools/teleop_net.py server --dry-run     # bez ramienia, tylko drukuje cele

Na laptopie (lerobot[feetech] 0.6.1 + plik kalibracji leadera skopiowany z Pi, patrz docs/SETUP.md):
    python tools/teleop_net.py client --host robot.local --leader-port COM9
    python tools/teleop_net.py client --host robot.local --fake    # bez leadera, test lacza;
                                                                    # serwer przyjmie to TYLKO z --dry-run

Bezpieczenstwo (serwer):
- krok kazdego stawu ograniczony do --max-step st na tick (wlasny limit w Pythonie, bez
  max_relative_target lerobot - ten doklada sync_read co tick i zapycha magistrale, HARDWARE.md pkt 10);
- brak pakietu przez --timeout s -> ramie TRZYMA ostatnia pozycje (nie wysylamy nowych celow);
  po powrocie lacza ramie dojezdza do leadera z tym samym limitem kroku;
- pakiety starsze niz ostatni (UDP miesza kolejnosc) sa odrzucane; nowa sesja klienta (restart) jest przyjmowana;
- Ctrl+C na kliencie wysyla "bye" -> serwer od razu trzyma;
- Ctrl+C na serwerze: torque ZOSTAJE (kamera na ramieniu nie opada), jak w act_pick.py.
Pierwsze uruchomienie z czlowiekiem trzymajacym wylacznik zasilania serw.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import socket
import sys
import time

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
KEYS = [f"{j}.pos" for j in JOINT_NAMES]  # klucze akcji lerobot (get_action / send_action)

DEFAULT_PORT = 5005
LOOP_HZ = 30.0
MAX_STEP_DEG = 5.0   # na tick; przy 30 Hz = 150 st/s, reka leadera zwykle wolniejsza
TIMEOUT_S = 0.5


# ---------------------------------------------------------------------------
# Protokol: jeden JSON na datagram
# ---------------------------------------------------------------------------

def encode_target(session: str, seq: int, action: dict, t: float, fake: bool = False) -> bytes:
    msg = {"s": session, "n": seq, "t": t, "a": {k: round(float(action[k]), 3) for k in KEYS}}
    if fake:
        msg["fake"] = True  # sztuczne pozycje (wokol 0), nie z leadera
    return json.dumps(msg).encode()


def encode_bye(session: str) -> bytes:
    return json.dumps({"s": session, "bye": True}).encode()


def decode(data: bytes):
    """Zwraca slownik pakietu albo None (smieci, zly format, brak stawu, NaN)."""
    try:
        msg = json.loads(data.decode())
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(msg, dict) or not isinstance(msg.get("s"), str):
        return None
    if msg.get("bye"):
        return msg
    a = msg.get("a")
    if not isinstance(a, dict) or not isinstance(msg.get("n"), int):
        return None
    try:
        vals = {k: float(a[k]) for k in KEYS}
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in vals.values()):
        return None
    msg["a"] = vals
    return msg


def step_toward(current: dict, target: dict, max_step: float) -> dict:
    """Kazdy staw przesuniety w strone celu o co najwyzej max_step."""
    out = {}
    for k in KEYS:
        d = target[k] - current[k]
        out[k] = current[k] + max(-max_step, min(max_step, d))
    return out


# ---------------------------------------------------------------------------
# Serwer (Pi): logika bez gniazda i bez lerobot, zeby dala sie testowac
# ---------------------------------------------------------------------------

class TeleopServer:
    """Przyjmuje pakiety, pilnuje swiezosci i kolejnosci, wylicza akcje dla followera na tick."""

    def __init__(self, start_pose: dict, max_step: float = MAX_STEP_DEG, timeout: float = TIMEOUT_S,
                 accept_fake: bool = False):
        self.accept_fake = accept_fake  # tylko --dry-run: pakiety z --fake nie ruszaja ramienia
        self.commanded = {k: float(start_pose[k]) for k in KEYS}  # ostatnio wyslany cel
        self.max_step = max_step
        self.timeout = timeout
        self.session = None
        self.seq = -1
        self.target = None
        self.last_rx = None      # czas (serwera) ostatniego przyjetego celu
        self.state = "wait"      # wait | active | hold

    def handle(self, msg: dict, now: float) -> bool:
        """True, gdy pakiet przyjety (cel albo bye)."""
        if msg.get("bye"):
            if msg["s"] == self.session:
                self.target = None
                self.last_rx = None
                return True
            return False
        if msg.get("fake") and not self.accept_fake:
            return False
        if msg["s"] != self.session:
            self.session = msg["s"]   # nowy klient albo restart klienta
            self.seq = -1
        if msg["n"] <= self.seq:
            return False              # stary albo zdublowany pakiet
        self.seq = msg["n"]
        self.target = msg["a"]
        self.last_rx = now
        return True

    def tick(self, now: float):
        """Akcja do send_action albo None (trzymaj pozycje)."""
        fresh = (self.target is not None and self.last_rx is not None
                 and now - self.last_rx <= self.timeout)
        if not fresh:
            self.state = "hold" if self.session is not None else "wait"
            return None
        self.state = "active"
        self.commanded = step_toward(self.commanded, self.target, self.max_step)
        return dict(self.commanded)


def make_follower(port: str, robot_id: str):
    # lerobot importowany leniwie: modul (i testy) dziala bez niego
    from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
    from lerobot.robots.so_follower.so_follower import SO101Follower
    return SO101Follower(SO101FollowerConfig(port=port, id=robot_id, max_relative_target=None,
                                             disable_torque_on_disconnect=False))


class PrintFollower:
    """--dry-run: zamiast ramienia drukuje cele (raz na sekunde)."""

    def __init__(self):
        self.last_print = 0.0

    def connect(self, calibrate=False):
        pass

    def get_observation(self):
        return {k: 0.0 for k in KEYS}

    def send_action(self, action):
        now = time.monotonic()
        if now - self.last_print >= 1.0:
            self.last_print = now
            print("  cel: " + " ".join(f"{k[:-4]}={action[k]:.1f}" for k in KEYS))
        return action

    def disconnect(self):
        pass


def run_server(args) -> int:
    follower = PrintFollower() if args.dry_run else make_follower(args.follower_port, args.follower_id)
    # calibrate=False: nigdy nie wolno ruszac kalibracji followera (CLAUDE.md, HARDWARE.md)
    follower.connect(calibrate=False)
    try:
        obs = follower.get_observation()
        server = TeleopServer({k: obs[k] for k in KEYS}, max_step=args.max_step, timeout=args.timeout,
                              accept_fake=args.dry_run)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", args.port))
        sock.setblocking(False)
        print(f"Serwer teleop UDP :{args.port}, {args.hz:g} Hz, krok <= {args.max_step:g} st/tick, "
              f"timeout {args.timeout:g} s. Ctrl+C konczy (torque zostaje).")
        period = 1.0 / args.hz
        prev_state = None
        errors = 0
        rejected = set()  # sesje --fake odrzucone (komunikat raz)
        while True:
            t0 = time.monotonic()
            while True:
                try:
                    data, addr = sock.recvfrom(4096)
                except (BlockingIOError, InterruptedError):
                    break
                except ConnectionResetError:  # Windows: ICMP po wyslaniu ack do zamknietego klienta
                    continue
                msg = decode(data)
                if msg is None:
                    continue
                new_session = msg["s"] != server.session
                if msg.get("fake") and not server.accept_fake and msg["s"] not in rejected:
                    rejected.add(msg["s"])
                    print(f"Odrzucam klienta {addr[0]} z --fake (dziala tylko z serwerem --dry-run)")
                if server.handle(msg, t0):
                    if new_session and not msg.get("bye"):
                        print(f"Klient {addr[0]} (sesja {msg['s']})")
                    if "n" in msg:
                        try:
                            sock.sendto(json.dumps({"ack": msg["n"], "t": msg.get("t")}).encode(), addr)
                        except OSError:
                            pass
            action = server.tick(t0)
            if server.state != prev_state:
                print({"wait": "Czekam na klienta...",
                       "active": "AKTYWNE: follower jedzie za leaderem",
                       "hold": "TRZYMAM pozycje (brak swiezych pakietow)"}[server.state])
                prev_state = server.state
            if action is not None:
                try:
                    follower.send_action(action)
                    errors = 0
                except Exception as exc:  # pojedyncze bledy magistrali Feetech zdarzaja sie
                    errors += 1
                    print(f"(blad zapisu do ramienia: {exc})")
                    if errors >= 10:
                        print("10 bledow z rzedu - koniec. Sprawdz zasilanie serw i kabel.")
                        return 1
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("Koniec (torque zostaje, ramie trzyma pozycje).")
        return 0
    finally:
        follower.disconnect()


# ---------------------------------------------------------------------------
# Klient (laptop z leaderem)
# ---------------------------------------------------------------------------

def make_leader(port: str, leader_id: str):
    from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
    from lerobot.teleoperators.so_leader.so_leader import SOLeader
    leader = SOLeader(SOLeaderTeleopConfig(port=port, id=leader_id))
    if not leader.calibration:
        raise SystemExit(
            f"Brak kalibracji leadera '{leader_id}' na tym komputerze ({leader.calibration_fpath}).\n"
            "Skopiuj ja z Pi (NIE kalibruj od nowa - ACT byl nagrany z ta kalibracja):\n"
            "  scp robot@robot.local:~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/"
            f"{leader_id}.json \"{leader.calibration_fpath}\"")
    return leader


class FakeLeader:
    """--fake: wolna sinusoida barku (+-10 st wokol 0), reszta stawow stoi. Test lacza bez leadera."""

    def __init__(self):
        self.t0 = time.monotonic()

    def connect(self, calibrate=False):
        pass

    def get_action(self):
        a = {k: 0.0 for k in KEYS}
        a["shoulder_pan.pos"] = 10.0 * math.sin(0.5 * (time.monotonic() - self.t0))
        return a

    def disconnect(self):
        pass


def run_client(args) -> int:
    leader = FakeLeader() if args.fake else make_leader(args.leader_port, args.leader_id)
    addr = (socket.gethostbyname(args.host), args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    session = f"{random.getrandbits(32):08x}"
    leader.connect(calibrate=False)
    print(f"Klient -> {args.host} ({addr[0]}:{addr[1]}), sesja {session}. Ctrl+C konczy.")
    period = 1.0 / args.hz
    seq = 0
    rtts = []
    last_ack = time.monotonic()
    last_report = time.monotonic()
    try:
        while True:
            t0 = time.monotonic()
            try:
                action = leader.get_action()
                sock.sendto(encode_target(session, seq, action, t0, fake=args.fake), addr)
                seq += 1
            except OSError as exc:
                print(f"(blad wysylania: {exc})")
            except Exception as exc:
                print(f"(blad odczytu leadera: {exc})")
            while True:
                try:
                    data, _ = sock.recvfrom(4096)
                except (BlockingIOError, InterruptedError, ConnectionResetError):
                    break
                try:
                    ack = json.loads(data.decode())
                    rtts.append(time.monotonic() - float(ack["t"]))
                    last_ack = time.monotonic()
                except (ValueError, KeyError, TypeError):
                    pass
            if t0 - last_report >= 1.0:
                if rtts:
                    rtts.sort()
                    print(f"lacze: {len(rtts)} ack/s, RTT mediana {1000 * rtts[len(rtts) // 2]:.0f} ms, "
                          f"max {1000 * rtts[-1]:.0f} ms")
                else:
                    print(f"BRAK odpowiedzi serwera od {t0 - last_ack:.0f} s "
                          f"(serwer na Pi uruchomiony? ten sam adres/port?)")
                rtts = []
                last_report = t0
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):  # UDP moze zgubic; serwer i tak zatrzyma sie po timeout
            try:
                sock.sendto(encode_bye(session), addr)
            except OSError:
                pass
        leader.disconnect()
        print("Rozlaczono. Serwer trzyma pozycje.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Teleoperacja leader (laptop) -> follower (Pi) po UDP")
    sub = p.add_subparsers(dest="mode", required=True)

    s = sub.add_parser("server", help="na Pi: follower wykonuje cele z sieci")
    s.add_argument("--port", type=int, default=DEFAULT_PORT)
    s.add_argument("--follower-port", default=os.environ.get("ROBOT_ARM_PORT", "/dev/robot-arm"))
    s.add_argument("--follower-id", default=os.environ.get("ROBOT_ARM_ID", "so101"))
    s.add_argument("--hz", type=float, default=LOOP_HZ)
    s.add_argument("--max-step", type=float, default=MAX_STEP_DEG, help="st na tick, kazdy staw")
    s.add_argument("--timeout", type=float, default=TIMEOUT_S, help="s bez pakietu -> trzymaj")
    s.add_argument("--dry-run", action="store_true", help="bez ramienia, drukuje cele")

    c = sub.add_parser("client", help="na laptopie: czyta leadera i wysyla na Pi")
    c.add_argument("--host", default="robot.local")
    c.add_argument("--port", type=int, default=DEFAULT_PORT)
    c.add_argument("--leader-port", default=os.environ.get("ROBOT_ARM_LEADER_PORT", "COM9"))
    c.add_argument("--leader-id", default=os.environ.get("ROBOT_ARM_LEADER_ID", "so101_leader"))
    c.add_argument("--hz", type=float, default=LOOP_HZ)
    c.add_argument("--fake", action="store_true", help="bez leadera: sinusoida barku")

    args = p.parse_args(argv)
    return run_server(args) if args.mode == "server" else run_client(args)


if __name__ == "__main__":
    sys.exit(main())
