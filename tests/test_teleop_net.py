"""Testy tools/teleop_net.py - protokol, limit kroku, timeout, kolejnosc pakietow (bez sprzetu i bez lerobot)."""
from __future__ import annotations

import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import teleop_net as tn  # noqa: E402
from teleop_net import KEYS, TeleopServer, decode, encode_bye, encode_target, step_toward  # noqa: E402


def pose(v=0.0, **over):
    p = {k: v for k in KEYS}
    p.update({f"{j}.pos": x for j, x in over.items()})
    return p


def msg(session="a", seq=0, action=None, fake=False):
    return decode(encode_target(session, seq, action or pose(), 0.0, fake=fake))


def test_roundtrip_and_garbage():
    m = msg(seq=7, action=pose(1.5, gripper=40))
    assert m["n"] == 7 and m["a"]["gripper.pos"] == 40 and m["a"]["elbow_flex.pos"] == 1.5
    assert decode(b"\xff\x00") is None
    assert decode(b'{"s": "a", "n": 1, "a": {"gripper.pos": 1}}') is None  # brak stawow
    assert decode(b'{"s": "a", "n": 1, "a": ' + b'{' + b", ".join(
        f'"{k}": NaN'.encode() for k in KEYS) + b"}}") is None
    assert decode(encode_bye("a"))["bye"] is True


def test_step_toward_limits_each_joint():
    out = step_toward(pose(0), pose(0, shoulder_pan=20, elbow_flex=-3, gripper=-30), 5.0)
    assert out["shoulder_pan.pos"] == 5.0
    assert out["elbow_flex.pos"] == -3.0
    assert out["gripper.pos"] == -5.0


def test_holds_until_first_packet_and_ramps_to_target():
    s = TeleopServer(pose(0), max_step=5.0, timeout=0.5)
    assert s.tick(0.0) is None and s.state == "wait"
    assert s.handle(msg(action=pose(0, shoulder_lift=12)), 0.0)
    seen = [s.tick(t)["shoulder_lift.pos"] for t in (0.01, 0.02, 0.03, 0.04)]
    assert seen == [5.0, 10.0, 12.0, 12.0]
    assert s.state == "active"


def test_stale_link_holds_then_resumes_from_last_command():
    s = TeleopServer(pose(0), max_step=5.0, timeout=0.5)
    s.handle(msg(seq=0, action=pose(0, wrist_flex=4)), 0.0)
    assert s.tick(0.1)["wrist_flex.pos"] == 4.0
    assert s.tick(0.7) is None and s.state == "hold"
    s.handle(msg(seq=1, action=pose(0, wrist_flex=40)), 2.0)
    assert s.tick(2.0)["wrist_flex.pos"] == 9.0  # z ostatniej komendy, nie skokiem


def test_old_and_duplicate_packets_dropped_new_session_accepted():
    s = TeleopServer(pose(0))
    assert s.handle(msg(seq=5, action=pose(1)), 0.0)
    assert not s.handle(msg(seq=5, action=pose(2)), 0.0)
    assert not s.handle(msg(seq=3, action=pose(3)), 0.0)
    assert s.target["gripper.pos"] == 1
    assert s.handle(msg(session="b", seq=0, action=pose(4)), 0.0)  # restart klienta
    assert s.target["gripper.pos"] == 4


def test_bye_holds_immediately_only_for_own_session():
    s = TeleopServer(pose(0))
    s.handle(msg(session="a", seq=0), 0.0)
    assert not s.handle(decode(encode_bye("x")), 0.0)
    assert s.tick(0.01) is not None
    assert s.handle(decode(encode_bye("a")), 0.02)
    assert s.tick(0.03) is None and s.state == "hold"


def test_fake_packets_rejected_unless_dry_run():
    assert not TeleopServer(pose(0)).handle(msg(fake=True), 0.0)
    assert TeleopServer(pose(0), accept_fake=True).handle(msg(fake=True), 0.0)


def test_client_and_dry_run_server_over_udp(monkeypatch):
    """Prawdziwe gniazda na localhost: klient --fake -> serwer --dry-run dostaje cele."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]

    sent = []

    class Recorder(tn.PrintFollower):
        def send_action(self, action):
            sent.append(action)
            if len(sent) >= 5:
                raise KeyboardInterrupt  # koniec serwera
            return action

    monkeypatch.setattr(tn, "PrintFollower", Recorder)
    result = {}
    srv = threading.Thread(target=lambda: result.setdefault(
        "rc", tn.main(["server", "--dry-run", "--port", str(port), "--hz", "100"])), daemon=True)
    srv.start()
    time.sleep(0.2)

    stop = threading.Event()
    real_sleep = time.sleep

    def client_sleep(dt):
        if stop.is_set() and threading.current_thread() is cli:
            raise KeyboardInterrupt
        real_sleep(dt)

    monkeypatch.setattr(tn.time, "sleep", client_sleep)
    cli = threading.Thread(target=lambda: tn.main(
        ["client", "--fake", "--host", "127.0.0.1", "--port", str(port), "--hz", "100"]), daemon=True)
    cli.start()
    srv.join(5)
    stop.set()
    cli.join(5)
    assert result.get("rc") == 0
    assert len(sent) >= 5
    assert all(abs(a["shoulder_pan.pos"]) <= 10.0 for a in sent)
