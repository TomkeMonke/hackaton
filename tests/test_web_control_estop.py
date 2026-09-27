"""Wylacznik z telefonu (/stop w web_control.py): zatrzask, numer STOP, blokada jazdy, API HTTP."""

import functools
import http.server
import json
import sys
import threading
import time
import types
import urllib.error
import urllib.request

import pytest

# web_control importuje websockets (nie ma go w CI ani w requirements-pinecone.txt); tu niepotrzebny.
sys.modules.setdefault("websockets", types.ModuleType("websockets"))

import web_control  # noqa: E402


@pytest.fixture
def robot(monkeypatch):
    st = web_control.RobotState()
    monkeypatch.setattr(web_control, "state", st)
    arm_calls = []
    monkeypatch.setattr(web_control, "arm_stop", lambda: arm_calls.append(time.time()))
    st.arm_calls = arm_calls
    return st


@pytest.fixture
def server(robot):
    handler = functools.partial(web_control.FrontendHandler, directory=str(web_control.STATIC_DIR))
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{httpd.server_address[1]}"
    httpd.shutdown()
    httpd.server_close()


def _post(url, payload=None, headers=None):
    body = json.dumps(payload or {}).encode("ascii")
    req = urllib.request.Request(url, data=body, method="POST",
                                 headers={"Content-Type": "application/json", **(headers or {})})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_estop_latches_and_numbers_each_press(robot):
    assert robot.estop() == 1
    assert robot.estop_latched
    assert robot.estop() == 2


def test_stale_release_does_not_undo_newer_stop(robot):
    first = robot.estop()
    second = robot.estop()
    assert robot.estop_release(first) is False
    assert robot.estop_latched
    assert robot.estop_release(second) is True
    assert not robot.estop_latched


def test_hard_stop_zeroes_drive_and_drops_auto_mode(robot):
    robot.mode = "coverage"
    robot.speed, robot.steer = 0.5, -0.3
    robot.keys["w"] = True
    robot.recording = True
    robot.hard_stop()
    assert (robot.mode, robot.speed, robot.steer) == ("manual", 0.0, 0.0)
    assert not any(robot.keys.values())
    assert not robot.recording


def test_sequence_does_not_start_while_latched(robot):
    robot.estop()
    steps = web_control.parse_steps([{"type": "wait", "seconds": 1}])
    assert robot.start_sequence("x", steps) is False
    assert robot.mode == "manual"
    assert "/stop" in robot.seq_error


def test_http_estop_latches_and_stops_arm(server, robot):
    code, data = _post(f"{server}/api/estop")
    assert code == 200 and data["latched"] and data["id"] == 1
    assert robot.estop_latched
    for _ in range(50):
        if robot.arm_calls:
            break
        time.sleep(0.01)
    assert robot.arm_calls


def test_http_release_needs_current_id(server, robot):
    _post(f"{server}/api/estop")
    _post(f"{server}/api/estop")
    code, data = _post(f"{server}/api/estop_release", {"id": 1})
    assert code == 409 and data["latched"]
    code, data = _post(f"{server}/api/estop_release", {"id": 2})
    assert code == 200 and not data["latched"]


def test_http_rejects_foreign_origin_and_plain_text(server, robot):
    code, _ = _post(f"{server}/api/estop_release", {"id": 0}, {"Origin": "http://evil.example"})
    assert code == 403
    req = urllib.request.Request(f"{server}/api/estop", data=b"{}", method="POST",
                                 headers={"Content-Type": "text/plain"})
    with pytest.raises(urllib.error.HTTPError) as exc:
        urllib.request.urlopen(req, timeout=5)
    assert exc.value.code == 415
    assert not robot.estop_latched


def test_http_status_and_stop_page(server, robot):
    with urllib.request.urlopen(f"{server}/api/estop", timeout=5) as resp:
        data = json.loads(resp.read())
    assert data["latched"] is False and data["mode"] == "manual"
    with urllib.request.urlopen(f"{server}/stop", timeout=5) as resp:
        page = resp.read().decode("ascii")
    assert "/api/estop" in page and 'id="stop"' in page
