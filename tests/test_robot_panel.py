"""Panel zbiorczy: uslugi (start/stop/logi), stan z portu, paczka logow, dorobek, API."""
from __future__ import annotations

import io
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request
import zipfile
from http.server import ThreadingHTTPServer

from pinecone_bot.supervisor import (
    EventLog,
    Service,
    ServiceSpec,
    count_tests,
    dataset_episodes,
    health,
    logs_zip,
    parse_meminfo,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

CHILD = ("import sys, time\n"
         "print('hello from child', flush=True)\n"
         "print('to stderr', file=sys.stderr, flush=True)\n"
         "time.sleep(30)\n")


def wait_for(cond, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def svc(tmp_path, argv, port=None, probe=None, events=None):
    spec = ServiceSpec("child", "Dziecko", argv, port=port)
    kwargs = {"probe": probe} if probe else {}
    return Service(spec, str(tmp_path), str(tmp_path / "logs"), events or EventLog(), **kwargs)


def test_start_captures_output_and_stop(tmp_path):
    ev = EventLog()
    s = svc(tmp_path, [sys.executable, "-u", "-c", CHILD], events=ev)
    ok, _ = s.start()
    assert ok and s.running()
    assert s.status()["state"] == "running"
    assert wait_for(lambda: any("to stderr" in line for line in s.tail()))
    assert any("hello from child" in line for line in s.tail())
    assert not s.start()[0]  # drugi start odrzucony
    ok, _ = s.stop(timeout=2.0)
    assert ok and not s.running()
    assert wait_for(lambda: s.exit_code is not None)
    assert s.status()["state"] == "stopped"
    with open(s.log_path, encoding="utf-8") as fh:
        assert "hello from child" in fh.read()
    assert wait_for(lambda: [e["kind"] for e in ev.since(0)] == ["start", "stop"])


def test_crash_is_visible(tmp_path):
    ev = EventLog()
    s = svc(tmp_path, [sys.executable, "-c", "raise SystemExit(3)"], events=ev)
    s.start()
    assert wait_for(lambda: s.exit_code == 3)
    assert s.status()["state"] == "crashed"
    assert wait_for(lambda: ev.since(0)[-1]["kind"] == "error")


def test_external_service_is_not_started_twice(tmp_path):
    s = svc(tmp_path, [sys.executable, "-c", "pass"], port=1234, probe=lambda port: True)
    assert s.status()["state"] == "external"
    ok, msg = s.start()
    assert not ok and "poza panelem" in msg
    assert s.proc is None


def test_missing_program_does_not_raise(tmp_path):
    s = svc(tmp_path, [str(tmp_path / "nie_ma_takiego_programu")])
    ok, msg = s.start()
    assert not ok and "nie wystartowal" in msg


def test_events_since():
    ev = EventLog(wall=lambda: 100.0)
    ev.add("start", "a")
    ev.add("stop", "b")
    assert [e["text"] for e in ev.since(0)] == ["a", "b"]
    assert [e["text"] for e in ev.since(1)] == ["b"]
    assert ev.since(2) == []


def test_logs_zip_contains_logs_and_extras(tmp_path):
    s = svc(tmp_path, [sys.executable, "-c", "pass"])
    s._append("linia w pamieci")
    extra = tmp_path / "pinecone_log.csv"
    extra.write_text("t,state\n0,SEARCH\n")
    data = logs_zip([s], [str(extra), str(tmp_path / "brak.csv")], {"ok": 1})
    with zipfile.ZipFile(io.BytesIO(data)) as zf:
        names = set(zf.namelist())
        assert names == {"logs/child.log", "pinecone_log.csv", "status.json"}
        assert "linia w pamieci" in zf.read("logs/child.log").decode()


def test_health_and_meminfo():
    h = health(devices=())
    assert "cpu_temp_c" in h and "devices" in h
    m = parse_meminfo("MemTotal:  8000000 kB\nMemAvailable:  2000000 kB\n")
    assert m == {"mem_total_mb": 7812, "mem_used_pct": 75.0}
    assert parse_meminfo("") == {}


def test_facts_helpers(tmp_path):
    t = tmp_path / "tests"
    t.mkdir()
    (t / "test_a.py").write_text("def test_x():\n    pass\n\ndef helper():\n    pass\n\ndef test_y():\n    pass\n")
    assert count_tests(str(t)) == 2
    meta = tmp_path / "ds" / "grasp" / "meta"
    meta.mkdir(parents=True)
    (meta / "info.json").write_text(json.dumps({"total_episodes": 50}))
    assert dataset_episodes([str(tmp_path / "ds"), str(tmp_path / "brak")]) == {"grasp": 50}


def test_specs_demo_and_real():
    import robot_panel

    demo = {s.name: s for s in robot_panel.build_specs(demo=True, python="py")}
    assert "--fake" in demo["arm"].argv and "--no-home" in demo["arm"].argv
    assert demo["vision"].argv[-2:] == ["--source", "sim"]
    real = {s.name: s for s in robot_panel.build_specs(demo=False, arm_home=True, python="py")}
    assert "--fake" not in real["arm"].argv and "--no-home" not in real["arm"].argv
    assert "--source" not in real["vision"].argv
    assert real["drive"].port == 8765 and real["arm"].port == 8010 and real["vision"].port == 8020


def test_origin_check():
    import robot_panel

    assert robot_panel.origin_ok("", 8090)
    assert robot_panel.origin_ok("http://172.20.10.4:8090", 8090)
    assert not robot_panel.origin_ok("http://evil.example", 8090)
    assert not robot_panel.origin_ok("http://evil.example:8000", 8090)
    assert not robot_panel.origin_ok("http://x:8090/", 8090)


def test_http_api(tmp_path):
    import robot_panel

    ev = EventLog()
    s = svc(tmp_path, [sys.executable, "-u", "-c", CHILD], events=ev)
    hub = robot_panel.Hub([s], ev, repo=REPO_ROOT)
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), robot_panel.make_handler(hub, 8090))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"

    def post(path, origin=None):
        req = urllib.request.Request(base + path, method="POST", data=b"{}",
                                     headers={"Origin": origin} if origin else {})
        try:
            with urllib.request.urlopen(req) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)

    try:
        with urllib.request.urlopen(base + "/") as r:
            assert b"Makarena" in r.read()
        assert post("/api/svc/child/start", origin="http://evil.example:8000")[0] == 403
        assert post("/api/svc/nope/start")[0] == 404
        code, body = post("/api/svc/child/start", origin="http://robot.local:8090")
        assert code == 200 and body["ok"]
        with urllib.request.urlopen(base + "/api/status") as r:
            st = json.load(r)
        assert st["services"][0]["state"] == "running"
        assert st["events"][0]["kind"] == "start"
        assert wait_for(lambda: any("hello" in line for line in s.tail()))
        with urllib.request.urlopen(base + "/api/logs/child?n=50") as r:
            assert any("hello" in line for line in json.load(r)["lines"])
        with urllib.request.urlopen(base + "/logs.zip") as r:
            assert "attachment" in r.headers["Content-Disposition"]
            assert "logs/child.log" in zipfile.ZipFile(io.BytesIO(r.read())).namelist()
        code, body = post("/api/stop_all")
        assert code == 200 and not s.running()
        with urllib.request.urlopen(base + "/api/facts") as r:
            assert json.load(r)["tests"] > 0
    finally:
        httpd.shutdown()
        if s.running():
            s.stop()
