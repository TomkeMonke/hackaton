"""
Panel zbiorczy robota: kamera z liczba szyszek, jazda, ramie, uslugi, logi, zdrowie Pi.
Jedno okno SSH zamiast trzech: panel sam uruchamia i pilnuje reszty.

Uzycie na Pi (z katalogu repo):
    python tools/robot_panel.py --autostart     # http://<IP_PI>:8090, startuje jazde, ramie i kamere
    python tools/robot_panel.py                 # uslugi startuje sie przyciskami w panelu
    python tools/robot_panel.py --demo --autostart   # laptop: ramie-atrapa, kamera z symulatora

    http://<IP>:8090/       panel operatora
    http://<IP>:8090/show   tryb pokazu (projektor): kamera, licznik, co zbudowalismy

Uslugi (osobne procesy, jak dotad; blad jednej nie zatrzymuje reszty):
    jazda   web_control.py          :8000 strona, :8765 WebSocket (port Xiao z ROBOT_DRIVE_PORT)
    ramie   tools/arm_web.py        :8010 (domyslnie --no-home: kamera siedzi na ramieniu)
    kamera  tools/vision_web.py     :8020 (RealSense; kamere moze trzymac tylko jeden proces)
Usluga odpalona recznie w SSH jest widoczna jako "poza panelem" i panel z niej korzysta.
Logi: logs/<usluga>.log, paczka ZIP pod /logs.zip (razem z pinecone_log.csv).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pinecone_bot.supervisor import (  # noqa: E402
    EventLog,
    Service,
    ServiceSpec,
    health,
    logs_zip,
    project_facts,
)

HTTP_PORT = 8090
MAX_BODY = 1024
LOG_DIR = os.path.join(REPO_ROOT, "logs")
STATIC = {
    "/": ("robot_panel.html", "text/html; charset=utf-8"),
    "/show": ("robot_panel.html", "text/html; charset=utf-8"),
    "/arm_panel.js": ("arm_panel.js", "text/javascript; charset=utf-8"),
}
EXTRA_LOGS = [os.path.join(REPO_ROOT, "pinecone_log.csv")]


def build_specs(demo: bool = False, arm_home: bool = False, vision_source: str | None = None,
                python: str = sys.executable) -> list[ServiceSpec]:
    linux = sys.platform.startswith("linux")
    drive_env = {}
    if "ROBOT_DRIVE_PORT" not in os.environ and linux:
        drive_env["ROBOT_DRIVE_PORT"] = "/dev/robot-drive"
    arm_argv = [python, "-u", "tools/arm_web.py"]
    if not arm_home:
        arm_argv.append("--no-home")
    if demo:
        arm_argv.append("--fake")
    elif "ROBOT_ARM_PORT" not in os.environ and linux:
        arm_argv += ["--port", "/dev/robot-arm"]
    vision_argv = [python, "-u", "tools/vision_web.py"]
    source = vision_source or ("sim" if demo else None)
    if source:
        vision_argv += ["--source", source]
    specs = [
        ServiceSpec("drive", "Jazda", [python, "-u", "web_control.py"], port=8765, link=8000, env=drive_env,
                    note="Xiao po USB, WASD, failsafe 1 s"),
        ServiceSpec("arm", "Rami\u0119", arm_argv, port=8010, link=8010,
                    note="SO-101" + (" (atrapa)" if demo else "") + ("" if arm_home else ", bez HOME")),
        ServiceSpec("vision", "Kamera", vision_argv, port=8020, link=8020,
                    note=f"detekcja HSV, zrodlo: {source or 'RealSense'}"),
    ]
    if os.path.exists(os.path.join(REPO_ROOT, "tools", "estop_server.py")):
        specs.append(ServiceSpec("estop", "E-stop", [python, "-u", "tools/estop_server.py"], port=8001,
                                 link=8001, note="STOP z telefonu"))
    return specs


def origin_ok(origin: str, port: int) -> bool:
    """POST tylko ze strony tego panelu (dowolny host, nasz port) albo bez Origin (curl)."""
    if not origin:
        return True
    return re.fullmatch(r"http://[^/]+:" + str(port), origin) is not None


class Hub:
    def __init__(self, services: list[Service], events: EventLog, repo: str = REPO_ROOT):
        self.services = {s.spec.name: s for s in services}
        self.events = events
        self.repo = repo
        self.started = time.time()
        self._facts: dict | None = None

    def facts(self) -> dict:
        if self._facts is None:
            try:
                self._facts = project_facts(self.repo)
            except OSError as exc:
                self._facts = {"error": str(exc)}
        return self._facts

    def status(self, since: int = 0) -> dict:
        return {
            "services": [s.status() for s in self.services.values()],
            "health": health(),
            "events": self.events.since(since),
            "hub_uptime_s": round(time.time() - self.started),
            "ts": round(time.time(), 3),
        }

    def action(self, name: str, act: str) -> tuple[int, dict]:
        svc = self.services.get(name)
        if svc is None or act not in ("start", "stop", "restart"):
            return 404, {"ok": False, "msg": "nie ma takiej uslugi albo akcji"}
        ok, msg = getattr(svc, act)()
        return (200 if ok else 409), {"ok": ok, "msg": msg}

    def stop_all(self) -> list[str]:
        msgs = []
        for s in self.services.values():
            if s.running():
                msgs.append(s.stop()[1])
        return msgs


def make_handler(hub: Hub, own_port: int = HTTP_PORT):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload) -> None:
            self._send(code, json.dumps(payload).encode("utf-8"), "application/json")

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            path = url.path
            if path in STATIC:
                name, ctype = STATIC[path]
                with open(os.path.join(hub.repo, name), "rb") as fh:
                    self._send(200, fh.read(), ctype)
            elif path == "/api/status":
                try:
                    since = int((q.get("since") or ["0"])[0])
                except ValueError:
                    since = 0
                self._json(200, hub.status(since))
            elif path == "/api/facts":
                self._json(200, hub.facts())
            elif m := re.fullmatch(r"/api/logs/(\w+)", path):
                svc = hub.services.get(m.group(1))
                if svc is None:
                    self._json(404, {"ok": False, "msg": "nie ma takiej uslugi"})
                    return
                try:
                    n = max(1, min(600, int((q.get("n") or ["200"])[0])))
                except ValueError:
                    n = 200
                self._json(200, {"name": svc.spec.name, "lines": svc.tail(n)})
            elif path == "/logs.zip":
                data = logs_zip(list(hub.services.values()), EXTRA_LOGS, hub.status())
                fname = time.strftime("robot_logs_%Y%m%d_%H%M%S.zip")
                self._send(200, data, "application/zip",
                           {"Content-Disposition": f'attachment; filename="{fname}"'})
            else:
                self._json(404, {"ok": False, "msg": "nie ma"})

        def do_POST(self):
            if not origin_ok(self.headers.get("Origin") or "", own_port):
                self._json(403, {"ok": False, "msg": "obcy origin"})
                return
            length = int(self.headers.get("Content-Length") or 0)
            if length > MAX_BODY:
                self._json(400, {"ok": False, "msg": "za dlugo"})
                return
            if length:
                self.rfile.read(length)
            path = urlparse(self.path).path
            if m := re.fullmatch(r"/api/svc/(\w+)/(\w+)", path):
                code, payload = hub.action(m.group(1), m.group(2))
                self._json(code, payload)
            elif path == "/api/stop_all":
                msgs = hub.stop_all()
                hub.events.add("stop", "panel: zatrzymano wszystkie uslugi")
                self._json(200, {"ok": True, "msg": "; ".join(msgs) or "nic nie chodzilo z panelu"})
            else:
                self._json(404, {"ok": False, "msg": "nie ma"})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "0.0.0.0"))
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    parser.add_argument("--autostart", action="store_true", help="od razu uruchom wszystkie uslugi")
    parser.add_argument("--demo", action="store_true", help="bez sprzetu: ramie --fake, kamera --source sim")
    parser.add_argument("--arm-home", action="store_true", help="ramie z HOME przy starcie (domyslnie --no-home)")
    parser.add_argument("--vision-source", default=None, help="zrodlo kamery dla tools/vision_web.py --source")
    args = parser.parse_args()

    events = EventLog()
    services = [Service(spec, REPO_ROOT, LOG_DIR, events)
                for spec in build_specs(args.demo, args.arm_home, args.vision_source)]
    hub = Hub(services, events)
    threading.Thread(target=hub.facts, daemon=True).start()  # git i liczenie linii w tle
    events.add("start", "panel wystartowal" + (" (demo)" if args.demo else ""))
    if args.autostart:
        for s in services:
            ok, msg = s.start()
            print(msg, flush=True)

    httpd = ThreadingHTTPServer((args.host, args.http_port), make_handler(hub, args.http_port))
    httpd.daemon_threads = True
    print(f"panel: http://<IP>:{args.http_port}  (pokaz: /show)", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("koniec, zatrzymuje uslugi", flush=True)
    finally:
        httpd.server_close()
        for msg in hub.stop_all():
            print(msg, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
