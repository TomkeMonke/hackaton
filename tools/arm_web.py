"""Panel webowy do recznego sterowania ramieniem SO-101 (arm_panel.html).

Uzycie (na Pi, z katalogu repo):
    python tools/arm_web.py                       # port ramienia z ROBOT_ARM_PORT albo cfg.arm.port
    python tools/arm_web.py --port /dev/robot-arm
    python tools/arm_web.py --fake                # atrapa ramienia, bez lerobot (laptop)
    python tools/arm_web.py --no-home             # bez HOME (np. kamera na ramieniu): jog, chwytak, ruchy nagrane z panelu

Potem w przegladarce: http://<IP_PI>:8010 (sam panel ramienia) albo
http://<IP_PI>:8000 (panel jazdy web_control.py z sekcja ramienia - ten sam
arm_panel.js, pyta ten serwer przez CORS). Dwa osobne procesy: blad magistrali
ramienia nie zatrzymuje jazdy, a lerobot jest importowany tylko tutaj.

Po starcie ramie NAJPIERW jedzie do HOME; do tego czasu panel przyjmuje tylko
HOME i STOP. Jedna komenda naraz (kolejka w pinecone_bot/arm_panel.py),
zakres z kalibracji serw, max_relative_target=None + wlasny limit kroku,
odczyt pozycji max 2 Hz i tylko gdy ramie stoi (docs/HARDWARE.md, pulapka 10).

Nagrywanie ruchu z panelu: ustaw ramie jogiem, "+ PUNKT" dodaje biezaca poze do szkicu,
"ZAPISZ do motions/" pisze motions/<nazwa>.json (format jak tools/record_waypoints.py).
Taki ruch odtwarza sie z panelu i jako krok "arm" sekwencji w panelu jazdy (:8000).
--no-home: HOME wylaczone, ruchy z motions/ dozwolone, ale po pustym chwycie ramie NIE wraca
do HOME (kamera na ramieniu) - odtwarzaj tylko ruchy nagrane pod aktualny montaz.

Ctrl+C rozlacza ramie i lerobot WYLACZA torque - ramie opadnie. Najpierw HOME.

Porty: 8000/8765 = web_control.py (jazda), 8001 = STOP bazy, 8010 = ten panel.
Nie uruchamia `lerobot calibrate` i nie zapisuje kalibracji (connect(calibrate=False)).
"""
from __future__ import annotations

import argparse
import http.server
import json
import logging
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot.arm_panel import (  # noqa: E402
    FAKE_CALIBRATION,
    FAKE_NORM_MODES,
    ArmPanel,
    FakeSO101,
    limits_from_calibration,
)
from pinecone_bot.config import Config  # noqa: E402
from pinecone_bot.kinematics import So101Kinematics  # noqa: E402

STATIC = {
    "/": ("arm_panel.html", "text/html; charset=utf-8"),
    "/arm_panel.html": ("arm_panel.html", "text/html; charset=utf-8"),
    "/arm_panel.js": ("arm_panel.js", "text/javascript; charset=utf-8"),
}
HTTP_PORT = 8010
MAX_BODY = 4096
DRIVE_PANEL_PORT = 8000  # web_control.py; tylko jego strona moze wolac API z innego originu

log = logging.getLogger("arm_web")


def connect_real_arm(port: str, arm_id: str):
    """Laczy ramie przez arm_control (lerobot importowany dopiero tutaj). Zwraca (arm, limits)."""
    import arm_control  # lazy: ciagnie lerobot

    arm = arm_control.make_arm(port=port, arm_id=arm_id, max_relative_target=None)
    if not arm.calibration:
        raise SystemExit(
            f"brak pliku kalibracji {arm.calibration_fpath} - skopiuj go (NIE uruchamiaj lerobot calibrate)"
        )
    modes = {name: motor.norm_mode.name for name, motor in arm.bus.motors.items()}
    limits = limits_from_calibration(arm.calibration, modes)
    arm.connect(calibrate=False)
    return arm, limits


def origin_port_ok(origin: str, ports) -> bool:
    """True dla http://<dowolny host>:<port z listy>. Host zalezy od sieci (hotspot, LAN), port nie."""
    scheme, _, rest = origin.partition("://")
    return scheme == "http" and "/" not in rest and rest.rsplit(":", 1)[-1] in {str(p) for p in ports}


def make_handler(panel: ArmPanel, own_port: int = HTTP_PORT):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # bez linii w konsoli co 0.5 s od odpytywania stanu
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload: dict) -> None:
            self._send(code, json.dumps(payload).encode("ascii"), "application/json")

        def _cors(self) -> None:
            # frontend.html (panel jazdy, :8000) pyta ten serwer z innego portu. Nie "*":
            # inaczej dowolna strona otwarta w przegladarce w tej sieci moglaby ruszac ramieniem.
            origin = self.headers.get("Origin") or ""
            if origin_port_ok(origin, [DRIVE_PANEL_PORT]):
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")

        def do_OPTIONS(self):  # preflight CORS przed POST z application/json
            self.send_response(204)
            self._cors()
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Max-Age", "600")
            self.end_headers()

        def do_GET(self):
            if self.path in STATIC:
                name, ctype = STATIC[self.path]
                with open(os.path.join(REPO_ROOT, name), "rb") as fh:
                    self._send(200, fh.read(), ctype)
            elif self.path == "/api/state":
                self._json(200, panel.snapshot())
            else:
                self._json(404, {"ok": False, "msg": "nie ma"})

        def do_POST(self):
            if self.path != "/api/cmd":
                self._json(404, {"ok": False, "msg": "nie ma"})
                return
            # Komendy tylko z naszych stron. application/json wymusza preflight CORS, wiec obca
            # strona nie przemyci komendy "prostym" POST-em (text/plain), ktory idzie bez pytania.
            origin = self.headers.get("Origin")
            if origin and not origin_port_ok(origin, [DRIVE_PANEL_PORT, own_port]):
                self._json(403, {"ok": False, "msg": "obcy origin"})
                return
            if not (self.headers.get("Content-Type") or "").startswith("application/json"):
                self._json(415, {"ok": False, "msg": "wymagany application/json"})
                return
            length = int(self.headers.get("Content-Length") or 0)
            if length <= 0 or length > MAX_BODY:
                self._json(400, {"ok": False, "msg": "zla dlugosc"})
                return
            try:
                data = json.loads(self.rfile.read(length))
            except ValueError:
                self._json(400, {"ok": False, "msg": "zly JSON"})
                return
            if not isinstance(data, dict):
                self._json(400, {"ok": False, "msg": "zly JSON"})
                return
            ok, msg = panel.submit(data)
            if ok:
                log.info("komenda %s", data)
            self._json(200 if ok else 409, {"ok": ok, "msg": msg})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="port ramienia (domyslnie ROBOT_ARM_PORT albo cfg.arm.port)")
    parser.add_argument("--id", default=None, help="domyslnie cfg.arm.arm_id")
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "0.0.0.0"))
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    parser.add_argument("--fake", action="store_true", help="atrapa ramienia (bez lerobot i sprzetu)")
    parser.add_argument("--no-home", action="store_true",
                        help="bez HOME przy starcie i bez HOME po pustym chwycie (kamera na ramieniu); "
                             "jog, chwytak i ruchy z motions/ dozwolone")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    cfg = Config.load()
    port = args.port or os.environ.get("ROBOT_ARM_PORT") or cfg.arm.port
    arm_id = args.id or cfg.arm.arm_id

    if args.fake:
        arm = FakeSO101()
        limits = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
        print("ATRAPA ramienia (--fake), sprzet nie jest uzywany")
    else:
        print(f"lacze ramie: port {port}, id {arm_id}")
        arm, limits = connect_real_arm(port, arm_id)
    for joint, (lo, hi) in limits.items():
        print(f"  zakres {joint:14s} {lo:8.1f} .. {hi:8.1f}")

    try:
        kin = So101Kinematics(cfg.arm.urdf_path, cfg.arm.urdf_sign, cfg.arm.urdf_offset_deg)
        print(f"jog XYZ: URDF {cfg.arm.urdf_path}, offsety {cfg.arm.urdf_offset_deg or 'brak (ZERO URDF w panelu)'}")
    except Exception as exc:  # noqa: BLE001 - bez URDF panel dziala dalej, tylko bez jogu XYZ
        kin = None
        print(f"jog XYZ wylaczony: {exc}")

    panel = ArmPanel(arm, cfg, limits, manual_only=args.no_home, kinematics=kin, config_path=Config.default_path())
    panel.start(home_first=True)
    if args.no_home:
        print("--no-home: ramie stoi, HOME wylaczone; jog od odczytanej pozycji; ruchy z motions/ bez powrotu do HOME")
    else:
        print("ramie jedzie do HOME...")

    httpd = http.server.ThreadingHTTPServer((args.host, args.http_port), make_handler(panel, args.http_port))
    print(f"panel: http://<IP>:{args.http_port} (nasluch {args.host})")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("koniec")
    finally:
        httpd.server_close()
        panel.shutdown()
        try:
            arm.disconnect()
        except Exception as exc:  # noqa: BLE001
            print(f"(disconnect nieudany, pewnie juz rozlaczone: {exc})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
