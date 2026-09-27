"""
Web frontend for driving the hoverboard via the Xiao's ASCII serial
interface (xiao_send_pwm.ino), replacing keyboard_control.py's raw
terminal input with a browser UI.

Runs two servers:
  - HTTP  on :8000 -> serves frontend.html
  - WebSocket on :8765 -> receives key state from the browser,
    drives the motors, and broadcasts live status back to it.

Open http://localhost:8000 in a browser after starting this script (or
http://<robot-ip>:8000 from another device on the same network).

Configuration via environment variables (defaults suit the Windows dev PC):
  ROBOT_DRIVE_PORT  serial port of the Xiao (default COM9; on the Pi: /dev/robot-drive)
  ROBOT_HOST        interface to listen on (default 0.0.0.0 = all, reachable over WiFi)

Safety: if no browser message arrives for HEARTBEAT_TIMEOUT seconds (all tabs
closed, WiFi dropped, phone locked), the robot stops and drops back to manual
mode. The frontend sends a ping every 200 ms to keep the link alive.

Sequences ("hardcoded pinecone pickup"): sequences/<name>.json is a list of
steps (drive for N seconds, arm motion from motions/, wait) played back in
order by pinecone_bot/sequence.py in mode "sequence". Arm steps go to the
arm panel server (tools/arm_web.py, ROBOT_ARM_PANEL, default
http://127.0.0.1:8010) over HTTP; a lost heartbeat, STOP or any mode change
aborts the sequence, zeroes the drive and sends STOP to the arm.

Phone e-stop: http://<robot-ip>:8000/stop (stop.html) is one big STOP button.
It talks plain HTTP (POST /api/estop), not the WebSocket, so it never counts as
an operator heartbeat: a phone left on that page cannot keep the robot alive
after the driving browser drops. The stop latches: drive stays at zero (keys,
modes and sequences ignored) until someone presses ODBLOKUJ on /stop. The
release carries the latch number, so a delayed release cannot undo a newer STOP.
"""

import asyncio
import functools
import http.server
import json
import os
import sys
import threading
import urllib.error
import urllib.request
from pathlib import Path
from time import time

import serial
import websockets

sys.path.insert(0, str(Path(__file__).parent))
from pinecone_bot.sequence import (  # noqa: E402
    SequenceRunner,
    delete_sequence,
    load_sequences,
    parse_steps,
    save_sequence,
)

PORT_SERIAL = os.environ.get("ROBOT_DRIVE_PORT", "COM9")
BAUDRATE = 115200

HOST = os.environ.get("ROBOT_HOST", "0.0.0.0")
HTTP_PORT = 8000
WS_PORT = 8765

MAX_PWM = 100  # was 500; halved 2026-09-26 after the robot drove into the arm over a laggy hotspot
MAX_STEER = 400

LOOP_DELAY = 0.03  # matches the Arduino's loop delay / well under its 500ms timeout

# Operator dead-man: stop if the browser goes silent this long. The Xiao's own
# watchdog can't catch a dropped WiFi link, because the Pi keeps sending commands.
# 1.0 s: the iPhone hotspot has latency spikes that tripped 0.5 s several times a second.
HEARTBEAT_TIMEOUT = float(os.environ.get("ROBOT_HEARTBEAT_TIMEOUT", "1.0"))

# Default values for the live-tunable params below (all overridable from the frontend).
DEFAULT_PARAMS = {
    "accel_step": 0.06,      # manual mode: per-tick ramp toward the WASD target
    "fig8_speed": 0.06,      # figure-eight: constant forward speed fraction
    "fig8_steer": 0.025,     # figure-eight: steer fraction while arcing (keep well below fig8_speed!)
    "fig8_ramp": 0.02,       # figure-eight: per-tick ramp
    "fig8_loop_seconds": 10.0,  # figure-eight: seconds per half-loop before switching direction
    # coverage ("lawnmower", S-path): forward a lane, pivot ~90 deg twice (same direction) to
    # shift into the next lane heading the opposite way, then the next U-turn goes the other
    # way (left, right, left...) -> sweeps the whole floor instead of shuttling between 2 lanes.
    "cov_speed": 0.06,        # forward speed fraction while driving a lane
    "cov_forward_seconds": 6.0,   # how long to drive straight per lane
    "cov_turn_steer": 0.4,    # steer fraction while pivoting (in-place turn, speed=0)
    "cov_turn_seconds": 1.0,  # how long each ~90 deg pivot takes
    "cov_lane_seconds": 1.0,  # short forward creep between the two pivots (lane offset)
}
PARAM_LIMITS = {
    "accel_step": (0.005, 0.3),
    "fig8_speed": (0.0, 1.0),
    "fig8_steer": (0.0, 1.0),
    "fig8_ramp": (0.005, 0.3),
    "fig8_loop_seconds": (0.5, 60.0),
    "cov_speed": (0.0, 1.0),
    "cov_forward_seconds": (0.5, 120.0),
    "cov_turn_steer": (0.0, 1.0),
    "cov_turn_seconds": (0.1, 10.0),
    "cov_lane_seconds": (0.0, 10.0),
}

STATIC_DIR = Path(__file__).parent
RECORDINGS_DIR = STATIC_DIR / "recordings"
SEQUENCES_DIR = str(STATIC_DIR / "sequences")

# Panel ramienia (tools/arm_web.py) - kroki "arm" w sekwencji ida tam przez HTTP.
ARM_PANEL_URL = os.environ.get("ROBOT_ARM_PANEL", "http://127.0.0.1:8010").rstrip("/")
ARM_HTTP_TIMEOUT = 2.0


def arm_post(cmd):
    """POST /api/cmd do panelu ramienia -> (ok, msg). Brak polaczenia = (False, msg)."""
    body = json.dumps(cmd).encode("ascii")
    req = urllib.request.Request(
        f"{ARM_PANEL_URL}/api/cmd", data=body, method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=ARM_HTTP_TIMEOUT) as resp:
            data = json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        try:
            data = json.loads(exc.read())
        except ValueError:
            return False, f"panel ramienia: HTTP {exc.code}"
    except (urllib.error.URLError, OSError, ValueError) as exc:
        return False, f"panel ramienia niedostepny ({ARM_PANEL_URL}): {exc}"
    return bool(data.get("ok")), str(data.get("msg", ""))


def arm_state():
    """GET /api/state panelu ramienia -> dict albo None, gdy nie odpowiada."""
    try:
        with urllib.request.urlopen(f"{ARM_PANEL_URL}/api/state", timeout=ARM_HTTP_TIMEOUT) as resp:
            return json.loads(resp.read())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def arm_stop():
    arm_post({"cmd": "stop"})


def step_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


COV_NEXT_PHASE = {"forward": "turn1", "turn1": "lane", "lane": "turn2", "turn2": "forward"}


def coverage_advance(phase, turn_dir):
    """Next (phase, turn_dir) of the S-path. The U-turn side flips after every turn2, so
    consecutive U-turns alternate left/right and the lanes step across the floor."""
    if phase == "turn2":
        turn_dir = -turn_dir
    return COV_NEXT_PHASE[phase], turn_dir


def load_recordings():
    recordings = {}
    if RECORDINGS_DIR.exists():
        for f in RECORDINGS_DIR.glob("*.json"):
            try:
                recordings[f.stem] = json.loads(f.read_text())
            except (json.JSONDecodeError, OSError):
                pass
    return recordings


def save_recording_to_disk(name, samples):
    RECORDINGS_DIR.mkdir(exist_ok=True)
    safe_name = "".join(c for c in name if c.isalnum() or c in "-_") or "recording"
    (RECORDINGS_DIR / f"{safe_name}.json").write_text(json.dumps(samples))
    return safe_name


def delete_recording_from_disk(name):
    f = RECORDINGS_DIR / f"{name}.json"
    if f.exists():
        f.unlink()


class RobotState:
    def __init__(self):
        self.keys = {"w": False, "a": False, "s": False, "d": False}
        self.speed = 0.0
        self.steer = 0.0
        self.ser = None
        self.serial_error = None
        self.clients = set()
        self.last_client_msg = 0.0  # time() of the last message from any browser
        self.failsafe = True  # True while stopped for lack of an operator
        self.mode = "manual"  # "manual" | "figure8" | "coverage" | "playback"
        self.fig8_direction = 1
        self.fig8_half_start = time()
        self.cov_phase = "forward"  # "forward" | "turn1" | "lane" | "turn2"
        self.cov_phase_start = time()
        self.cov_turn_dir = 1  # +1 / -1: side of the current U-turn, flips after each one
        self.speed_scale = 1.0  # 0..1, multiplies both manual and figure8 speed targets
        self.params = dict(DEFAULT_PARAMS)

        self.recording = False
        self.record_buffer = []  # list of [t_offset, speed, steer]
        self.record_start = 0.0
        self.recordings = load_recordings()  # name -> list of [t_offset, speed, steer]

        self.playback_name = None
        self.playback_start = 0.0
        self.playback_index = 0

        # sekwencje (jazda + ramie); runner ustawia seq_speed/seq_steer z wlasnego watku
        self.seq_speed = 0.0
        self.seq_steer = 0.0
        self.sequences = load_sequences(SEQUENCES_DIR)  # name -> steps
        self.runner = SequenceRunner(
            drive=self._set_seq_target, arm_start=lambda name: arm_post({"cmd": "motion", "name": name}),
            arm_state=arm_state, arm_stop=arm_stop,
        )
        self.seq_error = None  # ostatni blad zapisu/uruchomienia (dla przegladarki)

        # wylacznik z telefonu (/stop): zatrzask + numer, zeby spozniony ODBLOKUJ nie zdjal nowszego STOP
        self.estop_latched = False
        self.estop_id = 0

    def estop(self):
        """STOP z /stop (watek HTTP). Petla sterowania zeruje jazde w nastepnym ticku. Zwraca numer zatrzasku."""
        self.estop_id += 1
        self.estop_latched = True
        return self.estop_id

    def estop_release(self, estop_id):
        """ODBLOKUJ z /stop. False = numer nieaktualny (w miedzyczasie ktos wcisnal STOP)."""
        if not self.estop_latched:
            return True
        if estop_id != self.estop_id:
            return False
        self.estop_latched = False
        return True

    def hard_stop(self):
        """Zero jazdy bez rampy, koniec trybow auto, sekwencji i nagrywania; klawisze trzeba wcisnac od nowa."""
        if self.mode == "sequence":
            self.stop_sequence()
        self.mode = "manual"
        self.playback_name = None
        self.recording = False
        self.record_buffer = []
        self.keys = {"w": False, "a": False, "s": False, "d": False}
        self.speed = 0.0
        self.steer = 0.0

    def _set_seq_target(self, speed, steer):
        self.seq_speed = speed
        self.seq_steer = steer

    def start_sequence(self, name, steps):
        """True = ruszyla. Zeruje klawisze i przelacza tryb; jedna sekwencja naraz."""
        if self.estop_latched:
            # nie startuj wcale: pierwszy krok ramienia poszedlby, zanim petla zdazy przerwac
            self.seq_error = "E-STOP z telefonu - odblokuj na /stop"
            return False
        if self.runner.status()["running"]:
            self.seq_error = "inna sekwencja jeszcze trwa"
            return False
        self.recording = False
        self.record_buffer = []
        self.playback_name = None
        self.keys = {"w": False, "a": False, "s": False, "d": False}
        self.seq_speed = 0.0
        self.seq_steer = 0.0
        self.mode = "sequence"
        self.seq_error = None
        return self.runner.start(name, steps)

    def stop_sequence(self):
        self.runner.stop()
        self.seq_speed = 0.0
        self.seq_steer = 0.0

    def connect_serial(self):
        try:
            self.ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=0)
            self.serial_error = None
        except serial.SerialException as exc:
            self.ser = None
            self.serial_error = str(exc)


state = RobotState()


def estop_status():
    return {
        "latched": state.estop_latched,
        "id": state.estop_id,
        "mode": state.mode,
        "speed": round(state.speed, 2),
        "steer": round(state.steer, 2),
        "failsafe": state.failsafe,
        "connected": state.ser is not None,
    }


class FrontendHandler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        if path == "/":
            path = "/frontend.html"
        elif path == "/stop":
            path = "/stop.html"
        return super().translate_path(path)

    def log_message(self, fmt, *args):
        if self.path.startswith("/api/estop"):
            return  # /stop odpytuje stan co 0.5 s - bez tego log zarasta
        super().log_message(fmt, *args)

    def _json(self, code, payload):
        body = json.dumps(payload).encode("ascii")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/estop":
            self._json(200, estop_status())
        else:
            super().do_GET()

    def do_POST(self):
        # body zawsze przeczytac przed odpowiedzia: zamkniecie gniazda z nieprzeczytanymi danymi = RST u klienta
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if 0 < length <= 1024 else b""
        if self.path not in ("/api/estop", "/api/estop_release"):
            self._json(404, {"ok": False, "msg": "nie ma"})
            return
        # Tylko z naszej strony: application/json wymusza preflight CORS (na OPTIONS nie odpowiadamy),
        # wiec obca strona w tej sieci nie zdejmie zatrzasku.
        origin = self.headers.get("Origin")
        if origin and not (origin.startswith("http://") and origin.endswith(f":{HTTP_PORT}")):
            self._json(403, {"ok": False, "msg": "obcy origin"})
            return
        if not (self.headers.get("Content-Type") or "").startswith("application/json"):
            self._json(415, {"ok": False, "msg": "wymagany application/json"})
            return
        try:
            data = json.loads(raw) if raw else {}
        except ValueError:
            data = {}
        if not isinstance(data, dict):
            data = {}

        if self.path == "/api/estop":
            estop_id = state.estop()
            print(f"E-STOP #{estop_id} z {self.client_address[0]}")
            threading.Thread(target=arm_stop, daemon=True).start()  # HTTP do ramienia do 2 s - nie blokuj odpowiedzi
            self._json(200, {"ok": True, **estop_status()})
        else:
            try:
                estop_id = int(data.get("id"))
            except (TypeError, ValueError):
                self._json(400, {"ok": False, "msg": "brak numeru STOP"})
                return
            if state.estop_release(estop_id):
                print(f"E-STOP #{estop_id} odblokowany z {self.client_address[0]}")
                self._json(200, {"ok": True, **estop_status()})
            else:
                self._json(409, {"ok": False, "msg": "w miedzyczasie nowszy STOP", **estop_status()})


def start_http_server():
    handler = functools.partial(FrontendHandler, directory=str(STATIC_DIR))
    httpd = http.server.ThreadingHTTPServer((HOST, HTTP_PORT), handler)
    httpd.serve_forever()


async def ws_handler(websocket):
    state.clients.add(websocket)
    try:
        async for message in websocket:
            state.last_client_msg = time()
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue
            if data.get("type") == "ping":
                continue
            if data.get("type") == "keys":
                for k in state.keys:
                    if k in data:
                        state.keys[k] = bool(data[k])
            elif data.get("type") == "set_mode":
                new_mode = data.get("mode")
                if new_mode in ("manual", "figure8", "coverage"):
                    if state.mode == "sequence":
                        state.stop_sequence()
                    if state.recording and new_mode != "manual":
                        state.recording = False
                        state.record_buffer = []
                    if new_mode == "figure8" and state.mode != "figure8":
                        state.fig8_direction = 1
                        state.fig8_half_start = time()
                    if new_mode == "coverage" and state.mode != "coverage":
                        state.cov_phase = "forward"
                        state.cov_phase_start = time()
                        state.cov_turn_dir = 1
                    state.playback_name = None
                    state.mode = new_mode
                    if new_mode == "manual":
                        state.keys = {"w": False, "a": False, "s": False, "d": False}
            elif data.get("type") == "set_speed":
                try:
                    state.speed_scale = max(0.0, min(1.0, float(data.get("value", 1.0))))
                except (TypeError, ValueError):
                    pass
            elif data.get("type") == "set_params":
                values = data.get("values", {})
                for key, (lo, hi) in PARAM_LIMITS.items():
                    if key in values:
                        try:
                            state.params[key] = max(lo, min(hi, float(values[key])))
                        except (TypeError, ValueError):
                            pass
            elif data.get("type") == "start_record":
                state.mode = "manual"
                state.keys = {"w": False, "a": False, "s": False, "d": False}
                state.recording = True
                state.record_buffer = []
                state.record_start = time()
            elif data.get("type") == "stop_record":
                if state.recording:
                    state.recording = False
                    name = str(data.get("name") or "nagranie").strip()
                    if state.record_buffer:
                        safe_name = save_recording_to_disk(name, state.record_buffer)
                        state.recordings[safe_name] = state.record_buffer
                    state.record_buffer = []
            elif data.get("type") == "play_record":
                name = data.get("name")
                if name in state.recordings and state.recordings[name]:
                    state.mode = "playback"
                    state.playback_name = name
                    state.playback_start = time()
                    state.playback_index = 0
            elif data.get("type") == "delete_record":
                name = data.get("name")
                if name in state.recordings:
                    del state.recordings[name]
                    delete_recording_from_disk(name)
            elif data.get("type") == "drive_step":
                # pojedynczy krok jazdy do sprawdzenia (ten sam kod co w sekwencji)
                try:
                    steps = parse_steps([{"type": "drive", "speed": data.get("speed"),
                                          "steer": data.get("steer"), "seconds": data.get("seconds")}])
                except ValueError as exc:
                    state.seq_error = str(exc)
                else:
                    state.start_sequence("krok", steps)
            elif data.get("type") == "play_sequence":
                name = data.get("name")
                if name in state.sequences:
                    state.start_sequence(name, state.sequences[name])
            elif data.get("type") == "stop_sequence":
                state.stop_sequence()
            elif data.get("type") == "save_sequence":
                try:
                    name = str(data.get("name") or "").strip()
                    save_sequence(SEQUENCES_DIR, name, data.get("steps"))
                    state.sequences[name] = parse_steps(data.get("steps"))
                    state.seq_error = None
                except (ValueError, OSError) as exc:
                    state.seq_error = f"zapis sekwencji: {exc}"
            elif data.get("type") == "delete_sequence":
                name = data.get("name")
                if name in state.sequences:
                    del state.sequences[name]
                    delete_sequence(SEQUENCES_DIR, name)
    finally:
        state.clients.discard(websocket)


async def broadcast(payload):
    if not state.clients:
        return
    dead = []
    msg = json.dumps(payload)
    for ws in list(state.clients):
        try:
            await ws.send(msg)
        except websockets.exceptions.ConnectionClosed:
            dead.append(ws)
    for ws in dead:
        state.clients.discard(ws)


async def control_loop():
    last_reconnect_attempt = 0.0

    while True:
        if state.ser is None and time() - last_reconnect_attempt > 2.0:
            state.connect_serial()
            last_reconnect_attempt = time()

        p = state.params

        operator_lost = not state.clients or time() - state.last_client_msg > HEARTBEAT_TIMEOUT
        if state.failsafe != operator_lost:
            print("Failsafe: no operator heartbeat, stopping." if operator_lost else "Operator heartbeat back.")
            state.failsafe = operator_lost

        if operator_lost or state.estop_latched:
            # Hard stop (no ramp) and drop any auto mode: nobody is watching the robot,
            # or the phone e-stop is latched (then every tick, so keys/modes stay ignored).
            state.hard_stop()
        elif state.mode == "figure8":
            if time() - state.fig8_half_start >= p["fig8_loop_seconds"]:
                state.fig8_direction *= -1
                state.fig8_half_start = time()
            speed_target = p["fig8_speed"] * state.speed_scale
            steer_target = state.fig8_direction * p["fig8_steer"] * state.speed_scale
            state.speed = step_toward(state.speed, speed_target, p["fig8_ramp"])
            state.steer = step_toward(state.steer, steer_target, p["fig8_ramp"])
        elif state.mode == "coverage":
            phase_durations = {
                "forward": p["cov_forward_seconds"],
                "turn1": p["cov_turn_seconds"],
                "lane": p["cov_lane_seconds"],
                "turn2": p["cov_turn_seconds"],
            }
            if time() - state.cov_phase_start >= phase_durations[state.cov_phase]:
                state.cov_phase, state.cov_turn_dir = coverage_advance(state.cov_phase, state.cov_turn_dir)
                state.cov_phase_start = time()

            if state.cov_phase in ("forward", "lane"):
                speed_target = p["cov_speed"] * state.speed_scale
                steer_target = 0.0
            else:
                speed_target = 0.0
                steer_target = state.cov_turn_dir * p["cov_turn_steer"] * state.speed_scale

            state.speed = step_toward(state.speed, speed_target, p["accel_step"])
            state.steer = step_toward(state.steer, steer_target, p["accel_step"])
        elif state.mode == "sequence":
            if not state.runner.status()["running"]:
                state.mode = "manual"   # sekwencja skonczona / przerwana / blad (szczegoly w statusie)
                state.seq_speed = 0.0
                state.seq_steer = 0.0
            state.speed = step_toward(state.speed, state.seq_speed, p["accel_step"])
            state.steer = step_toward(state.steer, state.seq_steer, p["accel_step"])
        elif state.mode == "playback":
            samples = state.recordings.get(state.playback_name) or []
            elapsed = time() - state.playback_start
            idx = state.playback_index
            while idx + 1 < len(samples) and samples[idx + 1][0] <= elapsed:
                idx += 1
            state.playback_index = idx
            if idx >= len(samples) - 1 and (not samples or elapsed >= samples[-1][0]):
                state.speed = 0.0
                state.steer = 0.0
                state.mode = "manual"
                state.playback_name = None
            else:
                state.speed = samples[idx][1]
                state.steer = samples[idx][2]
        else:
            speed_target = (1.0 if state.keys["w"] else 0.0) - (1.0 if state.keys["s"] else 0.0)
            speed_target *= state.speed_scale
            steer_target = (1.0 if state.keys["d"] else 0.0) - (1.0 if state.keys["a"] else 0.0)
            steer_target *= state.speed_scale
            state.speed = step_toward(state.speed, speed_target, p["accel_step"])
            state.steer = step_toward(state.steer, steer_target, p["accel_step"])

        if state.recording:
            state.record_buffer.append([round(time() - state.record_start, 3), state.speed, state.steer])

        pwm_speed = int(state.speed * MAX_PWM)
        pwm_steer = int(state.steer * MAX_STEER)

        if state.ser is not None:
            try:
                state.ser.write(f"a{pwm_speed} b{pwm_steer}\n".encode("ascii"))
            except serial.SerialException as exc:
                state.serial_error = str(exc)
                state.ser = None

        await broadcast(
            {
                "type": "status",
                "connected": state.ser is not None,
                "error": state.serial_error,
                "speed": round(state.speed, 2),
                "steer": round(state.steer, 2),
                "pwm_speed": pwm_speed,
                "pwm_steer": pwm_steer,
                "keys": state.keys,
                "mode": state.mode,
                "cov_phase": state.cov_phase if state.mode == "coverage" else None,
                "speed_scale": round(state.speed_scale, 2),
                "params": state.params,
                "port": PORT_SERIAL,
                "recording": state.recording,
                "record_seconds": round(time() - state.record_start, 1) if state.recording else 0,
                "recordings": {
                    name: round(samples[-1][0], 1) if samples else 0.0
                    for name, samples in state.recordings.items()
                },
                "playback_name": state.playback_name,
                "failsafe": state.failsafe,
                "estop": state.estop_latched,
                "sequence": state.runner.status(),
                "sequences": state.sequences,
                "seq_error": state.seq_error,
                "arm_panel": ARM_PANEL_URL,
            }
        )

        await asyncio.sleep(LOOP_DELAY)


async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    print(f"Frontend: http://localhost:{HTTP_PORT} (listening on {HOST})")
    print(f"WebSocket control on ws://localhost:{WS_PORT}")
    print(f"Drive serial port: {PORT_SERIAL}")
    print(f"Arm panel for sequences: {ARM_PANEL_URL} (sequences in {SEQUENCES_DIR}: {', '.join(state.sequences) or 'none'})")

    state.connect_serial()
    if state.ser is None:
        print(f"Warning: could not open {PORT_SERIAL} yet ({state.serial_error}). Will keep retrying.")

    async with websockets.serve(ws_handler, HOST, WS_PORT):
        try:
            await control_loop()
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            if state.ser is not None:
                try:
                    state.ser.write(b"a0 b0\n")
                except serial.SerialException:
                    pass
                state.ser.close()


if __name__ == "__main__":
    asyncio.run(main())
