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
"""

import asyncio
import functools
import http.server
import json
import os
import threading
from pathlib import Path
from time import time

import serial
import websockets

PORT_SERIAL = os.environ.get("ROBOT_DRIVE_PORT", "COM9")
BAUDRATE = 115200

HOST = os.environ.get("ROBOT_HOST", "0.0.0.0")
HTTP_PORT = 8000
WS_PORT = 8765

MAX_PWM = 500
MAX_STEER = 400

LOOP_DELAY = 0.03  # matches the Arduino's loop delay / well under its 500ms timeout

# Operator dead-man: stop if the browser goes silent this long. The Xiao's own
# watchdog can't catch a dropped WiFi link, because the Pi keeps sending commands.
HEARTBEAT_TIMEOUT = 0.5

# Default values for the live-tunable params below (all overridable from the frontend).
DEFAULT_PARAMS = {
    "accel_step": 0.06,      # manual mode: per-tick ramp toward the WASD target
    "fig8_speed": 0.06,      # figure-eight: constant forward speed fraction
    "fig8_steer": 0.025,     # figure-eight: steer fraction while arcing (keep well below fig8_speed!)
    "fig8_ramp": 0.02,       # figure-eight: per-tick ramp
    "fig8_loop_seconds": 10.0,  # figure-eight: seconds per half-loop before switching direction
    # coverage ("lawnmower"): forward a lane, pivot ~90 deg twice (same direction) to
    # shift into the next lane heading the opposite way, repeat -> sweeps the whole floor.
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


def step_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


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
        self.speed_scale = 1.0  # 0..1, multiplies both manual and figure8 speed targets
        self.params = dict(DEFAULT_PARAMS)

        self.recording = False
        self.record_buffer = []  # list of [t_offset, speed, steer]
        self.record_start = 0.0
        self.recordings = load_recordings()  # name -> list of [t_offset, speed, steer]

        self.playback_name = None
        self.playback_start = 0.0
        self.playback_index = 0

    def connect_serial(self):
        try:
            self.ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=0)
            self.serial_error = None
        except serial.SerialException as exc:
            self.ser = None
            self.serial_error = str(exc)


state = RobotState()


class FrontendHandler(http.server.SimpleHTTPRequestHandler):
    def translate_path(self, path):
        if path == "/":
            path = "/frontend.html"
        return super().translate_path(path)


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
                    if state.recording and new_mode != "manual":
                        state.recording = False
                        state.record_buffer = []
                    if new_mode == "figure8" and state.mode != "figure8":
                        state.fig8_direction = 1
                        state.fig8_half_start = time()
                    if new_mode == "coverage" and state.mode != "coverage":
                        state.cov_phase = "forward"
                        state.cov_phase_start = time()
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

        if operator_lost:
            # Hard stop (no ramp) and drop any auto mode: nobody is watching the robot.
            state.mode = "manual"
            state.playback_name = None
            state.recording = False
            state.record_buffer = []
            state.keys = {"w": False, "a": False, "s": False, "d": False}
            state.speed = 0.0
            state.steer = 0.0
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
            next_phase = {"forward": "turn1", "turn1": "lane", "lane": "turn2", "turn2": "forward"}
            if time() - state.cov_phase_start >= phase_durations[state.cov_phase]:
                state.cov_phase = next_phase[state.cov_phase]
                state.cov_phase_start = time()

            if state.cov_phase in ("forward", "lane"):
                speed_target = p["cov_speed"] * state.speed_scale
                steer_target = 0.0
            else:
                speed_target = 0.0
                steer_target = p["cov_turn_steer"] * state.speed_scale

            state.speed = step_toward(state.speed, speed_target, p["accel_step"])
            state.steer = step_toward(state.steer, steer_target, p["accel_step"])
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
            }
        )

        await asyncio.sleep(LOOP_DELAY)


async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    print(f"Frontend: http://localhost:{HTTP_PORT} (listening on {HOST})")
    print(f"WebSocket control on ws://localhost:{WS_PORT}")
    print(f"Drive serial port: {PORT_SERIAL}")

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
