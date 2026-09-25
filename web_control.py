"""
Web frontend for driving the hoverboard via the Xiao's ASCII serial
interface (xiao_send_pwm.ino), replacing keyboard_control.py's raw
terminal input with a browser UI.

Runs two servers:
  - HTTP  on :8000 -> serves frontend.html
  - WebSocket on :8765 -> receives key state from the browser,
    drives the motors, and broadcasts live status back to it.

Open http://localhost:8000 in a browser after starting this script.
"""

import asyncio
import functools
import http.server
import json
import threading
from pathlib import Path
from time import time

import serial
import websockets

PORT_SERIAL = "COM9"
BAUDRATE = 115200

HTTP_PORT = 8000
WS_PORT = 8765

MAX_PWM = 500
MAX_STEER = 400

LOOP_DELAY = 0.03  # matches the Arduino's loop delay / well under its 500ms timeout

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


def step_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


class RobotState:
    def __init__(self):
        self.keys = {"w": False, "a": False, "s": False, "d": False}
        self.speed = 0.0
        self.steer = 0.0
        self.ser = None
        self.serial_error = None
        self.clients = set()
        self.mode = "manual"  # "manual" | "figure8" | "coverage"
        self.fig8_direction = 1
        self.fig8_half_start = time()
        self.cov_phase = "forward"  # "forward" | "turn1" | "lane" | "turn2"
        self.cov_phase_start = time()
        self.speed_scale = 1.0  # 0..1, multiplies both manual and figure8 speed targets
        self.params = dict(DEFAULT_PARAMS)

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
    httpd = http.server.ThreadingHTTPServer(("localhost", HTTP_PORT), handler)
    httpd.serve_forever()


async def ws_handler(websocket):
    state.clients.add(websocket)
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue
            if data.get("type") == "keys":
                for k in state.keys:
                    if k in data:
                        state.keys[k] = bool(data[k])
            elif data.get("type") == "set_mode":
                new_mode = data.get("mode")
                if new_mode in ("manual", "figure8", "coverage"):
                    if new_mode == "figure8" and state.mode != "figure8":
                        state.fig8_direction = 1
                        state.fig8_half_start = time()
                    if new_mode == "coverage" and state.mode != "coverage":
                        state.cov_phase = "forward"
                        state.cov_phase_start = time()
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

        if state.mode == "figure8":
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
        else:
            speed_target = (1.0 if state.keys["w"] else 0.0) - (1.0 if state.keys["s"] else 0.0)
            speed_target *= state.speed_scale
            steer_target = (1.0 if state.keys["d"] else 0.0) - (1.0 if state.keys["a"] else 0.0)
            steer_target *= state.speed_scale
            state.speed = step_toward(state.speed, speed_target, p["accel_step"])
            state.steer = step_toward(state.steer, steer_target, p["accel_step"])

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
            }
        )

        await asyncio.sleep(LOOP_DELAY)


async def main():
    threading.Thread(target=start_http_server, daemon=True).start()
    print(f"Frontend: http://localhost:{HTTP_PORT}")
    print(f"WebSocket control on ws://localhost:{WS_PORT}")

    state.connect_serial()
    if state.ser is None:
        print(f"Warning: could not open {PORT_SERIAL} yet ({state.serial_error}). Will keep retrying.")

    async with websockets.serve(ws_handler, "localhost", WS_PORT):
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
