"""
Drives the hoverboard in a figure-eight pattern, autonomously.

Loops: constant forward speed while steering hard one way for
LOOP_SECONDS (traces one circle), then switches steer direction for
the same duration (traces the other circle) -> figure eight.

Ctrl+C stops immediately and zeroes PWM.
"""

import sys
from time import sleep, time

import serial

PORT = "COM9"
BAUDRATE = 115200

SPEED_FRAC = 0.06  # forward speed as a fraction of MAX_PWM, constant throughout
STEER_FRAC = 0.025 # steer magnitude as a fraction of MAX_STEER while turning
# steer must stay well below speed, otherwise one wheel goes negative and
# the robot pivots in place instead of arcing into a circle.
RAMP_STEP = 0.02   # per-tick ramp for speed/steer, avoids a jolt at the start

MAX_PWM = 500
MAX_STEER = 400

LOOP_SECONDS = 10.0  # time spent tracing each half of the eight
SEND_DELAY = 0.03   # matches the Arduino's loop delay / well under its 500ms timeout


def step_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def main():
    print(f"Connecting to Xiao on {PORT} @ {BAUDRATE} baud...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0)
    except serial.SerialException:
        print(f"Could not open {PORT}. Is the Arduino plugged in and is {PORT} the right port?")
        sys.exit(1)
    sleep(2)  # let the board reset after opening the serial port

    print("Driving figure eight. Ctrl+C to stop.")

    speed = 0.0
    steer = 0.0
    direction = 1  # +1 = steer right first, -1 = steer left first
    half_start = time()

    try:
        while True:
            elapsed = time() - half_start
            if elapsed >= LOOP_SECONDS:
                direction *= -1
                half_start = time()

            speed = step_toward(speed, SPEED_FRAC, RAMP_STEP)
            steer = step_toward(steer, direction * STEER_FRAC, RAMP_STEP)

            pwm_speed = int(speed * MAX_PWM)
            pwm_steer = int(steer * MAX_STEER)

            ser.write(f"a{pwm_speed} b{pwm_steer}\n".encode("ascii"))
            sleep(SEND_DELAY)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print("\nStopping: zeroing PWM")
        ser.write(b"a0 b0\n")
        sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()
