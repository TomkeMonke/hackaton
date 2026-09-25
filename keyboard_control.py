"""
Drive the hoverboard via the Xiao's simple ASCII serial interface
(xiao_send_pwm.ino), using the keyboard instead of a gamepad.

Controls:
  W / S -> forward / backward (speed)
  A / D -> steer left / right
  release keys -> speed/steer decay back to 0
  Q or Ctrl+C -> quit (zeroes PWM first)

Sends lines like "a100 b20\n" over USB serial:
  a<value> -> speed/pwm
  b<value> -> steer/turn
"""

import os
import sys
from time import sleep, time

import keyboard
import serial

PORT = os.environ.get("ROBOT_DRIVE_PORT", "COM9")
BAUDRATE = 115200

MAX_PWM = 500
MAX_STEER = 400

# Fraction of max value added/removed per loop tick while a key is held/released.
ACCEL_STEP = 0.06

LOOP_DELAY = 0.03  # matches the Arduino's loop delay / well under its 500ms timeout

PRINT_EVERY = 0.2  # seconds between status prints, so the loop isn't spammed


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

    print("Controls: W/S speed, A/D steer, Q or Ctrl+C to quit")

    speed = 0.0  # -1.0..1.0
    steer = 0.0  # -1.0..1.0

    last_print = 0.0
    last_cmd = (None, None)

    try:
        while True:
            if keyboard.is_pressed("q"):
                break

            speed_target = 0.0
            if keyboard.is_pressed("w"):
                speed_target += 1.0
            if keyboard.is_pressed("s"):
                speed_target -= 1.0

            steer_target = 0.0
            if keyboard.is_pressed("d"):
                steer_target += 1.0
            if keyboard.is_pressed("a"):
                steer_target -= 1.0

            speed = step_toward(speed, speed_target, ACCEL_STEP)
            steer = step_toward(steer, steer_target, ACCEL_STEP)

            pwm_speed = int(speed * MAX_PWM)
            pwm_steer = int(steer * MAX_STEER)

            ser.write(f"a{pwm_speed} b{pwm_steer}\n".encode("ascii"))

            now = time()
            if (pwm_speed, pwm_steer) != last_cmd and now - last_print >= PRINT_EVERY:
                print(f"speed={speed:+.2f} steer={steer:+.2f} -> a{pwm_speed} b{pwm_steer}")
                last_print = now
                last_cmd = (pwm_speed, pwm_steer)

            sleep(LOOP_DELAY)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print("\nStopping: zeroing PWM")
        ser.write(b"a0 b0\n")
        sleep(0.1)
        ser.close()


if __name__ == "__main__":
    main()
