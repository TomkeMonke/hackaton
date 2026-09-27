"""Surowa komenda do Xiao na zadany czas: python raw_drive.py SPEED STEER SECONDS"""
import sys, time, serial
speed, steer, secs = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
line = f"a{speed} b{steer}\n".encode()
with serial.Serial("/dev/robot-drive", 115200, timeout=0, write_timeout=0.5) as s:
    time.sleep(0.3)
    print(f"a{speed} b{steer} przez {secs} s", flush=True)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < secs:
            s.write(line); time.sleep(0.05)
    finally:
        for _ in range(5): s.write(b"a0 b0\n"); time.sleep(0.02)
        print("stop", flush=True)
