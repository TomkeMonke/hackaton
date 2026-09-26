# Odtworzenie nagranego cyklu (CYCLE = lista pozycji z czasem), tempo / SLOW. Chwytak: przy zacisku (odczyt < 12.5) cel = odczyt - 6.
import json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
C = json.loads(CYCLE)
SLOW = 2.0
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
arm.bus.connect()
cur = arm.bus.sync_read("Present_Position")
arm.bus.sync_write("Goal_Position", cur)
arm.bus.enable_torque()
ctl = WaypointArm(cfg, arm=arm)
def cmd(p):
    q = {j: p[j] for j in J}
    if q["gripper"] < 12.5:
        q["gripper"] = max(0.0, q["gripper"] - 6.0)
    return q
try:
    print("dojazd do startu cyklu (3 s)", file=sys.stderr, flush=True)
    ctl._move(cmd(C[0]), 3.0)
    t0 = time.time()
    for p in C:
        target_t = (p["t"] - C[0]["t"]) * SLOW
        wait = t0 + target_t - time.time()
        if wait > 0:
            time.sleep(wait)
        arm.send_action({f"{j}.pos": v for j, v in cmd(p).items()})
        if abs(p["t"] - 61.9) < 0.05 or abs(p["t"] - 66.5) < 0.05:
            now = arm.bus.sync_read("Present_Position")
            print(f"t {p['t']:.1f}: chwytak {now['gripper']:.1f}", file=sys.stderr, flush=True)
    time.sleep(0.8)
    now = arm.bus.sync_read("Present_Position")
    print("koniec:", {k: round(v, 1) for k, v in now.items()}, file=sys.stderr)
finally:
    arm.bus.disconnect(disable_torque=False)
