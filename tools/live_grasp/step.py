# Wysylany na Pi przez stdin (python -). ARGS podstawiane przez step.sh.
import base64, json, sys, time, cv2
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera
from pinecone_bot.detector import HsvConeDetector
A = json.loads(ARGS)
cfg = Config.load()
cfg.arm.port = "/dev/robot-arm"
arm = ac.make_arm(port=cfg.arm.port, max_relative_target=None)
arm.bus.connect()
cur = arm.bus.sync_read("Present_Position")
arm.bus.sync_write("Goal_Position", cur)
arm.bus.enable_torque()
ctl = WaypointArm(cfg, arm=arm)
target = dict(A.get("abs", {}))
for j, d in A.get("rel", {}).items():
    target[j] = cur[j] + d
if target:
    ctl._move(target, float(A.get("sec", 1.0)))
    time.sleep(0.4)
pose = arm.bus.sync_read("Present_Position")
arm.bus.disconnect(disable_torque=False)
print("POSE " + json.dumps({k: round(v, 1) for k, v in pose.items()}), file=sys.stderr)
if A.get("snap", True):
    cam = RealSenseCamera(cfg, depth=False)
    try:
        for _ in range(3):
            bgr, _ = cam.read()
    finally:
        cam.close()
    dets = HsvConeDetector(cfg.detector).detect(bgr)
    for d in dets:
        print(f"DET px={d.px:.0f} py={d.py:.0f} area={d.area:.0f} bbox={d.bbox} partial={d.partial}", file=sys.stderr)
    img = HsvConeDetector.draw_debug(bgr, dets)
    cv2.drawMarker(img, (421, 336), (255, 0, 255), cv2.MARKER_CROSS, 30, 2)
    print(base64.b64encode(cv2.imencode(".jpg", img)[1].tobytes()).decode())
