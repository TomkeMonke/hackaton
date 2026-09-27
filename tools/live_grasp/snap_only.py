import base64, sys, json
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.camera import RealSenseCamera
from pinecone_bot.detector import HsvConeDetector
cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
arm.bus.connect()
pos = arm.bus.sync_read("Present_Position")
tq = arm.bus.sync_read("Torque_Enable", normalize=False)
arm.bus.disconnect(disable_torque=False)
print("POSE", json.dumps({k: round(v, 1) for k, v in pos.items()}), "torque", sum(tq.values()), file=sys.stderr)
cam = RealSenseCamera(cfg, depth=True, laser_power=360)
try:
    for _ in range(5):
        bgr, depth = cam.read()
finally:
    cam.close()
intr = cam.intrinsics
valid = (depth > 0.1) & (depth < 1.5)
P = xyz_image(depth, intr)
n, d = fit_ground(P, valid)
H = np.where(valid, P @ n + d, -1.0)
print("mean BGR", bgr.reshape(-1, 3).mean(0).round(0), file=sys.stderr)
for c in HsvConeDetector(cfg.detector).detect(bgr):
    print(f"HSV px={c.px:.0f} py={c.py:.0f} area={c.area:.0f}", file=sys.stderr)
    x, y, w, h = c.bbox
    cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(H, 0, 0.06), alpha=255.0 / 0.06), cv2.COLORMAP_JET)
vis[H < 0] = 0
print(base64.b64encode(cv2.imencode(".jpg", np.hstack((cv2.resize(bgr, (480, 360)), cv2.resize(vis, (480, 360)))))[1].tobytes()).decode())
