# Nagranie ruchu reka: torque off, pozycje 30 Hz + zdjecie co 0.5 s. Wyjscie na stdout (linie POS/IMG).
import base64, json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.camera import RealSenseCamera
DUR = float(__import__("os").environ.get("REC_SECONDS", "120"))
cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
arm.bus.connect()
arm.bus.disable_torque()
cam = RealSenseCamera(cfg, depth=True, laser_power=360)
intr = cam.intrinsics
print("START", time.time(), file=sys.stderr, flush=True)
t0 = time.time(); i = 0
try:
    while time.time() - t0 < DUR:
        bgr, depth = cam.read()
        try:
            pos = arm.bus.sync_read("Present_Position")
        except Exception:
            continue
        t = time.time() - t0
        print("POS " + json.dumps({"t": round(t, 3), **{k: round(v, 2) for k, v in pos.items()}}), flush=True)
        if i % 15 == 0:
            valid = (depth > 0.1) & (depth < 1.5)
            g = fit_ground(xyz_image(depth, intr), valid)
            if g is not None:
                H = np.where(valid, xyz_image(depth, intr) @ g[0] + g[1], -1.0)
                vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(H, 0, 0.06), alpha=255.0 / 0.06), cv2.COLORMAP_JET)
                vis[H < 0] = 0
            else:
                vis = np.zeros_like(bgr)
            img = np.hstack((cv2.resize(bgr, (320, 240)), cv2.resize(vis, (320, 240))))
            cv2.putText(img, f"{t:.1f}s", (6, 20), 0, 0.6, (255, 255, 255), 2)
            print("IMG %.2f %s" % (t, base64.b64encode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()).decode()), flush=True)
        i += 1
finally:
    cam.close()
    arm.bus.disconnect(disable_torque=False)
    print("KONIEC", file=sys.stderr, flush=True)
