# Zbieranie z modelu z uczenia (doklejane za percept.py; MODEL = json z fit.py).
# HOME -> zdjecie glebi -> szyszka -> stawy chwytu z modelu -> podejscie -> zacisk -> sloik (sciezka z nagrania) -> HOME.
import base64, json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera

M = json.loads(MODEL)
J4 = M["joints"]
HOME = {"shoulder_pan": -5.45, "shoulder_lift": 88.3, "elbow_flex": 7.56, "wrist_flex": -87.87, "wrist_roll": 88.88}
# sciezka do sloika z nagrania (cykl 61.9-70 s), bez podstawy na poczatku (zostaje z chwytu)
JAR_UP = [  # (pan albo None = jak przy chwycie, lift, elbow, wrist, sekundy)
    (None, 74.4, -63.3, -72.8, 1.2), (None, 36.7, -77.5, -72.9, 1.2), (None, -35.3, -78.8, -73.7, 1.6),
    (-18.1, -38.5, -98.8, -74.0, 1.2), (-19.0, -57.8, -97.8, -84.7, 1.0), (-22.2, -60.8, -97.7, -84.6, 1.0)]
JAR_BACK = [(-21.8, -59.7, -76.0, -80.3, 1.0), (-21.9, -45.1, -33.3, -80.3, 1.0), (-21.8, -29.8, -3.1, -80.3, 1.0),
            (-13.2, 60.1, 17.8, -80.2, 2.0)]
T0 = time.time()


def log(*a):
    print(f"[{time.time() - T0:5.1f}s]", *a, file=sys.stderr, flush=True)


cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
arm.bus.connect()
cur = arm.bus.sync_read("Present_Position")
arm.bus.sync_write("Goal_Position", cur)
arm.bus.enable_torque()
ctl = WaypointArm(cfg, arm=arm)
cam = RealSenseCamera(cfg, depth=True, laser_power=360)
intr = cam.intrinsics


def pose():
    return arm.bus.sync_read("Present_Position")


def go(target, sec):
    ctl._move(target, sec)
    time.sleep(0.25)
    cmd = dict(target)
    for _ in range(3):  # korekta calkujaca bledu serw
        now = pose()
        err = {j: target[j] - now[j] for j in target if j != "gripper"}
        if all(abs(e) <= 0.8 for e in err.values()):
            break
        cmd = {j: (cmd[j] + max(-6.0, min(6.0, err[j])) if j in err and abs(err[j]) > 0.8 else cmd[j]) for j in cmd}
        ctl._move(cmd, 0.4)
        time.sleep(0.2)
    return pose()


def feats(p):
    x, z = p[0], p[2]
    f = [1.0, x, z] + ([x * x, z * z, x * z] if M["quad"] else [])
    return np.array(f)


def snapshot():
    for _ in range(6):
        bgr, depth = cam.read()
    valid = (depth > 0.1) & (depth < 1.5)
    P = xyz_image(depth, intr)
    n, d = fit_ground(P, valid)
    H = np.where(valid, P @ n + d, 0.0)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    grip = (cv2.inRange(hsv, (88, 100, 90), (120, 255, 255)) > 0) | ((H > 0.08) & valid)
    grip = cv2.dilate(grip.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    low = ((H > 0.012) & (H < 0.08) & valid & ~grip).astype(np.uint8)
    low = cv2.morphologyEx(low, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, lab, st, cent = cv2.connectedComponentsWithStats(low, connectivity=8)
    cones = []
    for i in range(1, num):
        a = int(st[i, cv2.CC_STAT_AREA])
        if 250 <= a <= 9000 and not (cent[i][1] > 380 and 330 < cent[i][0] < 580):  # kabel serwa pod chwytakiem
            cones.append({"uv": (float(cent[i][0]), float(cent[i][1])), "p": P[lab == i].mean(0)})
    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(H, 0, 0.06), alpha=255 / 0.06), cv2.COLORMAP_JET)
    return cones, bgr, vis



N_TRIES = 10
bias = np.zeros(3)      # poprawka celu w ukladzie kamery HOME [m], uczona z przepchniec
wrist_extra = 0.0       # glebiej, gdy szyszka nieruszona
sheet, result = [], "NIE ZLAPANA"
qmin = np.array(M["q_min"]) - 8.0
qmax = np.array(M["q_max"]) + 8.0


def cone_near(cones, p, max_d=0.12):
    c = min(cones, key=lambda c: np.linalg.norm(c["p"] - p), default=None)
    return c if c is not None and np.linalg.norm(c["p"] - p) < max_d else None


try:
    go({**HOME, "gripper": 30.0}, 3.0)
    cones, bgr, vis = snapshot()
    target = max(cones, key=lambda c: c["p"][0] * 0 - c["p"][2]) if cones else None  # najblizsza kamerze
    if target is None:
        raise StopIteration
    p_track = target["p"]
    for k in range(N_TRIES):
        c = cone_near(cones, p_track)
        if c is None:
            result = "SZYSZKA ZNIKNELA Z KADRU"
            break
        p_track = c["p"]
        aim = c["p"] + bias
        q = np.clip(feats(aim) @ np.array(M["W"]), qmin, qmax)
        grasp = {j: float(q[i]) for i, j in enumerate(J4)}
        grasp["wrist_flex"] += wrist_extra
        grasp["wrist_roll"] = M["wrist_roll"]
        pre = {j: grasp[j] + M["approach_delta"][i] for i, j in enumerate(J4)}
        pre["wrist_roll"] = M["wrist_roll"]
        log(f"proba {k + 1}: szyszka p {np.round(c['p'], 3)} poprawka {np.round(bias * 100, 1)} cm, wrist+{wrist_extra:.0f} -> "
            f"{({j: round(v, 1) for j, v in grasp.items()})}")
        im = bgr.copy()
        cv2.circle(im, (int(c["uv"][0]), int(c["uv"][1])), 16, (255, 255, 255), 2)
        cv2.putText(im, f"proba {k + 1}", (8, 30), 0, 1.0, (0, 255, 255), 2)
        sheet.append(cv2.resize(im, (320, 240)))
        go({**pre, "gripper": 100.0}, 2.0)
        go({**grasp, "gripper": 100.0}, 1.5)
        ctl._move({"gripper": 0.0}, 1.0)
        time.sleep(0.4)
        g = pose()["gripper"]
        held = g >= cfg.arm.empty_gripper_below
        log(f"  zacisk: chwytak {g:.1f} -> {'TRZYMA' if held else 'PUSTY'}")
        if held:
            hold = max(g - 4.0, 0.0)
            ctl._move({"gripper": hold}, 0.3)
            for pan, lift, elbow, wrist, sec in JAR_UP:
                go({"shoulder_pan": grasp["shoulder_pan"] if pan is None else pan, "shoulder_lift": lift,
                    "elbow_flex": elbow, "wrist_flex": wrist, "wrist_roll": M["wrist_roll"], "gripper": hold}, sec * 1.5)
            ctl._move({"gripper": 100.0}, 0.8)
            time.sleep(0.5)
            for pan, lift, elbow, wrist, sec in JAR_BACK:
                go({"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist,
                    "wrist_roll": M["wrist_roll"], "gripper": 60.0}, sec * 1.5)
            result = f"ZLAPANA w probie {k + 1}, do sloika"
            break
        go({**pre, "gripper": 100.0}, 1.2)
        go({**HOME, "gripper": 30.0}, 2.0)
        cones, bgr, vis = snapshot()
        after = cone_near(cones, p_track)
        if after is None:
            result = "SZYSZKA ZNIKNELA Z KADRU"
            break
        push = after["p"] - p_track
        if np.linalg.norm(push) > 0.01:
            bias = bias + 0.7 * push
            log(f"  przepchnieta o {np.round(push * 100, 1)} cm -> poprawka {np.round(bias * 100, 1)} cm")
        else:
            wrist_extra = min(wrist_extra + 3.0, 12.0)
            log(f"  nieruszona -> glebiej, wrist +{wrist_extra:.0f}")
        p_track = after["p"]
except StopIteration:
    pass
finally:
    try:
        go({**HOME, "gripper": 30.0}, 2.5)
    finally:
        arm.bus.disconnect(disable_torque=False)
        cam.close()
log("RESULT", result)
if sheet:
    while len(sheet) % 2:
        sheet.append(np.zeros_like(sheet[0]))
    img = np.vstack([np.hstack(sheet[i:i + 2]) for i in range(0, len(sheet), 2)])
    print(base64.b64encode(cv2.imencode(".jpg", img)[1].tobytes()).decode())
