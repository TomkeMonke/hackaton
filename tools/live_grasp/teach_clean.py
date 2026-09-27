# Petla uczenia (doklejane za percept.py, uruchamiane na Pi przez nohup, zapis do /tmp/teach.jsonl).
# Cykl: HOME (torque) -> zdjecie glebi szyszek -> chwytak otwarty szeroko = "twoja kolej" -> torque off
# -> czlowiek chwyta reka (zamkniecie chwytaka = zapis pozy chwytu) -> wrzuca do sloika, otwiera
# -> po 5 s robot wraca do HOME i widzi, ktorej szyszki brakuje.
import base64, json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera

MAX_SAMPLES, MAX_SECONDS = 12, 1200
HOME = {"shoulder_pan": -5.45, "shoulder_lift": 88.3, "elbow_flex": 7.56, "wrist_flex": -87.87, "wrist_roll": 88.88}
J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
OUT = open("/tmp/teach.jsonl", "a")
T0 = time.time()


def emit(kind, **kw):
    OUT.write(json.dumps({"kind": kind, "t": round(time.time() - T0, 2), **kw}) + "\n")
    OUT.flush()


def log(*a):
    print(f"[{time.time() - T0:6.1f}s]", *a, file=sys.stderr, flush=True)


cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
arm.bus.connect()
ctl = WaypointArm(cfg, arm=arm)
cam = RealSenseCamera(cfg, depth=True, laser_power=360)
intr = cam.intrinsics


def pose():
    return arm.bus.sync_read("Present_Position")


def torque_on_here():
    cur = pose()
    arm.bus.sync_write("Goal_Position", cur)
    arm.bus.enable_torque()
    ctl._last_cmd = dict(cur)


def go_home():
    ctl._move({**HOME, "gripper": 30.0}, 3.0)
    time.sleep(0.3)
    cmd = dict(HOME)
    for _ in range(4):
        now = pose()
        err = {j: HOME[j] - now[j] for j in HOME}
        if all(abs(e) <= 0.6 for e in err.values()):
            break
        cmd = {j: cmd[j] + max(-5.0, min(5.0, err[j])) if abs(err[j]) > 0.6 else cmd[j] for j in cmd}
        ctl._move({**cmd, "gripper": 30.0}, 0.5)
        time.sleep(0.25)
    return pose()


def home_snapshot():
    """Chwytak w gore z kadru nie jest potrzebny: w HOME szczeki sa na dole obrazu (v > 390 odrzucane)."""
    for _ in range(6):
        bgr, depth = cam.read()
    valid = (depth > 0.1) & (depth < 1.5)
    P = xyz_image(depth, intr)
    g = fit_ground(P, valid)
    if g is None:
        return [], bgr, None
    n, d = g
    H = np.where(valid, P @ n + d, 0.0)
    # chwytak w HOME: niebieski PLA (przy swietle) + wszystko wyzej niz 8 cm, powiekszone; reszta kadru (takze dol) zostaje
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    grip = (cv2.inRange(hsv, (88, 100, 90), (120, 255, 255)) > 0) | ((H > 0.08) & valid)
    grip = cv2.dilate(grip.astype(np.uint8), np.ones((15, 15), np.uint8)) > 0
    tall = ((H > 0.08) & valid & ~(cv2.inRange(hsv, (88, 100, 90), (120, 255, 255)) > 0)).astype(np.uint8)
    tall = cv2.morphologyEx(tall, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    HAND["seen"] = int(tall[:300].sum()) > 3000  # wysoko nad ziemia, nie niebieskie, w gornej czesci kadru = reka
    low = ((H > 0.012) & (H < 0.08) & valid & ~grip).astype(np.uint8)
    low = cv2.morphologyEx(low, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, lab, st, cent = cv2.connectedComponentsWithStats(low, connectivity=8)
    cones = []
    for i in range(1, num):
        a = int(st[i, cv2.CC_STAT_AREA])
        if a < 250 or a > 9000 or (cent[i][1] > 380 and 330 < cent[i][0] < 580):  # kabel serwa pod chwytakiem
            continue
        m = lab == i
        cones.append({"uv": [round(float(cent[i][0]), 1), round(float(cent[i][1]), 1)],
                      "p": [round(float(x), 4) for x in P[m].mean(0)], "h": round(float(H[m].max()), 4), "area": a})
    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(np.where(valid, H, 0), 0, 0.06), alpha=255 / 0.06), cv2.COLORMAP_JET)
    vis[grip] = (vis[grip] * 0.4).astype(np.uint8)
    for c in cones:
        cv2.circle(vis, (int(c["uv"][0]), int(c["uv"][1])), 14, (255, 255, 255), 2)
    img = np.hstack((cv2.resize(bgr, (320, 240)), cv2.resize(vis, (320, 240))))
    return cones, img, [round(float(x), 4) for x in n] + [round(float(d), 4)]


HAND = {"seen": False}
samples = 0
try:
    while samples < MAX_SAMPLES and time.time() - T0 < MAX_SECONDS:
        # 1. HOME i zdjecie: czekaj na dokladnie 1 szyszke i brak reki, 2 zgodne zdjecia pod rzad
        torque_on_here()
        hp = go_home()
        log("HOME, blad serw: " + str({j[:5]: round(hp[j] - HOME[j], 1) for j in HOME}))
        prev, t_ask = None, 0.0
        while time.time() - T0 < MAX_SECONDS:
            cones, img, ground = home_snapshot()
            ok = len(cones) == 1 and not HAND["seen"]
            if ok and prev is not None and np.linalg.norm(np.subtract(cones[0]["p"], prev)) < 0.01:
                break
            prev = cones[0]["p"] if ok else None
            if time.time() - t_ask > 10:
                log(f"czekam: szyszek {len(cones)}, reka {HAND['seen']} - poloz JEDNA szyszke, rece poza kadrem")
                t_ask = time.time()
            time.sleep(0.7)
        emit("home", cones=cones, ground=ground, joints={j: round(hp[j], 2) for j in HOME},
             img=base64.b64encode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()).decode())
        log(f"HOME: 1 szyszka uv {cones[0]['uv']}")
        # 2. sygnal: chwytak szeroko otwarty, torque off
        ctl._move({"gripper": 100.0}, 0.6)
        time.sleep(0.3)
        arm.bus.disable_torque()
        log("TWOJA KOLEJ: chwyc szyszke")
        # 3. czekaj na zacisk (spadek ponizej 12.5), potem na puszczenie (> 16)
        traj, state, q_grasp = [], "open", None
        t_wait = time.time()
        while time.time() - T0 < MAX_SECONDS:
            try:
                q = pose()
            except Exception:
                continue
            traj.append({"t": round(time.time() - T0, 2), **{j: round(q[j], 2) for j in J}})
            away = max(abs(q[j] - HOME[j]) for j in ("shoulder_pan", "elbow_flex", "wrist_flex")) > 5.0
            closing = q["gripper"] < 25.0 and time.time() - t_wait > 1.0 and away
            if state == "open" and closing:
                # zacisk = chwytak < 25 i stabilny (+-2) przez 0.5 s (szyszka w szczekach, reka juz nie domyka)
                recent = [p["gripper"] for p in traj if p["t"] >= traj[-1]["t"] - 0.5]
                if len(recent) >= 10 and max(recent) - min(recent) < 2.0 and max(recent) < 25.0:
                    q_grasp = {j: round(q[j], 2) for j in J}
                    state = "closed"
                    log(f"ZACISK: {q_grasp}")
            elif state == "closed" and q["gripper"] > q_grasp["gripper"] + 6.0:
                log("PUSZCZONA - puszczaj ramie, za 5 s wracam do HOME")
                break
            time.sleep(0.033)
        if q_grasp is None:
            break
        emit("grasp", q=q_grasp, traj=traj[-240:])
        samples += 1
        time.sleep(5.0)
finally:
    try:
        torque_on_here()
        ctl._move({**HOME, "gripper": 30.0}, 3.0)
        cones, img, ground = home_snapshot()
        emit("home", cones=cones, ground=ground,
             img=base64.b64encode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()).decode())
    finally:
        arm.bus.disconnect(disable_torque=False)
        cam.close()
        log(f"KONIEC, probek {samples}")
