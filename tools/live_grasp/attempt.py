# Jedna proba zebrania szyszki, samo ramie. Wysylane na Pi przez stdin (python -), bez plikow na Pi.
# Wejscie: STATE (json, podstawiany przez run.sh). Wyjscie na stderr: log, "RESULT ...", "STATE {...}".
# Na stdout: base64 jpg ze zdjecia po podniesieniu.
import base64, json, sys, time
import cv2
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera
from pinecone_bot.detector import HsvConeDetector

S = json.loads(STATE)
HOME = {"shoulder_pan": -5.45, "shoulder_lift": 88.92, "elbow_flex": 7.56, "wrist_flex": -87.87, "wrist_roll": 88.88}
PRE = {"shoulder_lift": 86.64, "elbow_flex": -3.87, "wrist_flex": -87.87, "wrist_roll": 88.97}
GR = {"shoulder_lift": 89.1, "elbow_flex": -13.36, "wrist_flex": -65.54, "wrist_roll": 88.09}
SEARCH_PANS = [-15, -5, -25, 5, -35, 15]
T0 = time.time()


def log(*a):
    print(f"[{time.time() - T0:5.1f}s]", *a, file=sys.stderr, flush=True)


cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
for i in range(3):
    try:
        arm.bus.connect()
        break
    except Exception as e:  # magistrala czasem nie widzi serwa chwytaka przy scisku
        log("connect nieudany, ponawiam:", str(e).splitlines()[0])
        try:
            arm.bus.port_handler.closePort()
        except Exception:
            pass
        time.sleep(1.0)
else:
    raise SystemExit("RESULT BLAD magistrala serw")
cur = arm.bus.sync_read("Present_Position")
arm.bus.sync_write("Goal_Position", cur)
arm.bus.enable_torque()
ctl = WaypointArm(cfg, arm=arm)
cam = RealSenseCamera(cfg, depth=True, laser_power=360)
hsv_det = HsvConeDetector(cfg.detector)
intr = cam.intrinsics


class Det:
    def __init__(self, px, py, area, bbox):
        self.px, self.py, self.area, self.bbox, self.partial = px, py, area, bbox, False


HVIEW = {"H": None}


def depth_cones(depth):
    """Szyszki = wybrzuszenia 1.2-8 cm nad plaszczyzna ziemi (dziala po ciemku: glebia to podczerwien z laserem)."""
    valid = (depth > 0.1) & (depth < 1.5)
    P = xyz_image(depth, intr)
    g = fit_ground(P, valid)
    if g is None:
        return []
    n, d = g
    H = np.where(valid, P @ n + d, 0.0)
    HVIEW["H"] = np.where(valid, H, -1.0)
    low = ((H > 0.012) & (H < 0.08) & valid).astype(np.uint8)
    low = cv2.morphologyEx(low, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, lab, st, cent = cv2.connectedComponentsWithStats(low, connectivity=8)
    out = []
    for i in range(1, num):
        a = st[i, cv2.CC_STAT_AREA]
        if 150 <= a <= 9000:
            out.append(Det(float(cent[i][0]), float(cent[i][1]), float(a), tuple(int(x) for x in st[i, :4])))
    return out


def pose():
    return arm.bus.sync_read("Present_Position")


def go(target, sec, fix=True):
    ctl._move(target, sec)
    time.sleep(0.35)
    if fix:  # regulacja calkujaca: niskie P serw + grawitacja zostawiaja blad, dopisz go do celu (max 4 razy)
        cmd = dict(target)
        for _ in range(4):
            now = pose()
            err = {j: target[j] - now[j] for j in target if j != "gripper"}
            if all(abs(e) <= 0.8 for e in err.values()):
                break
            cmd = {j: (cmd[j] + max(-6.0, min(6.0, err[j])) if j in err and abs(err[j]) > 0.8 else cmd[j]) for j in cmd}
            ctl._move(cmd, 0.4)
            time.sleep(0.3)
    return pose()


def in_jaws(d):
    x0, y0, x1, y1 = S["jaw_box"]
    return x0 <= d.px <= x1 and d.py >= y0 or d.py >= S["body_y"]


def snap():
    for _ in range(4):
        bgr, depth = cam.read()
    dets = [d for d in depth_cones(depth) if not in_jaws(d)]
    # kolor (przy swietle): widzi szyszke tez w cieniu stereo szczeki, gdzie glebia jest slepa
    for c in hsv_det.detect(bgr):
        if c.partial or in_jaws(c) or c.area < 150:
            continue
        if all(abs(c.px - d.px) > 30 or abs(c.py - d.py) > 30 for d in dets):
            dets.append(Det(c.px, c.py, c.area, c.bbox))
    Hv = HVIEW["H"] if HVIEW["H"] is not None else np.zeros(depth.shape)
    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(Hv, 0, 0.06), alpha=255.0 / 0.06), cv2.COLORMAP_JET)
    vis[Hv < 0] = 0  # brak pomiaru (cien stereo, za blisko)
    for d in dets:
        x, y, w, h = d.bbox
        cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 255, 255), 2)
    cv2.drawMarker(vis, (int(S["align_x"]), int(S["align_y"])), (255, 0, 255), cv2.MARKER_CROSS, 30, 2)
    last_img["img"] = np.hstack((cv2.resize(bgr, (480, 360)), cv2.resize(vis, (480, 360))))
    return bgr, dets


def nearest(dets, x, y):
    return min(dets, key=lambda d: (d.px - x) ** 2 + (d.py - y) ** 2) if dets else None


def reachable(d):
    return abs(d.py - S["align_y"]) <= S["reach_py"]


result = "BRAK"
img = None
last_img = {"img": None}
try:
    pan = pose()["shoulder_pan"]
    go({**PRE, "shoulder_pan": pan, "gripper": 100.0}, 1.5)
    # 1. szukanie
    target = None
    for p in [None] + SEARCH_PANS:
        if p is not None:
            pan = go({**PRE, "shoulder_pan": float(p), "gripper": 100.0}, 1.0)["shoulder_pan"]
        bgr, dets = snap()
        # podstawa ma w serwie zakres tylko +-23 st: bierz szyszki, do ktorych dosiegnie
        ok = [d for d in dets if reachable(d)]
        log(f"pan {pan:.1f}: szyszki {[(round(d.px), round(d.py)) for d in dets]}, w zasiegu {len(ok)}")
        if ok:
            target = nearest(ok, S["align_x"], S["align_y"])
            break
    if target is None:
        result = "BRAK szyszki w zasiegu"
        raise StopIteration
    # 2. celowanie: podstawa (pan + -> szyszka w lewo, k px/st) i bark (lift - -> szyszka w dol, ky px/st)
    tx, ty = target.px, target.py
    bw, bh = target.bbox[2], target.bbox[3]
    lift = pose()["shoulder_lift"]
    for it in range(10):
        ex, ey = tx - S["align_x"], ty - S["align_y"]
        log(f"celowanie {it}: szyszka ({tx:.0f},{ty:.0f}) blad x {ex:+.0f} y {ey:+.0f}, k={S['k']:.1f} ky={S['ky']:.1f}")
        if abs(ex) <= S["tol_px"] and abs(ey) <= S["tol_py"]:
            break
        dpan = max(-12.0, min(12.0, ex / S["k"])) if abs(ex) > S["tol_px"] else 0.0
        dlift = max(-6.0, min(6.0, ey / S["ky"])) if abs(ey) > S["tol_py"] else 0.0
        new_lift = max(S["lift_min"], min(S["lift_max"], lift + dlift))
        before = (pan, lift)
        now = go({**PRE, "shoulder_pan": pan + dpan, "shoulder_lift": new_lift, "gripper": 100.0}, 0.8)
        pan, lift = now["shoulder_pan"], now["shoulder_lift"]
        mp, ml = pan - before[0], lift - before[1]
        bgr, dets = snap()
        px_pred, py_pred = tx - S["k"] * mp, ty - S["ky"] * ml
        seen = nearest(dets, px_pred, py_pred)
        if seen is None or abs(seen.px - px_pred) > 80 or abs(seen.py - py_pred) > 80:
            # zgubiona (pod szczeka): nie zgaduj, cofnij pol kroku i zmierz jeszcze raz
            log(f"  szyszka zgubiona (przewidziane {px_pred:.0f},{py_pred:.0f}) - cofam pol kroku")
            now = go({**PRE, "shoulder_pan": pan - mp / 2, "shoulder_lift": lift - ml / 2, "gripper": 100.0}, 0.8)
            pan, lift = now["shoulder_pan"], now["shoulder_lift"]
            bgr, dets = snap()
            seen = nearest(dets, tx - S["k"] * (pan - before[0]), ty - S["ky"] * (lift - before[1]))
            if seen is None:
                raise_lost = True
                log("  nadal nie widac szyszki - przerywam bez chwytu")
                result = "ZGUBIONA"
                raise StopIteration
            tx, ty = seen.px, seen.py
            bw, bh = seen.bbox[2], seen.bbox[3]
            continue
        # uczenie wspolczynnikow z obserwacji (tylko os, ktora sie wyraznie ruszyla)
        if abs(mp) >= 1.0 and abs(ml) < 0.5:
            k_obs = -(seen.px - tx) / mp
            if 3.0 < k_obs < 40.0:
                S["k"] = round(0.6 * S["k"] + 0.4 * k_obs, 2)
        if abs(ml) >= 1.0 and abs(mp) < 0.5:
            ky_obs = -(seen.py - ty) / ml
            if 1.5 < ky_obs < 30.0:
                S["ky"] = round(0.6 * S["ky"] + 0.4 * ky_obs, 2)
        tx, ty = seen.px, seen.py
        bw, bh = seen.bbox[2], seen.bbox[3]
    else:
        result = "NIE WYCELOWANO"
        log("  10 krokow bez trafienia w punkt celowania - bez chwytu")
        raise StopIteration
    dl = lift - PRE["shoulder_lift"]
    aligned = (round(tx), round(ty))
    # 3. chwyt
    # szyszka lezy w poprzek (szersza niz wyzsza na obrazie) -> obroc szczeki o 90 st, zeby zlapac ja przez srednice
    roll_off = S["roll_turn"] if bw > S["wide_ratio"] * bh else 0.0
    grasp = {**GR, "shoulder_pan": pan}
    grasp["wrist_roll"] += roll_off
    grasp["shoulder_lift"] = max(S["lift_min"], min(S["lift_max"], GR["shoulder_lift"] + dl))
    grasp["elbow_flex"] += S["goff_elbow"]
    grasp["wrist_flex"] += S["goff_wrist"]
    if roll_off:
        log(f"szyszka w poprzek (bbox {bw}x{bh}) -> obracam chwytak o {roll_off:+.0f} st")
        go({**PRE, "shoulder_pan": pan, "shoulder_lift": lift, "wrist_roll": PRE["wrist_roll"] + roll_off, "gripper": 100.0}, 1.2)
    log(f"schodze: pan {pan:.1f} lift {grasp['shoulder_lift']:.1f}, glebokosc elbow {S['goff_elbow']:+.1f} wrist {S['goff_wrist']:+.1f}")
    go({**grasp, "gripper": 100.0}, 1.5)
    ctl._move({"gripper": 0.0}, 1.0)
    time.sleep(0.4)
    g = pose()["gripper"]
    held = g >= cfg.arm.empty_gripper_below
    log(f"chwytak {g:.1f} -> {'TRZYMA' if held else 'PUSTY'}")
    hold = max(g - 3.0, 0.0) if held else 0.0
    ctl._move({"gripper": hold}, 0.3)  # mniejszy scisk: przy celu 0 serwo chwytaka potrafi zniknac z magistrali
    go({**PRE, "shoulder_pan": pan, "shoulder_lift": lift, "wrist_roll": PRE["wrist_roll"] + roll_off, "gripper": hold}, 1.5)
    img, dets = snap()
    left = nearest(dets, *aligned)
    shift = None if left is None or abs(left.px - aligned[0]) > 120 or abs(left.py - aligned[1]) > 120         else (round(left.px - aligned[0]), round(left.py - aligned[1]))
    S["tries"] += 1
    if held:
        S["ok"] += 1
        result = "MAM SZYSZKE"
    else:
        result = "PUSTO"
        log(f"po probie szyszka przesunieta o {shift} px (None = nie widac)")
    S["log"] = (S["log"] + [{"pan": round(pan, 1), "lift": round(lift, 1), "aligned": aligned, "roll": roll_off, "bbox": [bw, bh], "gripper": round(g, 1), "shift": shift if not held else None, "result": result}])[-20:]
except StopIteration:
    pass
finally:
    try:
        g = pose()["gripper"]
        go({**HOME, "gripper": g}, 2.0)
        if result == "MAM SZYSZKE":
            log("HOME z szyszka - zabierz ja z chwytaka (4 s), potem otwieram")
            time.sleep(4.0)
        ctl._move({"gripper": 100.0}, 0.8)
    finally:
        arm.bus.disconnect(disable_torque=False)
        cam.close()
log("RESULT", result)
print("STATE " + json.dumps(S), file=sys.stderr)
if last_img["img"] is not None:
    print(base64.b64encode(cv2.imencode(".jpg", last_img["img"])[1].tobytes()).decode())
