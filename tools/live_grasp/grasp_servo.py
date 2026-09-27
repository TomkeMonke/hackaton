# Proby chwytu z uczeniem po kazdej probie (doklejane za percept.py; MODEL, STATE podstawia runner).
# 1) HOME, glebia -> szyszka -> poza chwytu z modelu + nauczona poprawka dq_bias.
# 2) w pozie chwytu (szczeki otwarte) zdjecie: gdzie szyszka wzgledem srodka miedzy koncowkami szczek.
# 3) korekta podstawa (bok) i bark (odleglosc), 1 s ruchu + zdjecie, jakobian 2x2 uczony (Broyden).
# 4) zacisk; pusto -> uczenie: glebokosc nadgarstka / punkt docelowy miedzy szczekami. Stan wraca jako STATE.
import base64, json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera

M = json.loads(MODEL)
S = json.loads(STATE)
J4 = M["joints"]
HOME = {"shoulder_pan": -5.45, "shoulder_lift": 88.3, "elbow_flex": 7.56, "wrist_flex": -87.87, "wrist_roll": 88.88}
LIM = {"shoulder_pan": (-22.5, 22.5), "shoulder_lift": (55.0, 88.3), "elbow_flex": (-40.0, 35.0), "wrist_flex": (-100.0, -10.0)}
JAR_UP = [(None, 74.4, -63.3, -72.8, 1.2), (None, 36.7, -77.5, -72.9, 1.2), (None, -35.3, -78.8, -73.7, 1.6),
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


def clampq(q):
    return {j: (max(LIM[j][0], min(LIM[j][1], v)) if j in LIM else v) for j, v in q.items()}


def go(target, sec):
    target = clampq(target)
    ctl._move(target, sec)
    time.sleep(0.25)
    cmd = dict(target)
    for _ in range(3):
        now = pose()
        err = {j: target[j] - now[j] for j in target if j != "gripper"}
        if all(abs(e) <= 0.8 for e in err.values()):
            break
        cmd = {j: (cmd[j] + max(-6.0, min(6.0, err[j])) if j in err and abs(err[j]) > 0.8 else cmd[j]) for j in cmd}
        ctl._move(cmd, 0.4)
        time.sleep(0.2)
    return pose()


def frames():
    for _ in range(5):
        bgr, depth = cam.read()
    return bgr, depth


def blue_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (88, 100, 90), (120, 255, 255))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)) > 0


def cones_in(bgr, depth, exclude):
    valid = (depth > 0.1) & (depth < 1.5)
    P = xyz_image(depth, intr)
    g = fit_ground(P, valid)
    if g is None:
        return [], None
    H = np.where(valid, P @ g[0] + g[1], 0.0)
    ex = cv2.dilate(exclude.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    low = ((H > 0.012) & (H < 0.08) & valid & ~ex & ~((H > 0.08) & valid)).astype(np.uint8)
    low = cv2.morphologyEx(low, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, lab, st, cent = cv2.connectedComponentsWithStats(low, connectivity=8)
    out = []
    for i in range(1, num):
        a = int(st[i, cv2.CC_STAT_AREA])
        if 200 <= a <= 12000 and not (cent[i][1] > 380 and 330 < cent[i][0] < 580):
            out.append({"uv": np.array([cent[i][0], cent[i][1]], float), "p": P[lab == i].mean(0), "area": a})
    return out, H


def jaw_target(bgr):
    """Cel = koncowka widocznej (ruchomej) szczeki + nauczone przesuniecie. W pozie chwytu kamera widzi jedna szczeke."""
    bm = blue_mask(bgr)
    num, lab, st, _ = cv2.connectedComponentsWithStats(bm.astype(np.uint8), connectivity=8)
    comps = [i for i in range(1, num) if st[i, cv2.CC_STAT_AREA] > 800]
    if not comps:
        return None, bm, []
    i = min(comps, key=lambda i: st[i, cv2.CC_STAT_TOP])  # najwyzej siegajacy obszar = szczeka
    ys, xs = np.nonzero(lab == i)
    sel = ys <= ys.min() + 15
    tip = np.array([xs[sel].mean(), ys[sel].mean()])
    return tip + np.array(S["tip_off"]), bm, [tip]


def cone_near(cones, uv, max_px=150):
    c = min(cones, key=lambda c: np.linalg.norm(c["uv"] - uv), default=None)
    return c if c is not None and np.linalg.norm(c["uv"] - uv) <= max_px else None


def feats(p):
    x, z = p[0], p[2]
    return np.array([1.0, x, z] + ([x * x, z * z, x * z] if M["quad"] else []))


sheet, result = [], "NIE ZLAPANA"


def shot(bgr, tgt, tips, cone, label):
    im = bgr.copy()
    for t in tips:
        cv2.circle(im, (int(t[0]), int(t[1])), 6, (255, 0, 255), -1)
    if tgt is not None:
        cv2.drawMarker(im, (int(tgt[0]), int(tgt[1])), (255, 255, 0), cv2.MARKER_CROSS, 30, 2)
    if cone is not None:
        cv2.circle(im, (int(cone["uv"][0]), int(cone["uv"][1])), 16, (255, 255, 255), 2)
    cv2.putText(im, label, (8, 30), 0, 0.9, (0, 255, 255), 2)
    sheet.append(cv2.resize(im, (320, 240)))


try:
    for k in range(S["tries"]):
        # 1. HOME i szyszka
        go({**HOME, "gripper": 30.0}, 2.5)
        bgr, depth = frames()
        cones, _ = cones_in(bgr, depth, blue_mask(bgr))
        if not cones:
            result = "BRAK SZYSZKI W KADRZE"
            break
        c0 = min(cones, key=lambda c: c["p"][2])
        q = feats(c0["p"]) @ np.array(M["W"]) + np.array(S["dq_bias"])
        grasp = clampq({j: float(q[i]) for i, j in enumerate(J4)})
        grasp["wrist_flex"] += S["wrist_extra"]
        grasp["wrist_roll"] = M["wrist_roll"]
        # nad szyszka: szczeki wyzej (lokiec do tylu, nadgarstek w gore), podstawa jak do chwytu
        high = dict(grasp)
        high["elbow_flex"] = grasp["elbow_flex"] - S["up_elbow"]
        high["wrist_flex"] = grasp["wrist_flex"] - S["up_wrist"]
        log(f"proba {k + 1}: szyszka HOME uv {np.round(c0['uv'])} -> chwyt {({j: round(v, 1) for j, v in grasp.items()})}")
        h0 = pose()
        go({**{j: h0[j] for j in J4}, "elbow_flex": high["elbow_flex"], "wrist_flex": high["wrist_flex"],
            "wrist_roll": M["wrist_roll"], "gripper": 100.0}, 1.5)          # 1. w gore w miejscu
        go({**high, "gripper": 100.0}, 1.5)                                   # 2. obrot i bark w powietrzu
        now = pose()
        # 2-3. korekta w pozie chwytu: podstawa + bark, jakobian 2x2 (px na stopien)
        Jm = np.array(S["J2"])
        prev_uv, aligned = None, False
        for it in range(S["max_corr"]):
            bgr, depth = frames()
            tgt, bm, tips = jaw_target(bgr)
            cones, _ = cones_in(bgr, depth, bm)
            cone = cone_near(cones, tgt if prev_uv is None else prev_uv) if tgt is not None else None
            shot(bgr, tgt, tips, cone, f"{k + 1}.{it}")
            if tgt is None or cone is None:
                log(f"  korekta {it}: {'brak szczek' if tgt is None else 'szyszki nie widac (zaslonieta?)'}")
                aligned = cone is None and prev_uv is not None and np.linalg.norm(prev_uv - tgt) < 60 if tgt is not None else False
                break
            e = cone["uv"] - tgt
            log(f"  korekta {it}: szyszka {np.round(cone['uv'])} cel {np.round(tgt)} blad {np.round(e)} px")
            if abs(e[0]) <= S["tol_u"] and abs(e[1]) <= S["tol_v"]:
                aligned = True
                break
            # e = J @ dq  ->  dq = -J^-1 e (przy uwarunkowaniu: tlumione)
            dq = -np.linalg.solve(Jm.T @ Jm + 4.0 * np.eye(2), Jm.T @ e)
            dq = np.clip(dq, -5.0, 5.0)
            before = pose()
            now = go({**{j: before[j] for j in J4}, "wrist_roll": M["wrist_roll"], "gripper": 100.0,
                      "shoulder_pan": before["shoulder_pan"] + dq[0], "shoulder_lift": before["shoulder_lift"] + dq[1]}, 1.0)
            dq_act = np.array([now["shoulder_pan"] - before["shoulder_pan"], now["shoulder_lift"] - before["shoulder_lift"]])
            if prev_uv is not None or True:
                bgr2, depth2 = frames()
                tgt2, bm2, _ = jaw_target(bgr2)
                cones2, _ = cones_in(bgr2, depth2, bm2)
                c2 = cone_near(cones2, cone["uv"] - Jm @ dq_act) if tgt2 is not None else None
                if c2 is not None and np.abs(dq_act).max() >= 1.0:
                    de = (c2["uv"] - tgt2) - e
                    Jm = Jm + 0.5 * np.outer(de - Jm @ dq_act, dq_act) / (dq_act @ dq_act)  # uczenie px/stopien
                prev_uv = None if c2 is None else c2["uv"]
        S["J2"] = np.round(Jm, 2).tolist()
        # 3. pionowo w dol: tylko lokiec i nadgarstek wracaja do wartosci chwytu
        here = pose()
        go({**{j: here[j] for j in J4}, "elbow_flex": here["elbow_flex"] + S["up_elbow"],
            "wrist_flex": here["wrist_flex"] + S["up_wrist"], "wrist_roll": M["wrist_roll"], "gripper": 100.0}, 1.5)
        # 4. zacisk
        ctl._move({"gripper": 0.0}, 1.0)
        time.sleep(0.4)
        g = pose()["gripper"]
        held = g >= cfg.arm.empty_gripper_below
        qf = pose()
        log(f"  zacisk: chwytak {g:.1f} -> {'TRZYMA' if held else 'PUSTY'} (wycelowane: {aligned})")
        # uczenie modelu: poprawka stawow = gdzie skonczyla korekta minus przewidywanie modelu
        corr = np.array([qf[j] for j in J4]) - (feats(c0["p"]) @ np.array(M["W"]))
        corr[3] -= S["wrist_extra"]
        S["dq_bias"] = np.round(0.5 * np.array(S["dq_bias"]) + 0.5 * corr, 2).tolist()
        if held:
            hold = max(g - 4.0, 0.0)
            ctl._move({"gripper": hold}, 0.3)
            for pan, lift, elbow, wrist, sec in JAR_UP:
                go({"shoulder_pan": qf["shoulder_pan"] if pan is None else pan, "shoulder_lift": lift, "elbow_flex": elbow,
                    "wrist_flex": wrist, "wrist_roll": M["wrist_roll"], "gripper": hold}, sec * 1.5)
            ctl._move({"gripper": 100.0}, 0.8)
            time.sleep(0.5)
            for pan, lift, elbow, wrist, sec in JAR_BACK:
                go({"shoulder_pan": pan, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist,
                    "wrist_roll": M["wrist_roll"], "gripper": 60.0}, sec * 1.5)
            result = f"ZLAPANA w probie {k + 1}, do sloika"
            S["ok"] = S.get("ok", 0) + 1
            break
        # pusto: wycelowana a pusto -> szczeki za wysoko: glebiej; niewycelowana -> tylko poprawka stawow
        if aligned:
            S["wrist_extra"] = round(min(S["wrist_extra"] + 3.0, 15.0), 1)
            log(f"  wycelowane, ale pusto -> glebiej: wrist_extra {S['wrist_extra']}")
        ctl._move({"gripper": 100.0}, 0.6)
        up = pose()
        go({**{j: up[j] for j in J4}, "elbow_flex": up["elbow_flex"] - S["up_elbow"], "wrist_flex": up["wrist_flex"] - S["up_wrist"],
            "wrist_roll": M["wrist_roll"], "gripper": 100.0}, 1.2)
except Exception as exc:
    log("BLAD", repr(exc))
    raise
finally:
    try:
        go({**HOME, "gripper": 30.0}, 2.5)
    finally:
        arm.bus.disconnect(disable_torque=False)
        cam.close()
log("RESULT", result)
print("STATE " + json.dumps(S), file=sys.stderr)
if sheet:
    while len(sheet) % 3:
        sheet.append(np.zeros_like(sheet[0]))
    img = np.vstack([np.hstack(sheet[i:i + 3]) for i in range(0, len(sheet), 3)])
    print(base64.b64encode(cv2.imencode(".jpg", img)[1].tobytes()).decode())
