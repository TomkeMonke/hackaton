# Serwo wizyjne z glebi, samo ramie. Doklejane za percept.py, wysylane na Pi przez stdin.
# STATE (json) podstawia runner. Log na stderr, na koncu "STATE {...}"; stdout: jpg ostatniej klatki.
import base64, json, sys, time
import arm_control as ac
from pinecone_bot.config import Config
from pinecone_bot.arm import WaypointArm
from pinecone_bot.camera import RealSenseCamera

S = json.loads(STATE)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
LIM = {"shoulder_pan": (-45, 30), "shoulder_lift": (55, 88.3), "elbow_flex": (-45, 40), "wrist_flex": (-100, -30)}
HOME = {"shoulder_pan": -5.45, "shoulder_lift": 88.3, "elbow_flex": 7.56, "wrist_flex": -87.87, "wrist_roll": 88.88}
T0 = time.time()


def log(*a):
    print(f"[{time.time() - T0:5.1f}s]", *a, file=sys.stderr, flush=True)


cfg = Config.load()
arm = ac.make_arm(port="/dev/robot-arm", max_relative_target=None)
for i in range(3):
    try:
        arm.bus.connect()
        break
    except Exception as e:
        log("connect nieudany:", str(e).splitlines()[0])
        try:
            arm.bus.port_handler.closePort()
        except Exception:
            pass
        time.sleep(1.0)
cur = arm.bus.sync_read("Present_Position")
arm.bus.sync_write("Goal_Position", cur)
arm.bus.enable_torque()
ctl = WaypointArm(cfg, arm=arm)
cam = RealSenseCamera(cfg, depth=True)
intr = cam.intrinsics


def pose():
    return arm.bus.sync_read("Present_Position")


def go(target, sec=1.0):
    """Ruch ~1 s, potem korekta calkujaca bledu serw (max 3 razy)."""
    ctl._move(target, sec)
    time.sleep(0.3)
    cmd = dict(target)
    for _ in range(3):
        now = pose()
        err = {j: target[j] - now[j] for j in target if j != "gripper"}
        if all(abs(e) <= 0.8 for e in err.values()):
            break
        cmd = {j: (cmd[j] + max(-6.0, min(6.0, err[j])) if j in err and abs(err[j]) > 0.8 else cmd[j]) for j in cmd}
        ctl._move(cmd, 0.4)
        time.sleep(0.25)
    return pose()


def clamp_joints(q):
    return {j: max(LIM[j][0], min(LIM[j][1], v)) if j in LIM else v for j, v in q.items()}


last = {"bgr": None, "per": None, "cone": None}


def measure(prefer_uv, max_jump=90):
    """-> (y [cm]: poziomo e1, poziomo e2, wysokosc koncowki; stozek; chwytak) albo None.
    Sledzi jedna szyszke: bez przeskoku na inna (dalej niz max_jump px od poprzedniej = zgubiona)."""
    for _ in range(3):
        bgr, depth = cam.read()
    per = perceive(depth, intr, bgr)
    last["bgr"], last["per"] = bgr, per
    if per is None or per["grip"] is None or not per["cones"]:
        return None
    cone = pick_cone(per, prefer_uv)
    if prefer_uv is not None and np.hypot(cone["uv"][0] - prefer_uv[0], cone["uv"][1] - prefer_uv[1]) > max_jump:
        return None
    last["cone"] = cone
    n, _ = per["ground"]
    x = np.array([1.0, 0.0, 0.0])
    e1 = x - (x @ n) * n
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    e = cone["p"] - per["grip"]["p"]
    y = np.array([e @ e1, e @ e2, per["grip"]["h"]]) * 100.0
    return y, cone, per["grip"]


def residual(y, h_goal):
    return y - np.array([S["off"][0], S["off"][1], h_goal])


def solve(J, r, max_step=4.0):
    J = np.asarray(J)
    lam = 0.1
    dq = -J.T @ np.linalg.solve(J @ J.T + lam * np.eye(J.shape[0]), r)
    m = np.abs(dq).max()
    if m > max_step:
        return dq * (max_step / m)
    if 1e-6 < m < 1.2:  # ponizej ~1 st serwo nie rusza (niskie P)
        return dq * (1.2 / m)
    return dq


def broyden(J, dq, dy):
    J = np.asarray(J)
    if np.abs(dq).max() < 1.5:
        return J
    return J + 0.3 * np.outer(dy - J @ dq, dq) / (dq @ dq)


def qvec(p):
    return np.array([p[j] for j in JOINTS])


result = "NIE UDALO SIE"
try:
    go({**{j: cur[j] for j in JOINTS}, "gripper": 0.0}, 0.8)  # zamkniety chwytak = jedna koncowka do sledzenia
    m = measure(None)
    if m is None:
        raise SystemExit("brak szyszki albo chwytaka w glebi")
    y, cone, grip = m
    uv = cone["uv"]
    log(f"start: blad poziomo ({y[0]:+.1f},{y[1]:+.1f}) cm, koncowka {y[2]:.1f} cm nad ziemia, szyszka h {cone['h']*100:.1f} cm")
    # 1. Jakobian: proba kazdym stawem (jesli nie ma nauczonego)
    J = None  # proby na starcie kazdego uruchomienia (Jakobian zalezy od pozy)
    if J is None:
        J = np.zeros((3, 4))
        for i, j in enumerate(JOINTS):
            p0 = pose()
            step = -3.0 if j == "shoulder_lift" else 3.0
            p1 = go({**{k: p0[k] for k in JOINTS}, j: p0[j] + step, "gripper": 0.0})
            m1 = measure(uv)
            dq = p1[j] - p0[j]
            if m1 is None or abs(dq) < 0.8:
                log(f"proba {j}: brak pomiaru albo staw nie ruszyl (dq {dq:.1f})")
                continue
            J[:, i] = (m1[0] - y) / dq
            log(f"proba {j}: dq {dq:+.1f} -> dy {np.round(m1[0] - y, 2)} cm")
            y, uv = m1[0], m1[1]["uv"]
        S["J"] = J.round(3).tolist()
    # 2. petla: 1 s ruchu, stop, glebia, nastepny krok
    phase, h_goal = "A", S["h_above"]
    lost = 0
    for it in range(S["max_iter"]):
        r = residual(y, h_goal)
        log(f"{phase}{it}: blad poziomo ({r[0]:+.1f},{r[1]:+.1f}) wysokosc {r[2]:+.1f} cm")
        if np.abs(r[:2]).max() <= S["tol_xy"] and abs(r[2]) <= S["tol_h"]:
            if phase == "A":
                # 3. nad szyszka: otworz, zejdz na slepo z Jakobianu, zacisnij
                p_above = pose()
                ctl._move({"gripper": 100.0}, 0.8)
                dq = solve(J, np.array([0.0, 0.0, y[2] - S["h_grasp"]]), max_step=12.0)
                log(f"schodze {y[2]:.1f} -> {S['h_grasp']:.1f} cm: dq {np.round(dq, 1)}")
                target = clamp_joints({j: p_above[j] + dq[k] for k, j in enumerate(JOINTS)})
                go({**target, "gripper": 100.0}, 1.5)
                ctl._move({"gripper": 0.0}, 1.0)
                time.sleep(0.4)
                g = pose()["gripper"]
                held = g >= cfg.arm.empty_gripper_below
                log(f"zacisk: chwytak {g:.1f} -> {'TRZYMA' if held else 'PUSTY'}")
                hold = max(g - 3.0, 0.0)
                ctl._move({"gripper": hold}, 0.3)
                go({**{j: p_above[j] for j in JOINTS}, "gripper": hold}, 1.5)
                cone_before = cone
                m2 = measure(uv)
                still = None
                if last["per"] is not None:
                    still = min(last["per"]["cones"], key=lambda c: np.linalg.norm(c["p"] - cone_before["p"]), default=None)
                    if still is not None and np.linalg.norm(still["p"] - cone_before["p"]) > 0.06:
                        still = None
                if held and still is None:
                    result = "PODNIESIONA"
                    S["ok"] = S.get("ok", 0) + 1
                    break
                S["fails"] = S.get("fails", 0) + 1
                if still is not None:
                    push = (still["p"] - cone_before["p"]) * 100.0
                    n, _ = last["per"]["ground"]
                    x = np.array([1.0, 0.0, 0.0]); e1 = x - (x @ n) * n; e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
                    ph = np.array([push @ e1, push @ e2])
                    if np.linalg.norm(ph) < 1.0:
                        S["h_grasp"] = round(max(-2.0, S["h_grasp"] - 0.7), 2)
                        log(f"szyszka nieruszona -> nizej: h_grasp {S['h_grasp']} cm")
                    else:
                        S["off"] = [round(S["off"][0] - 0.5 * ph[0], 2), round(S["off"][1] - 0.5 * ph[1], 2)]
                        log(f"szyszka popchnieta o ({ph[0]:+.1f},{ph[1]:+.1f}) cm -> przesuniecie celu {S['off']}")
                else:  # nie widac: lezy pod chwytakiem -> szczeki zamknely sie nad nia
                    S["h_grasp"] = round(max(-2.0, S["h_grasp"] - 0.8), 2)
                    log(f"szyszka pod chwytakiem (zaslonieta) -> nizej: h_grasp {S['h_grasp']} cm")
                if held:
                    ctl._move({"gripper": 100.0}, 0.8)
                ctl._move({"gripper": 0.0}, 0.8)
                # odjedz w gore, zeby zobaczyc szyszke i sprobowac jeszcze raz
                go({**{j: p_above[j] for j in JOINTS}, "shoulder_lift": min(LIM["shoulder_lift"][1], p_above["shoulder_lift"] + 4), "gripper": 0.0})
                m = measure(None)
                if m is None:
                    log("po probie nie widze szyszki")
                    break
                y, cone, grip = m
                uv = cone["uv"]
                continue
        p0 = pose()
        # stawy na granicy zakresu wypadaja z rozwiazania (inaczej obciety krok = zero ruchu)
        active = list(range(4))
        for _ in range(4):
            dq = np.zeros(4)
            dq[active] = solve(np.asarray(J)[:, active], r)
            blocked = [k for k in active if (p0[JOINTS[k]] + dq[k] > LIM[JOINTS[k]][1] - 0.3 and dq[k] > 0)
                       or (p0[JOINTS[k]] + dq[k] < LIM[JOINTS[k]][0] + 0.3 and dq[k] < 0)]
            if not blocked:
                break
            active = [k for k in active if k not in blocked]
            if not active:
                break
        target = clamp_joints({j: p0[j] + dq[k] for k, j in enumerate(JOINTS)})
        log(f"  krok {np.round(dq, 1)} (stawy {[JOINTS[k][:5] for k in active]})")
        p1 = go({**target, "gripper": 0.0})
        dq_act = qvec(p1) - qvec(p0)
        m = measure(uv)
        if m is None:
            y = y + J @ dq_act  # szyszka zaslonieta: przewidywanie z Jakobianu
            lost = lost + 1
            log(f"  krok {np.round(dq_act, 1)}: szyszka zaslonieta ({lost}), przewiduje blad {np.round(residual(y, h_goal), 1)}")
            if lost > 4:
                break
            continue
        lost = 0
        J = broyden(J, dq_act, m[0] - y)
        y, cone, grip = m
        uv = cone["uv"]
    S["J"] = np.asarray(J).round(3).tolist()
finally:
    try:
        g = pose()["gripper"]
        go({**HOME, "gripper": g}, 2.0)
        if result == "PODNIESIONA":
            log("HOME z szyszka - zabierz ja (4 s)")
            time.sleep(4.0)
        ctl._move({"gripper": 100.0}, 0.8)
    finally:
        arm.bus.disconnect(disable_torque=False)
        cam.close()
log("RESULT", result)
print("STATE " + json.dumps(S), file=sys.stderr)
if last["bgr"] is not None and last["per"] is not None:
    print(base64.b64encode(cv2.imencode(".jpg", debug_image(last["bgr"], last["per"], last["cone"]))[1].tobytes()).decode())
