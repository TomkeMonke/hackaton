# Dopasowanie modelu z uczenia: szyszka (3D w kamerze w HOME) -> stawy chwytu.
# Uzycie (laptop): python fit.py data/teach_sessionN.jsonl data/grasp_model.json
import json, sys, numpy as np
J4 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
recs = [json.loads(l) for l in open(sys.argv[1] if len(sys.argv) > 1 else "teach.jsonl")]
samples, ambiguous = [], []
for i, r in enumerate(recs):
    if r["kind"] != "grasp":
        continue
    before = next((recs[k] for k in range(i - 1, -1, -1) if recs[k]["kind"] == "home"), None)
    after = next((recs[k] for k in range(i + 1, len(recs)) if recs[k]["kind"] == "home"), None)
    if before is None or after is None:
        continue
    gone = []
    for c in before["cones"]:
        dmin = min((np.linalg.norm(np.subtract(c["p"], a["p"])) for a in after["cones"]), default=9)
        if dmin > 0.03:
            gone.append(c)
    tr = r["traj"]
    tg = next((p["t"] for p in tr if abs(p["gripper"] - r["q"]["gripper"]) < 1.0 and p["t"] > tr[0]["t"] + 0.8), tr[-1]["t"])
    pre = min(tr, key=lambda p: abs(p["t"] - (tg - 0.7)))
    item = {"q": r["q"], "pre": pre, "gone": gone}
    if len(gone) == 1:
        samples.append({"p": gone[0]["p"], "uv": gone[0]["uv"], "q": r["q"], "pre": pre})
    elif gone:
        ambiguous.append(item)
# niejednoznaczne: szyszka zgodna z katem podstawy (u ~ a + b * pan z czystych probek)
if len(samples) >= 3 and ambiguous:
    A = np.array([[1, s_["q"]["shoulder_pan"]] for s_ in samples]); u = np.array([s_["uv"][0] for s_ in samples])
    ab, *_ = np.linalg.lstsq(A, u, rcond=None)
    print(f"u = {ab[0]:.0f} + {ab[1]:.1f} * pan")
    for it in ambiguous:
        pred = ab[0] + ab[1] * it["q"]["shoulder_pan"]
        best = min(it["gone"], key=lambda c: abs(c["uv"][0] - pred))
        others = sorted(abs(c["uv"][0] - pred) for c in it["gone"])
        if abs(best["uv"][0] - pred) < 60 and (len(others) < 2 or others[1] - others[0] > 60):
            samples.append({"p": best["p"], "uv": best["uv"], "q": it["q"], "pre": it["pre"], "resolved": True})
            print(f"  rozstrzygniete: pan {it['q']['shoulder_pan']:.1f} -> szyszka u {best['uv'][0]:.0f} (przewidziane {pred:.0f})")
        else:
            print(f"  nadal niejednoznaczne: pan {it['q']['shoulder_pan']:.1f}, kandydaci u {[c['uv'][0] for c in it['gone']]}, przewidziane {pred:.0f}")
print(len(samples), "probek")
if not samples:
    sys.exit()
X = np.array([s["p"] for s in samples])
Y = np.array([[s["q"][j] for j in J4] for s in samples])
for s in samples:
    print("szyszka p", np.round(s["p"], 3), "uv", s["uv"], "-> q", [s["q"][j] for j in J4])


def feats(P, quad):
    P = np.atleast_2d(P)
    x, z = P[:, 0], P[:, 2]
    f = [np.ones(len(P)), x, z]
    if quad:
        f += [x * x, z * z, x * z]
    return np.stack(f, 1)


for quad in (False, True):
    if len(samples) < (8 if quad else 4):
        continue
    errs = []
    for k in range(len(samples)):
        m = np.arange(len(samples)) != k
        W, *_ = np.linalg.lstsq(feats(X[m], quad), Y[m], rcond=None)
        errs.append(feats(X[k], quad) @ W - Y[k])
    errs = np.abs(np.array(errs)).reshape(-1, 4)
    print(("kwadratowy" if quad else "liniowy"), "blad zostaw-jedna [st] sredni", np.round(errs.mean(0), 1), "max", np.round(errs.max(0), 1))
quad = len(samples) >= 10
W, *_ = np.linalg.lstsq(feats(X, quad), Y, rcond=None)
dpre = np.mean([[s["pre"][j] - s["q"][j] for j in J4] for s in samples], 0)
model = {"quad": quad, "W": W.tolist(), "joints": J4, "approach_delta": dpre.round(2).tolist(),
         "wrist_roll": float(np.mean([s["q"]["wrist_roll"] for s in samples])), "n": len(samples),
         "p_min": X.min(0).tolist(), "p_max": X.max(0).tolist(),
         "q_min": Y.min(0).tolist(), "q_max": Y.max(0).tolist()}
out = sys.argv[2] if len(sys.argv) > 2 else "grasp_model.json"
json.dump(model, open(out, "w"), indent=1)
print("zapisano", out)
print("model:", "kwadratowy" if quad else "liniowy", "podejscie delta", np.round(dpre, 1))
