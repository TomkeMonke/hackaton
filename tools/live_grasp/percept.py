# Percepcja tylko z glebi (wklejana na poczatek programow wysylanych na Pi).
# Ziemia = plaszczyzna (RANSAC). Chwytak = to, co nad ziemia i dotyka dolnej krawedzi obrazu.
# Szyszki = osobne wybrzuszenia 1.2-9 cm nad ziemia. Wszystko w metrach, uklad kamery.
import numpy as np
import cv2

RNG = np.random.default_rng(0)


def xyz_image(depth, intr):
    h, w = depth.shape
    u = np.arange(w)[None, :]
    v = np.arange(h)[:, None]
    z = depth
    return np.dstack(((u - intr.ppx) / intr.fx * z, (v - intr.ppy) / intr.fy * z, z))


def fit_ground(P, valid, iters=150, tol=0.008):
    h = P.shape[0]
    sel = valid.copy()
    sel[int(h * 0.55):, :] = False  # dol obrazu to chwytak
    pts = P[sel][::7]
    if len(pts) < 200:
        return None
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = pts[RNG.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        d = -n @ a
        cnt = int((np.abs(pts @ n + d) < tol).sum())
        if cnt > best_n:
            best, best_n = (n, d), cnt
    n, d = best
    inl = pts[np.abs(pts @ n + d) < tol]  # dopasowanie LSQ na inlierach
    c = inl.mean(0)
    n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
    d = -n @ c
    if d < 0:  # normalna w strone kamery: wysokosc > 0 nad ziemia
        n, d = -n, -d
    return n, d


def perceive(depth, intr, bgr):
    """-> dict: ground (n,d), grip (tip 3D, uv, h), cones [ (p3D, uv, h, area) ], height map."""
    valid = (depth > 0.1) & (depth < 1.5)
    P = xyz_image(depth, intr)
    g = fit_ground(P, valid)
    if g is None:
        return None
    n, d = g
    H = np.where(valid, P @ n + d, 0.0)
    # chwytak: jaskrawy niebieski PLA (duzo jasniejszy i bardziej nasycony niz trawa), najwiekszy obszar
    h_img = depth.shape[0]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (88, 120, 140), (118, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num_g, lab_g, st_g, _ = cv2.connectedComponentsWithStats(blue, connectivity=8)
    grip_ids = [i for i in range(1, num_g) if st_g[i, cv2.CC_STAT_AREA] > 1500]
    gm = np.isin(lab_g, grip_ids) if grip_ids else np.zeros(depth.shape, bool)
    gm_dil = cv2.dilate(gm.astype(np.uint8), np.ones((11, 11), np.uint8)) > 0
    # szyszki: 1.2-6 cm nad ziemia, poza chwytakiem
    low = ((H > 0.012) & (H < 0.06) & valid & ~gm_dil).astype(np.uint8)
    low = cv2.morphologyEx(low, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    num, lab, stats, cent = cv2.connectedComponentsWithStats(low, connectivity=8)
    grip_lab = []
    out = {"ground": (n, d), "H": H, "grip": None, "cones": [], "lab": lab}
    if gm.any():
        ys, xs = np.nonzero(gm)
        vtop = ys.min()
        tip = gm & (np.arange(h_img)[:, None] <= vtop + 20)
        tipv = tip & valid
        tp = P[tipv] if tipv.sum() >= 10 else P[gm & valid]
        out["grip"] = {"p": tp.mean(0), "uv": (float(np.nonzero(tip)[1].mean()), float(vtop + 10)),
                       "h": float(H[tip].mean()), "mask": gm}
    for i in range(1, num):
        if i in grip_lab:
            continue
        a = stats[i, cv2.CC_STAT_AREA]
        if a < 150 or a > 9000 or cent[i][1] > 390:  # dol obrazu: kabel serwa, korpus chwytaka
            continue
        m = lab == i
        hmax = float(H[m].max())
        out["cones"].append({"p": P[m].mean(0), "uv": (float(cent[i][0]), float(cent[i][1])), "h": hmax, "area": int(a),
                             "bbox": tuple(int(x) for x in stats[i, :4])})
    return out


def pick_cone(per, prefer_uv=None):
    cs = per["cones"]
    if not cs:
        return None
    if prefer_uv is not None:
        return min(cs, key=lambda c: (c["uv"][0] - prefer_uv[0]) ** 2 + (c["uv"][1] - prefer_uv[1]) ** 2)
    return max(cs, key=lambda c: c["area"])


def debug_image(bgr, per, cone=None):
    H = per["H"] if per else None
    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(H, 0, 0.15), alpha=255 / 0.15), cv2.COLORMAP_JET) if H is not None \
        else np.zeros_like(bgr)
    if per:
        if per["grip"]:
            u, v = per["grip"]["uv"]
            for img in (bgr, vis):
                cv2.drawMarker(img, (int(u), int(v)), (255, 0, 255), cv2.MARKER_CROSS, 26, 2)
        for c in per["cones"]:
            x, y, w, h = c["bbox"]
            col = (0, 0, 255) if c is cone else (0, 200, 255)
            for img in (bgr, vis):
                cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
    return np.hstack((cv2.resize(bgr, (480, 360)), cv2.resize(vis, (480, 360))))
