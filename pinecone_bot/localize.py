"""
Lokalizacja w gotowej mapie (RTAB-Map) z jednego zdjecia kolorowego - deterministycznie, bez ML.

Mapa = klatki kluczowe z nagrania `tools/record_rgbd.py` (kolor + glebia) z pozami z `map_poses.txt`
(`tools/rtabmap_build.py`). Dla kazdej klatki kluczowej: cechy ORB i ich punkty 3D w ukladzie mapy
(z glebi). Zapytanie: cechy ORB z nowego zdjecia -> dopasowanie do kilku najlepszych klatek kluczowych
-> PnP RANSAC -> poza kamery w mapie. Robot staje, patrzy, dostaje (x, y, kurs).

Uklad mapy RTAB-Map: x do przodu, y w lewo, z do gory (pierwsza klatka nagrania = poczatek).
Uklad optyczny kamery: X w prawo, Y w dol, Z do przodu.

  python -m pinecone_bot.localize build ~/mapy/ogrod1           # -> ~/mapy/ogrod1/map_features.npz
  python -m pinecone_bot.localize eval ~/mapy/ogrod1            # blad na klatkach spoza mapy
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np

# optyczny (X prawo, Y dol, Z przod) -> baza RTAB-Map (x przod, y lewo, z gora)
R_BASE_OPT = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]])

DEPTH_MIN_M, DEPTH_MAX_M = 0.3, 4.0


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float          # rad, 0 = kierunek x mapy, CCW dodatni
    inliers: int = 0
    keyframe: int = -1  # id klatki kluczowej z najwieksza liczba dopasowan


def quat_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_poses(path: str) -> dict[int, np.ndarray]:
    """map_poses.txt z rtabmap-export (#timestamp x y z qx qy qz qw id) -> {id: 4x4 baza->mapa}."""
    out = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            p = line.split()
            T = np.eye(4)
            T[:3, :3] = quat_to_R(*map(float, p[4:8]))
            T[:3, 3] = [float(v) for v in p[1:4]]
            out[int(p[8])] = T
    return out


def read_intrinsics(calib_yaml: str) -> np.ndarray:
    fs = cv2.FileStorage(calib_yaml, cv2.FILE_STORAGE_READ)
    K = fs.getNode("camera_matrix").mat()
    fs.release()
    if K is None:
        raise ValueError("brak camera_matrix w %s" % calib_yaml)
    return K.astype(np.float64)


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def pose_from_pnp(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """rvec/tvec z solvePnP (mapa -> optyczny) -> 4x4 baza kamery -> mapa."""
    R, _ = cv2.Rodrigues(rvec)
    T_opt_map = np.eye(4)
    T_opt_map[:3, :3] = R
    T_opt_map[:3, 3] = tvec.ravel()
    T_map_opt = np.linalg.inv(T_opt_map)
    T = np.eye(4)
    T[:3, :3] = T_map_opt[:3, :3] @ R_BASE_OPT.T
    T[:3, 3] = T_map_opt[:3, 3]
    return T


def keypoints_3d(kps, depth_m: np.ndarray, K: np.ndarray, T_map_base: np.ndarray):
    """Indeksy cech z dobra glebia i ich punkty 3D w ukladzie mapy."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    h, w = depth_m.shape
    idx, pts = [], []
    for i, kp in enumerate(kps):
        u, v = int(round(kp.pt[0])), int(round(kp.pt[1]))
        if not (0 <= u < w and 0 <= v < h):
            continue
        z = float(depth_m[v, u])
        if not (DEPTH_MIN_M <= z <= DEPTH_MAX_M):
            continue
        idx.append(i)
        pts.append(((kp.pt[0] - cx) * z / fx, (kp.pt[1] - cy) * z / fy, z))
    if not pts:
        return [], np.zeros((0, 3), np.float32)
    p_opt = np.array(pts)
    p_base = p_opt @ R_BASE_OPT.T
    p_map = p_base @ T_map_base[:3, :3].T + T_map_base[:3, 3]
    return idx, p_map.astype(np.float32)


class MapLocalizer:
    def __init__(self, K: np.ndarray, ids, poses, descs, points, n_features: int = 1500):
        self.K = K
        self.ids = list(ids)
        self.poses = list(poses)          # 4x4 baza -> mapa
        self.descs = list(descs)          # (N, 32) uint8
        self.points = list(points)        # (N, 3) float32, uklad mapy
        self.orb = cv2.ORB_create(n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    # --- budowa ---
    @classmethod
    def build(cls, dataset: str, poses_path: str | None = None, n_features: int = 1000,
              verbose: bool = True) -> "MapLocalizer":
        poses = read_poses(poses_path or os.path.join(dataset, "map_poses.txt"))
        K = read_intrinsics(os.path.join(dataset, "calib", "rs_color.yaml"))
        orb = cv2.ORB_create(n_features)
        ids, Ts, descs, pts = [], [], [], []
        for n, (kid, T) in enumerate(sorted(poses.items())):
            base = "%06d" % kid
            img = cv2.imread(os.path.join(dataset, "rgb", base + ".jpg"), cv2.IMREAD_GRAYSCALE)
            depth = cv2.imread(os.path.join(dataset, "depth", base + ".png"), cv2.IMREAD_UNCHANGED)
            if img is None or depth is None:
                continue
            kps, d = orb.detectAndCompute(img, None)
            if d is None:
                continue
            idx, p = keypoints_3d(kps, depth.astype(np.float32) / 1000.0, K, T)
            if len(idx) < 20:
                continue
            ids.append(kid)
            Ts.append(T)
            descs.append(d[idx])
            pts.append(p)
            if verbose and n % 50 == 0:
                print("  klatka %d: %d cech z glebia" % (kid, len(idx)), flush=True)
        if verbose:
            print("mapa: %d klatek kluczowych z %d poz" % (len(ids), len(poses)))
        return cls(K, ids, Ts, descs, pts)

    def save(self, path: str) -> None:
        np.savez_compressed(path, K=self.K, ids=np.array(self.ids), poses=np.array(self.poses),
                            counts=np.array([len(d) for d in self.descs]),
                            descs=np.concatenate(self.descs), points=np.concatenate(self.points))

    @classmethod
    def load(cls, path: str) -> "MapLocalizer":
        z = np.load(path)
        split = np.cumsum(z["counts"])[:-1]
        return cls(z["K"], z["ids"].tolist(), list(z["poses"]),
                   np.split(z["descs"], split), np.split(z["points"], split))

    # --- zapytanie ---
    def localize(self, bgr: np.ndarray, prior: Pose2D | None = None, radius_m: float = 3.0,
                 top_k: int = 3, min_inliers: int = 25) -> Pose2D | None:
        """Poza kamery w mapie albo None. prior zaweza szukanie do klatek w promieniu radius_m."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        kps, d = self.orb.detectAndCompute(gray, None)
        if d is None or len(kps) < min_inliers:
            return None
        cand = range(len(self.ids))
        if prior is not None:
            near = [i for i in cand if math.hypot(self.poses[i][0, 3] - prior.x,
                                                  self.poses[i][1, 3] - prior.y) <= radius_m]
            cand = near or cand
        scored = []
        for i in cand:
            good = self._good_matches(d, self.descs[i])
            if len(good) >= 8:
                scored.append((len(good), i, good))
        if not scored:
            return None
        scored.sort(key=lambda s: -s[0])
        obj, img = [], []
        for _n, i, good in scored[:top_k]:
            for m in good:
                obj.append(self.points[i][m.trainIdx])
                img.append(kps[m.queryIdx].pt)
        if len(obj) < min_inliers:
            return None
        ok, rvec, tvec, inl = cv2.solvePnPRansac(
            np.array(obj, np.float64), np.array(img, np.float64), self.K, None,
            iterationsCount=300, reprojectionError=4.0, confidence=0.995,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inl is None or len(inl) < min_inliers:
            return None
        T = pose_from_pnp(rvec, tvec)
        return Pose2D(float(T[0, 3]), float(T[1, 3]), yaw_of(T), int(len(inl)),
                      int(self.ids[scored[0][1]]))

    def _good_matches(self, q: np.ndarray, t: np.ndarray, ratio: float = 0.8):
        if len(t) < 2:
            return []
        good = []
        for pair in self.matcher.knnMatch(q, t, k=2):
            if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance:
                good.append(pair[0])
        return good


def _interp_pose(poses: dict[int, np.ndarray], fid: int) -> np.ndarray | None:
    """Poza klatki miedzy dwiema klatkami kluczowymi (liniowo x, y, kurs) - przyblizona prawda do oceny."""
    ks = sorted(poses)
    lo = max((k for k in ks if k <= fid), default=None)
    hi = min((k for k in ks if k >= fid), default=None)
    if lo is None or hi is None:
        return None
    if lo == hi:
        return poses[lo]
    a = (fid - lo) / (hi - lo)
    A, B = poses[lo], poses[hi]
    T = np.eye(4)
    T[:3, 3] = (1 - a) * A[:3, 3] + a * B[:3, 3]
    ya, yb = yaw_of(A), yaw_of(B)
    y = ya + a * math.atan2(math.sin(yb - ya), math.cos(yb - ya))
    T[:2, :2] = [[math.cos(y), -math.sin(y)], [math.sin(y), math.cos(y)]]
    return T


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["build", "eval"])
    ap.add_argument("dataset", help="katalog nagrania z map_poses.txt")
    ap.add_argument("--features", default=None, help="plik .npz (domyslnie <dataset>/map_features.npz)")
    ap.add_argument("--step", type=int, default=5, help="eval: co ktora klatke sprawdzac")
    args = ap.parse_args()
    ds = os.path.expanduser(args.dataset)
    npz = args.features or os.path.join(ds, "map_features.npz")

    if args.cmd == "build":
        loc = MapLocalizer.build(ds)
        loc.save(npz)
        print("zapisano", npz, "%.1f MB" % (os.path.getsize(npz) / 1e6))
        return 0

    import time
    loc = MapLocalizer.load(npz)
    poses = read_poses(os.path.join(ds, "map_poses.txt"))
    keyset = set(loc.ids)
    n_rgb = len(os.listdir(os.path.join(ds, "rgb")))
    errs, yerrs, fails, times = [], [], 0, []
    for fid in range(1, n_rgb + 1, args.step):
        if fid in keyset:
            continue
        truth = _interp_pose(poses, fid)
        if truth is None:
            continue
        img = cv2.imread(os.path.join(ds, "rgb", "%06d.jpg" % fid))
        t0 = time.time()
        p = loc.localize(img)
        times.append(time.time() - t0)
        if p is None:
            fails += 1
            continue
        errs.append(math.hypot(p.x - truth[0, 3], p.y - truth[1, 3]))
        yerrs.append(abs(math.degrees(math.atan2(math.sin(p.yaw - yaw_of(truth)),
                                                 math.cos(p.yaw - yaw_of(truth))))))
    n = len(errs) + fails
    if not errs:
        print("zadna z %d klatek sie nie zlokalizowala" % n)
        return 1
    e, ye = np.array(errs), np.array(yerrs)
    print("klatek %d, zlokalizowane %d (%.0f%%), czas %.2f s/klatke" % (
        n, len(errs), 100 * len(errs) / n, float(np.mean(times))))
    print("blad polozenia: mediana %.3f m, 90%% %.3f m; kursu: mediana %.1f st, 90%% %.1f st" % (
        np.median(e), np.percentile(e, 90), np.median(ye), np.percentile(ye, 90)))
    print("(prawda = interpolacja miedzy klatkami kluczowymi, sama ma blad rzedu cm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
