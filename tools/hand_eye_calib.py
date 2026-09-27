"""Kalibracja reka-oko: jak rusza sie kamera, gdy rusza sie ramie.

Kamera D415 siedzi na ramieniu, wiec jej polozenie w ukladzie podstawy ramienia
to zawsze:

    T_base_cam(stawy) = FK(stawy) @ T_gripper_cam

FK daje placo z URDF (jak w tools/frame_check.py), a T_gripper_cam to STALA
transformata kamera -> ramka chwytaka (gripper_frame_link), ktorej szukamy.
Zero ML: klasyczne AX = XB (cv2.calibrateHandEye) na parach
(FK(stawy_i), poza markera ArUco w kamerze_i) dla N poz ramienia, marker lezy
nieruchomo na stole.

Podkomendy:
    marker   - PNG z markerem ArUco do wydruku (DICT_4X4_50, id 0).
    collect  - torque OFF, prowadzisz ramie reka, Enter = probka (stawy + marker),
               q = koniec. Zapis hand_eye_samples.json (+ opcjonalnie klatki).
    solve    - FK per probka (placo), calibrateHandEye, residua, zapis
               camera_on_arm.json (T_gripper_cam).
    predict  - dla zadanych stawow (lub z ramienia) wypisuje polozenie kamery
               i kierunek patrzenia w ukladzie podstawy.

Wymagania do dobrego wyniku: 12-20 probek, marker widoczny caly, DUZE roznice
ORIENTACJI kamery miedzy probkami (krec nadgarstkiem i lokciem, nie tylko
przesuwaj), dlugosc boku markera zmierzona linijka po wydruku (--marker-len).

Zaleznosci sprzetowe (lerobot, placo, pyrealsense2) importowane leniwie:
`solve`/`predict` z gotowym JSON chodza wszedzie, gdzie jest placo (Pi);
czysta matematyka testowana na laptopie (tests/test_hand_eye_calib.py).
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

from pinecone_bot.arm import JOINT_NAMES  # noqa: E402
from frame_check import GRIPPER_FRAME, find_urdf, make_kinematics, pose_to_array  # noqa: E402

ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
DEFAULT_MARKER_LEN_M = 0.05
SAMPLES_FILE = "hand_eye_samples.json"
RESULT_FILE = "camera_on_arm.json"


# ---------------------------------------------------------------------------
# Matematyka (czysta, testowalna)
# ---------------------------------------------------------------------------

def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return make_T(R, np.asarray(tvec, dtype=float).reshape(3))


def rotation_angle_deg(R: np.ndarray) -> float:
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def rpy_deg(R: np.ndarray) -> list[float]:
    """Roll/pitch/yaw (ZYX) w stopniach, do czytania przez czlowieka."""
    sy = -R[2, 0]
    pitch = np.arcsin(np.clip(sy, -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return [float(np.degrees(v)) for v in (roll, pitch, yaw)]


def solve_hand_eye(T_base_gripper: list[np.ndarray], T_cam_marker: list[np.ndarray],
                   method: int = cv2.CALIB_HAND_EYE_TSAI) -> np.ndarray:
    """AX = XB -> X = T_gripper_cam (poza kamery w ramce chwytaka)."""
    if len(T_base_gripper) < 3 or len(T_base_gripper) != len(T_cam_marker):
        raise ValueError("potrzeba >= 3 par (FK, marker) o tej samej liczbie")
    R_g2b = [T[:3, :3] for T in T_base_gripper]
    t_g2b = [T[:3, 3].reshape(3, 1) for T in T_base_gripper]
    R_t2c = [T[:3, :3] for T in T_cam_marker]
    t_t2c = [T[:3, 3].reshape(3, 1) for T in T_cam_marker]
    R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=method)
    return make_T(R_c2g, t_c2g)


def residuals(T_base_gripper: list[np.ndarray], T_cam_marker: list[np.ndarray], X: np.ndarray) -> dict:
    """Marker jest nieruchomy: T_base_marker_i = T_bg_i @ X @ T_cm_i powinno byc stale.

    Zwraca rozrzut polozenia [mm] i orientacji [st] markera w bazie oraz odchylki per probka.
    """
    Ts = [Tbg @ X @ Tcm for Tbg, Tcm in zip(T_base_gripper, T_cam_marker)]
    pos = np.array([T[:3, 3] for T in Ts])
    mean_pos = pos.mean(axis=0)
    dev_mm = np.linalg.norm(pos - mean_pos, axis=1) * 1000.0
    R_ref = Ts[0][:3, :3]
    dev_deg = np.array([rotation_angle_deg(R_ref.T @ T[:3, :3]) for T in Ts])
    return {
        "marker_in_base_mean_m": [round(float(v), 4) for v in mean_pos],
        "pos_rms_mm": round(float(np.sqrt(np.mean(dev_mm ** 2))), 2),
        "pos_max_mm": round(float(dev_mm.max()), 2),
        "rot_rms_deg": round(float(np.sqrt(np.mean(dev_deg ** 2))), 3),
        "rot_max_deg": round(float(dev_deg.max()), 3),
        "per_sample_mm": [round(float(v), 2) for v in dev_mm],
        "per_sample_deg": [round(float(v), 3) for v in dev_deg],
    }


def camera_pose(T_base_gripper: np.ndarray, X: np.ndarray) -> np.ndarray:
    """T_base_cam dla danej FK. Kamera patrzy wzdluz swojej osi +Z."""
    return T_base_gripper @ X


def describe_camera(T_base_cam: np.ndarray) -> str:
    p = T_base_cam[:3, 3] * 100.0
    look = T_base_cam[:3, 2]
    return ("kamera w bazie: X {:+.1f} cm, Y {:+.1f} cm, Z {:+.1f} cm; patrzy wzdluz "
            "[{:+.2f}, {:+.2f}, {:+.2f}]").format(p[0], p[1], p[2], look[0], look[1], look[2])


# ---------------------------------------------------------------------------
# ArUco
# ---------------------------------------------------------------------------

def marker_object_points(marker_len_m: float) -> np.ndarray:
    h = marker_len_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)


def make_marker_image(px: int = 800, marker_id: int = MARKER_ID) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    img = cv2.aruco.generateImageMarker(d, marker_id, px)
    border = px // 8
    return cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)


def detect_marker(bgr: np.ndarray, K: np.ndarray, dist: np.ndarray, marker_len_m: float,
                  marker_id: int = MARKER_ID):
    """-> (rvec, tvec, corners) markera w ukladzie kamery albo None."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    d = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(d, params)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if int(i) != marker_id:
            continue
        ok, rvec, tvec = cv2.solvePnP(marker_object_points(marker_len_m), c.reshape(4, 2).astype(np.float32),
                                      K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            return rvec.reshape(3), tvec.reshape(3), c.reshape(4, 2)
    return None


def intrinsics_to_K(intr) -> np.ndarray:
    return np.array([[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]], dtype=float)


# ---------------------------------------------------------------------------
# Podkomendy
# ---------------------------------------------------------------------------

def cmd_marker(args) -> int:
    img = make_marker_image(args.px)
    cv2.imwrite(args.out, img)
    print(f"Zapisano {args.out} (DICT_4X4_50, id {MARKER_ID}). Wydrukuj, przyklej na sztywno,")
    print("zmierz linijka bok CZARNEGO kwadratu i podaj go w --marker-len przy collect.")
    return 0


def cmd_collect(args) -> int:
    from pinecone_bot.config import Config
    from pinecone_bot.camera import make_camera
    import arm_control as ac

    cfg = Config()
    cam = make_camera(cfg, depth=False)
    K = intrinsics_to_K(cam.intrinsics)
    dist = np.zeros(5)
    arm = ac.make_arm(port=args.port or cfg.arm.port, arm_id=cfg.arm.arm_id, max_relative_target=None)
    arm.connect(calibrate=False)
    samples = []
    if args.save_frames:
        os.makedirs(args.save_frames, exist_ok=True)
    try:
        arm.bus.disable_torque()
        print("Torque WYLACZONY - trzymaj ramie. Marker nieruchomo na stole, caly w kadrze.")
        print("Enter = probka, q = koniec. Miedzy probkami MOCNO zmieniaj orientacje kamery.")
        while True:
            cmd = input(f"[{len(samples)} probek] Enter/q: ").strip().lower()
            if cmd == "q":
                break
            pose = ac.read_joint_positions(arm, retries=2)
            bgr, _ = cam.read()
            det = detect_marker(bgr, K, dist, args.marker_len)
            if det is None:
                print("   marker NIE wykryty - popraw kadr i sprobuj jeszcze raz")
                continue
            rvec, tvec, corners = det
            samples.append({
                "t": time.time(),
                "joints": {k: float(v) for k, v in pose.items()},
                "rvec": [float(v) for v in rvec],
                "tvec": [float(v) for v in tvec],
                "corners_px": corners.tolist(),
            })
            print(f"   OK: marker w kamerze {tvec[2] * 100:.1f} cm przed obiektywem, "
                  f"stawy {', '.join(f'{k}={v:+.0f}' for k, v in pose.items())}")
            if args.save_frames:
                cv2.imwrite(os.path.join(args.save_frames, f"sample_{len(samples):02d}.png"), bgr)
    finally:
        print("Wlaczam torque (ramie trzyma pozycje)...")
        try:
            arm.bus.enable_torque()
        finally:
            try:
                arm.bus.disconnect(disable_torque=False)
            except Exception:  # noqa: BLE001
                arm.disconnect()
            cam.close()
    out = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "marker_len_m": args.marker_len,
        "aruco_dict": "DICT_4X4_50",
        "marker_id": MARKER_ID,
        "intrinsics": {"fx": K[0, 0], "fy": K[1, 1], "ppx": K[0, 2], "ppy": K[1, 2],
                       "width": cam.intrinsics.width, "height": cam.intrinsics.height},
        "samples": samples,
    }
    with open(args.out, "w", encoding="ascii") as f:
        json.dump(out, f, indent=1)
    print(f"Zapisano {len(samples)} probek do {args.out}. Dalej: solve --samples {args.out}")
    return 0 if len(samples) >= 3 else 1


def load_pairs(samples_path: str, kin) -> tuple[list[np.ndarray], list[np.ndarray], dict]:
    with open(samples_path, encoding="ascii") as f:
        data = json.load(f)
    T_bg, T_cm = [], []
    for s in data["samples"]:
        T_bg.append(np.asarray(kin.forward_kinematics(pose_to_array(s["joints"])), dtype=float))
        T_cm.append(rvec_tvec_to_T(s["rvec"], s["tvec"]))
    return T_bg, T_cm, data


def cmd_solve(args) -> int:
    urdf = args.urdf or find_urdf()
    kin = make_kinematics(urdf)
    T_bg, T_cm, data = load_pairs(args.samples, kin)
    X = solve_hand_eye(T_bg, T_cm)
    res = residuals(T_bg, T_cm, X)
    result = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "urdf": urdf,
        "gripper_frame": GRIPPER_FRAME,
        "n_samples": len(T_bg),
        "marker_len_m": data.get("marker_len_m"),
        "T_gripper_cam": [[round(float(v), 6) for v in row] for row in X],
        "translation_m": [round(float(v), 4) for v in X[:3, 3]],
        "rpy_deg": [round(v, 2) for v in rpy_deg(X[:3, :3])],
        "residuals": res,
    }
    with open(args.out, "w", encoding="ascii") as f:
        json.dump(result, f, indent=1)
    print(f"T_gripper_cam: przesuniecie {result['translation_m']} m, rpy {result['rpy_deg']} st")
    print(f"residua: polozenie RMS {res['pos_rms_mm']} mm (max {res['pos_max_mm']}), "
          f"orientacja RMS {res['rot_rms_deg']} st (max {res['rot_max_deg']})")
    print("Ocena: RMS < 10 mm i < 2 st = dobrze; wieksze = za malo roznych orientacji, "
          "zly --marker-len albo zle zera stawow (najpierw tools/frame_check.py).")
    print(f"Zapisano {args.out}")
    return 0


def cmd_predict(args) -> int:
    with open(args.result, encoding="ascii") as f:
        result = json.load(f)
    X = np.asarray(result["T_gripper_cam"], dtype=float)
    kin = make_kinematics(args.urdf or result.get("urdf") or find_urdf())
    if args.joints:
        vals = [float(v) for v in args.joints.split(",")]
        pose = dict(zip(JOINT_NAMES, vals))
    else:
        from pinecone_bot.config import Config
        import arm_control as ac
        cfg = Config()
        arm = ac.make_arm(port=args.port or cfg.arm.port, arm_id=cfg.arm.arm_id)
        arm.connect(calibrate=False)
        try:
            pose = ac.read_joint_positions(arm)
        finally:
            arm.disconnect()
    T_bg = np.asarray(kin.forward_kinematics(pose_to_array(pose)), dtype=float)
    print("stawy:", {k: round(v, 1) for k, v in pose.items()})
    print(describe_camera(camera_pose(T_bg, X)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Kalibracja reka-oko kamery na ramieniu SO-101")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("marker", help="PNG markera ArUco do wydruku")
    p.add_argument("--out", default="aruco_marker.png")
    p.add_argument("--px", type=int, default=800)
    p.set_defaults(func=cmd_marker)

    p = sub.add_parser("collect", help="zbieranie probek (torque off, Enter = probka)")
    p.add_argument("--marker-len", type=float, default=DEFAULT_MARKER_LEN_M, help="bok markera [m] po wydruku")
    p.add_argument("--port", default=None)
    p.add_argument("--out", default=SAMPLES_FILE)
    p.add_argument("--save-frames", default=None, help="katalog na klatki PNG (debug)")
    p.set_defaults(func=cmd_collect)

    p = sub.add_parser("solve", help="calibrateHandEye na zebranych probkach")
    p.add_argument("--samples", default=SAMPLES_FILE)
    p.add_argument("--urdf", default=None)
    p.add_argument("--out", default=RESULT_FILE)
    p.set_defaults(func=cmd_solve)

    p = sub.add_parser("predict", help="polozenie kamery dla stawow (z ramienia albo --joints)")
    p.add_argument("--result", default=RESULT_FILE)
    p.add_argument("--joints", default=None, help="6 wartosci [st] po przecinku w kolejnosci JOINT_NAMES")
    p.add_argument("--urdf", default=None)
    p.add_argument("--port", default=None)
    p.set_defaults(func=cmd_predict)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
