"""Testy tools/hand_eye_calib.py - matematyka i ArUco bez sprzetu (syntetyczne dane)."""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import hand_eye_calib as he  # noqa: E402


def _rot(axis: str, deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _synthetic(n: int = 12, seed: int = 3):
    rng = np.random.default_rng(seed)
    # prawdziwa kamera: 3 cm nad chwytakiem, 5 cm do tylu, pochylona 25 st
    X_true = he.make_T(_rot("x", 25.0) @ _rot("z", 5.0), [0.03, -0.05, 0.04])
    # marker nieruchomy na stole przed ramieniem
    T_bm = he.make_T(_rot("z", 10.0), [0.30, 0.02, 0.0])
    T_bg, T_cm = [], []
    for _ in range(n):
        R = _rot("z", rng.uniform(-40, 40)) @ _rot("y", rng.uniform(20, 70)) @ _rot("x", rng.uniform(-30, 30))
        t = np.array([rng.uniform(0.15, 0.30), rng.uniform(-0.12, 0.12), rng.uniform(0.15, 0.30)])
        Tbg = he.make_T(R, t)
        T_bg.append(Tbg)
        T_cm.append(he.inv_T(X_true) @ he.inv_T(Tbg) @ T_bm)
    return X_true, T_bm, T_bg, T_cm


def test_inv_T_and_make_T_roundtrip():
    T = he.make_T(_rot("y", 33.0), [0.1, -0.2, 0.3])
    assert np.allclose(T @ he.inv_T(T), np.eye(4), atol=1e-12)
    assert he.rotation_angle_deg(_rot("x", 40.0)) == 40.0 or abs(he.rotation_angle_deg(_rot("x", 40.0)) - 40.0) < 1e-9


def test_rvec_tvec_to_T_matches_rodrigues():
    rvec = np.array([0.1, -0.2, 0.3])
    T = he.rvec_tvec_to_T(rvec, [1.0, 2.0, 3.0])
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    assert np.allclose(T[:3, :3], R) and np.allclose(T[:3, 3], [1.0, 2.0, 3.0])


def test_solve_hand_eye_recovers_camera_pose_on_synthetic_data():
    X_true, T_bm, T_bg, T_cm = _synthetic()
    X = he.solve_hand_eye(T_bg, T_cm)
    assert np.allclose(X[:3, 3], X_true[:3, 3], atol=1e-4)
    assert he.rotation_angle_deg(X[:3, :3].T @ X_true[:3, :3]) < 0.01
    res = he.residuals(T_bg, T_cm, X)
    assert res["pos_rms_mm"] < 0.5 and res["rot_rms_deg"] < 0.05
    assert np.allclose(res["marker_in_base_mean_m"], T_bm[:3, 3], atol=1e-3)


def test_residuals_grow_with_wrong_X():
    X_true, _, T_bg, T_cm = _synthetic()
    X_bad = he.make_T(X_true[:3, :3], X_true[:3, 3] + [0.02, 0.0, 0.0])
    res = he.residuals(T_bg, T_cm, X_bad)
    assert res["pos_rms_mm"] > 5.0
    assert len(res["per_sample_mm"]) == len(T_bg)


def test_solve_hand_eye_needs_three_pairs():
    X_true, _, T_bg, T_cm = _synthetic(n=2)
    try:
        he.solve_hand_eye(T_bg, T_cm)
    except ValueError:
        return
    raise AssertionError("oczekiwano ValueError dla 2 par")


def test_camera_pose_and_describe():
    X = he.make_T(np.eye(3), [0.0, 0.0, 0.05])
    T_bg = he.make_T(_rot("y", 90.0), [0.2, 0.0, 0.1])  # chwytak patrzy w dol: jego +Z -> -X? sprawdzamy tylko format
    T_bc = he.camera_pose(T_bg, X)
    assert np.allclose(T_bc[:3, 3], T_bg[:3, 3] + T_bg[:3, :3] @ [0.0, 0.0, 0.05])
    text = he.describe_camera(T_bc)
    assert text.startswith("kamera w bazie: X ") and "patrzy wzdluz" in text


def test_rpy_deg_of_pure_rotations():
    assert np.allclose(he.rpy_deg(_rot("x", 20.0)), [20.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(he.rpy_deg(_rot("z", -35.0)), [0.0, 0.0, -35.0], atol=1e-9)


def test_marker_image_is_detected_and_pose_is_sane():
    marker_len = 0.05
    img = he.make_marker_image(px=300)  # 300 px czarnego kwadratu + biala ramka = 375 px
    frame = np.full((480, 640), 255, dtype=np.uint8)
    y0, x0 = 50, 130
    frame[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img
    bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
    det = he.detect_marker(bgr, K, np.zeros(5), marker_len)
    assert det is not None
    rvec, tvec, corners = det
    # marker 300 px szeroki przy f=600 px i boku 5 cm -> odleglosc ok. 0.10 m
    assert 0.08 < tvec[2] < 0.12
    assert corners.shape == (4, 2)
    # brak markera -> None
    assert he.detect_marker(np.full((480, 640, 3), 255, dtype=np.uint8), K, np.zeros(5), marker_len) is None


def test_intrinsics_to_K():
    class I:
        fx, fy, ppx, ppy = 610.0, 611.0, 318.0, 242.0

    K = he.intrinsics_to_K(I())
    assert K[0, 0] == 610.0 and K[1, 1] == 611.0 and K[0, 2] == 318.0 and K[1, 2] == 242.0 and K[2, 2] == 1.0
