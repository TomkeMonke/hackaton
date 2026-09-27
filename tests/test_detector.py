"""
Testy detektora HSV i FileCamera na syntetycznych klatkach - bez kamery.

Uruchomienie z katalogu repo:
    python -m pytest tests/test_detector.py -q

Klatka: zielona murawa (BGR ~ (40,140,40)) z szumem gaussowskim, na niej
brazowe elipsy (BGR (30,60,110) -> HSV ok. (11,185,110), w domyslnym zakresie
HsvRange lo=(5,60,20) hi=(25,255,200)).
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinecone_bot.camera import FileCamera, make_camera  # noqa: E402
from pinecone_bot.config import Config, DetectorConfig  # noqa: E402
from pinecone_bot.detector import HsvConeDetector  # noqa: E402

W, H = 640, 480
GREEN = (40, 140, 40)
BROWN = (30, 60, 110)
TOL_PX = 3.0


def make_frame(ellipses, noise_sigma=6.0, seed=0):
    """ellipses: lista (cx, cy, ax, ay) - srodek i polosie w px."""
    rng = np.random.default_rng(seed)
    img = np.empty((H, W, 3), dtype=np.float32)
    img[:] = GREEN
    img += rng.normal(0.0, noise_sigma, size=img.shape).astype(np.float32)
    img = np.clip(img, 0, 255).astype(np.uint8)
    for cx, cy, ax, ay in ellipses:
        cv2.ellipse(img, (int(cx), int(cy)), (int(ax), int(ay)), 0, 0, 360, BROWN, -1)
    return img


def _nearest(dets, cx, cy):
    return min(dets, key=lambda d: (d.px - cx) ** 2 + (d.py - cy) ** 2)


def test_synthetic_brown_is_inside_default_range():
    hsv = cv2.cvtColor(np.uint8([[BROWN]]), cv2.COLOR_BGR2HSV)[0, 0]
    lo, hi = DetectorConfig().hsv.lo, DetectorConfig().hsv.hi
    assert all(lo[i] <= hsv[i] <= hi[i] for i in range(3)), hsv


def test_single_cone_center():
    det = HsvConeDetector(DetectorConfig())
    frame = make_frame([(300, 250, 18, 12)])
    dets = det.detect(frame)
    assert len(dets) == 1
    assert abs(dets[0].px - 300) <= TOL_PX
    assert abs(dets[0].py - 250) <= TOL_PX
    assert dets[0].area > 0
    x, y, bw, bh = dets[0].bbox
    assert x <= 300 <= x + bw and y <= 250 <= y + bh


def test_multiple_cones_sorted_lowest_first():
    det = HsvConeDetector(DetectorConfig())
    targets = [(120, 100, 14, 10), (480, 300, 20, 14), (330, 420, 16, 16)]
    frame = make_frame(targets, seed=3)
    dets = det.detect(frame)
    assert len(dets) == 3
    for cx, cy, _ax, _ay in targets:
        d = _nearest(dets, cx, cy)
        assert abs(d.px - cx) <= TOL_PX, (cx, cy, d)
        assert abs(d.py - cy) <= TOL_PX, (cx, cy, d)
    pys = [d.py for d in dets]
    assert pys == sorted(pys, reverse=True)
    assert abs(dets[0].py - 420) <= TOL_PX  # najnizsza w obrazie = pierwsza


def test_small_blob_ignored_by_min_area():
    cfg = DetectorConfig()
    det = HsvConeDetector(cfg)
    # polosie 4x4 -> ok. 50 px pola, ponizej domyslnego min_area_px=60
    frame = make_frame([(200, 200, 30, 20), (500, 100, 4, 4)], seed=5)
    dets = det.detect(frame)
    assert len(dets) == 1
    assert abs(dets[0].px - 200) <= TOL_PX
    # Ten sam obraz z nizszym progiem: mala plama wraca - czyli odrzucil ja
    # filtr pola, a nie morfologia.
    cfg.min_area_px = 10
    dets = HsvConeDetector(cfg).detect(frame)
    assert len(dets) == 2
    small = _nearest(dets, 500, 100)
    assert abs(small.px - 500) <= TOL_PX and abs(small.py - 100) <= TOL_PX


def test_pure_green_gives_nothing():
    det = HsvConeDetector(DetectorConfig())
    frame = make_frame([], noise_sigma=8.0, seed=7)
    assert det.detect(frame) == []
    assert det.last_mask is not None and int(det.last_mask.max()) == 0


def test_draw_debug_does_not_modify_input():
    det = HsvConeDetector(DetectorConfig())
    frame = make_frame([(300, 250, 18, 12)])
    before = frame.copy()
    dets = det.detect(frame)
    vis = HsvConeDetector.draw_debug(frame, dets, cx=320.0, target_row=400.0, text="x")
    assert vis.shape == frame.shape
    assert np.array_equal(frame, before)
    assert not np.array_equal(vis, frame)


def test_file_camera_directory_loops(tmp_path):
    a = make_frame([(100, 100, 10, 10)], seed=1)
    b = make_frame([(500, 400, 10, 10)], seed=2)
    cv2.imwrite(str(tmp_path / "frame_0000.png"), a)
    cv2.imwrite(str(tmp_path / "frame_0001.png"), b)
    (tmp_path / "notes.txt").write_text("ignored")

    cam = FileCamera(str(tmp_path))
    assert len(cam) == 2
    f1, d1 = cam.read()
    assert d1 is None and f1.shape == (H, W, 3) and f1.dtype == np.uint8
    assert cam.current_name == "frame_0000.png"
    assert np.array_equal(f1, a)
    f2, _ = cam.read()
    assert cam.current_name == "frame_0001.png"
    assert np.array_equal(f2, b)
    f3, _ = cam.read()  # zapetlenie
    assert cam.current_name == "frame_0000.png"
    assert np.array_equal(f3, a)
    cam.close()

    strict = FileCamera(str(tmp_path), loop=False)
    strict.read()
    strict.read()
    with pytest.raises(EOFError):
        strict.read()


def test_file_camera_single_image_and_make_camera(tmp_path):
    a = make_frame([(100, 100, 10, 10)], seed=1)
    path = tmp_path / "one.png"
    cv2.imwrite(str(path), a)
    cam = make_camera(Config(), str(path))
    assert isinstance(cam, FileCamera)
    f1, _ = cam.read()
    f2, _ = cam.read()
    assert np.array_equal(f1, a) and np.array_equal(f2, a)
    f1[0, 0] = 0  # kopia, nie wspolny bufor
    assert np.array_equal(cam.read()[0], a)
    with pytest.raises(ValueError):
        make_camera(Config(), "sim")
    with pytest.raises(FileNotFoundError):
        make_camera(Config(), str(tmp_path / "missing"))


def test_hue_range_wrapping_through_180_joins_both_ends():
    """lo H > hi H: szyszka z odcieniem 175 i 5 (obie strony zera) to jeden blob."""
    hsv = np.zeros((H, W, 3), dtype=np.uint8)
    hsv[:, :] = (60, 30, 140)                      # 'trawa': jasna, poza V hi
    hsv[200:260, 300:330] = (175, 80, 60)          # lewa polowa szyszki, H tuz pod 180
    hsv[200:260, 330:360] = (5, 80, 60)            # prawa polowa, H tuz nad 0
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    wrap = DetectorConfig(min_area_px=100)
    wrap.hsv.lo, wrap.hsv.hi = (140, 20, 20), (15, 130, 95)
    dets = HsvConeDetector(wrap).detect(bgr)
    assert len(dets) == 1
    assert abs(dets[0].px - 330) <= TOL_PX and abs(dets[0].py - 230) <= TOL_PX

    # Zwykly zakres 140..179 widzi tylko lewa polowe (srodek przesuniety w lewo).
    plain = DetectorConfig(min_area_px=100)
    plain.hsv.lo, plain.hsv.hi = (140, 20, 20), (179, 130, 95)
    dets = HsvConeDetector(plain).detect(bgr)
    assert len(dets) == 1
    assert dets[0].px < 320
