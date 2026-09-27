"""Testy tools/record_rgbd.py - funkcje bez kamery."""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from record_rgbd import (  # noqa: E402
    FrameWriter, MAX_YAW_DEG_S, depth_coverage, small_gray, warnings_for, write_calibration,
    write_meta, yaw_rate_deg_s,
)


def _texture(w=640, h=480, seed=1):
    rng = np.random.default_rng(seed)
    img = cv2.GaussianBlur(rng.integers(0, 255, (h, w), dtype=np.uint8), (0, 0), 3)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_depth_coverage_counts_only_valid_range():
    d = np.zeros((10, 10), np.uint16)
    d[:5] = 1000          # 50% dobrych
    d[5, :] = 100         # za blisko
    d[6, :] = 9000        # za daleko
    assert depth_coverage(d) == 0.5


def test_yaw_rate_from_horizontal_shift():
    img = _texture()
    shifted = np.roll(img, 40, axis=1)        # 40 px w 640 -> 10 px w obrazie 160
    fx = 600.0
    fx_small = fx * 160 / 640
    yaw = yaw_rate_deg_s(small_gray(img), small_gray(shifted), 0.1, fx_small)
    expected = np.degrees(np.arctan2(10, fx_small)) / 0.1
    assert abs(yaw - expected) < 0.2 * expected


def test_yaw_rate_zero_for_still_image():
    g = small_gray(_texture())
    assert yaw_rate_deg_s(g, g, 0.066, 150.0) < 1.0
    assert yaw_rate_deg_s(g, g, 0.0, 150.0) == 0.0


def test_warnings():
    assert warnings_for(5.0, 0.8, 0.07) == []
    w = warnings_for(MAX_YAW_DEG_S + 10, 0.1, 1.0)
    assert len(w) == 3 and "OBROT" in w[0] and "GLEBI" in w[1] and "DZIURA" in w[2]


def test_calibration_header_is_opencv4(tmp_path):
    p = tmp_path / "rs_color.yaml"
    write_calibration(str(p), 640, 480, 615.5, 615.25, 320.1, 240.2)
    text = p.read_text()
    assert text.startswith("%YAML:1.0\n")
    fs = cv2.FileStorage(str(p), cv2.FILE_STORAGE_READ)
    k = fs.getNode("camera_matrix").mat()
    assert fs.getNode("image_width").real() == 640
    assert np.allclose(k, [[615.5, 0, 320.1], [0, 615.25, 240.2], [0, 0, 1]])


def test_writer_and_meta_layout(tmp_path):
    out = str(tmp_path)
    w = FrameWriter(out)
    color = _texture(64, 48)
    depth = np.full((48, 64), 1234, np.uint16)
    assert w.put(1, color, depth) and w.put(2, color, depth)
    w.close()
    assert w.error is None
    assert sorted(os.listdir(os.path.join(out, "rgb"))) == ["000001.jpg", "000002.jpg"]
    back = cv2.imread(os.path.join(out, "depth", "000002.png"), cv2.IMREAD_UNCHANGED)
    assert back.dtype == np.uint16 and int(back[0, 0]) == 1234

    write_meta(out, [10.0, 10.5], 64, 48, "Intel RealSense D435", fast_s=0.3, gaps=1)
    lines = open(os.path.join(out, "stamps.txt")).read().splitlines()
    assert lines == ["10.000000", "10.500000"]
    readme = open(os.path.join(out, "README_rtabmap.txt")).read()
    assert "2 par" in readme and "D435" in readme and "2.0 Hz" in readme
