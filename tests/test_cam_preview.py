"""Testy tools/cam_preview.py - podglad MJPEG w procesie lerobot (atrapa kamery, bez lerobot i sprzetu)."""
from __future__ import annotations

import os
import sys
import urllib.request

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import cam_preview  # noqa: E402


class FakeCam:
    """Jak kamera lerobot: read_latest() podglada bufor, nie zabiera klatki."""

    def __init__(self, frame=None, color_mode="rgb"):
        self.frame = frame
        self.color_mode = color_mode
        self.is_connected = True
        self.reads = 0

    def read_latest(self, max_age_ms=500):
        self.reads += 1
        if self.frame is None:
            raise RuntimeError("has not captured any frames yet")
        return self.frame

    def __str__(self):
        return "FakeCam(wrist)"


@pytest.fixture(autouse=True)
def clean_registry():
    cam_preview.CAMERAS.clear()
    yield
    cam_preview.CAMERAS.clear()


def red_rgb():
    frame = np.zeros((48, 64, 3), np.uint8)
    frame[..., 0] = 255  # R w RGB
    return frame


def test_grab_jpeg_converts_lerobot_rgb_to_bgr():
    jpg = cam_preview.grab_jpeg(FakeCam(red_rgb()))
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    b, g, r = img[24, 32]
    assert r > 200 and b < 50  # czerwony zostal czerwony, nie niebieski


def test_grab_jpeg_returns_none_when_camera_not_ready():
    assert cam_preview.grab_jpeg(FakeCam(None)) is None


@pytest.fixture
def server():
    srv = cam_preview.start_server("127.0.0.1", 0, fps=50)
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()


def test_index_lists_registered_cameras(server):
    html = urllib.request.urlopen(server + "/", timeout=5).read().decode()
    assert "Brak kamer" in html
    cam_preview.register(FakeCam(red_rgb()))
    html = urllib.request.urlopen(server + "/", timeout=5).read().decode()
    assert "/cam/0.mjpg" in html and "FakeCam(wrist)" in html


def test_snapshot_and_stream_serve_jpeg(server):
    cam = FakeCam(red_rgb())
    cam_preview.register(cam)
    snap = urllib.request.urlopen(server + "/cam/0.jpg", timeout=5)
    assert snap.headers["Content-Type"] == "image/jpeg"
    assert snap.read()[:2] == b"\xff\xd8"

    stream = urllib.request.urlopen(server + "/cam/0.mjpg", timeout=5)
    assert stream.headers["Content-Type"].startswith("multipart/x-mixed-replace")
    chunk = stream.read(200)
    assert chunk.startswith(b"--frame") and b"image/jpeg" in chunk
    stream.close()


def test_unknown_camera_is_404(server):
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(server + "/cam/3.jpg", timeout=5)
    assert err.value.code == 404


def test_main_runs_entry_in_same_process_with_its_args():
    seen = {}

    def entry():
        seen["argv"] = list(sys.argv)
        return None

    old_argv = sys.argv
    try:
        rc = cam_preview.main(["--preview-port", "0", "--preview-host", "127.0.0.1",
                               "lerobot-rollout", "--robot.id=so101", "--fps=30"],
                              resolve=lambda name: entry, hook=lambda: seen.setdefault("hook", True))
    finally:
        sys.argv = old_argv
    assert rc == 0
    assert seen["hook"] is True
    assert seen["argv"] == ["lerobot-rollout", "--robot.id=so101", "--fps=30"]
