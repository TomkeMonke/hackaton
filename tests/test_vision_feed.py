"""VisionFeed: liczenie szyszek, glebia, widoki, bledy kamery (bez sprzetu)."""
from __future__ import annotations

import json
import os
import sys
import threading
import urllib.request
from http.server import ThreadingHTTPServer

import cv2
import numpy as np

from pinecone_bot.config import Config
from pinecone_bot.detector import HsvConeDetector
from pinecone_bot.vision_feed import VisionFeed, depth_at

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# BGR koloru w progu z pinecone_config.json (H~150, S~110, V~120) na zielonym tle
CONE_BGR = (95, 50, 120)


def make_frame(cones, w=640, h=480, cut_top=False):
    img = np.full((h, w, 3), (40, 120, 40), np.uint8)
    for x, y in cones:
        cv2.ellipse(img, (x, y), (22, 32), 0, 0, 360, CONE_BGR, -1)
    if cut_top:
        cv2.ellipse(img, (500, 0), (22, 32), 0, 0, 360, CONE_BGR, -1)
    return img


class ListCamera:
    def __init__(self, frames):
        self.frames = list(frames)
        self.closed = False

    def read(self):
        item = self.frames.pop(0) if len(self.frames) > 1 else self.frames[0]
        if isinstance(item, Exception):
            raise item
        return item

    def close(self):
        self.closed = True


def detector():
    cfg = Config.load()
    det = HsvConeDetector(cfg.detector)
    return cfg, det


def test_frame_color_is_detected_by_current_config():
    _, det = detector()
    assert len(det.detect(make_frame([(200, 300)]))) == 1


def test_counts_full_and_partial_cones():
    _, det = detector()
    frame = make_frame([(150, 300), (320, 350), (480, 260)], cut_top=True)
    feed = VisionFeed(ListCamera([(frame, None)]), det)
    assert feed.step()
    s = feed.summary()
    assert s["count"] == 3
    assert s["partial"] == 1
    assert s["total"] == 4
    assert s["max_seen"] == 3
    assert s["nearest_m"] is None and s["has_depth"] is False
    assert s["frame"] == [640, 480]
    assert len(s["history"]) == 1 and s["history"][0][1] == 4


def test_depth_gives_nearest_distance_and_ignores_missing():
    _, det = detector()
    frame = make_frame([(150, 300), (450, 300)])
    depth = np.zeros((240, 320), np.float32)  # mniejsza glebia jest skalowana do obrazu
    depth[:, :160] = 0.62
    feed = VisionFeed(ListCamera([(frame, depth)]), det)
    feed.step()
    s = feed.summary()
    assert s["nearest_m"] == 0.62
    dists = sorted((d["dist_m"] is None, d["dist_m"]) for d in s["detections"])
    assert dists[0] == (False, 0.62) and dists[1][0] is True


def test_depth_at_edges():
    d = np.zeros((10, 10), np.float32)
    assert depth_at(None, 1, 1) is None
    assert depth_at(d, 5, 5) is None
    assert depth_at(d, 50, 5) is None
    d[4:7, 4:7] = 1.0
    assert depth_at(d, 5, 5) == 1.0


def test_camera_error_is_reported_and_cleared():
    _, det = detector()
    frame = make_frame([(200, 300)])
    feed = VisionFeed(ListCamera([RuntimeError("usb odpadl"), (frame, None)]), det)
    assert not feed.step()
    assert "usb odpadl" in feed.summary()["error"]
    assert feed.step()
    assert feed.summary()["error"] is None
    assert feed.summary()["errors"] == 1


def test_views_encode_jpeg_and_cache_per_frame():
    _, det = detector()
    feed = VisionFeed(ListCamera([(make_frame([(200, 300)]), None)]), det)
    assert feed.jpeg("overlay") == (0, None)
    feed.step()
    for view in ("overlay", "raw", "mask", "depth", "cos-innego"):
        seq, data = feed.jpeg(view)
        assert seq == 1 and data[:2] == b"\xff\xd8"
    assert feed.jpeg("mask")[1] is feed.jpeg("mask")[1]  # z pamieci podrecznej


def test_history_drops_old_entries():
    _, det = detector()
    t = [0.0]
    feed = VisionFeed(ListCamera([(make_frame([]), None)]), det, clock=lambda: t[0])
    for i in range(5):
        t[0] = i * 30.0
        feed.step()
    assert [n for _, n in feed.summary()["history"]] == [0, 0, 0]  # 60 s wstecz


def test_http_api_and_snapshot(tmp_path):
    sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))
    import vision_web

    _, det = detector()
    feed = VisionFeed(ListCamera([(make_frame([(200, 300)]), None)]), det, source="test")
    feed.step()
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), vision_web.make_handler(feed, str(tmp_path)))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{httpd.server_address[1]}"
    try:
        with urllib.request.urlopen(base + "/api/detections") as r:
            assert r.headers["Access-Control-Allow-Origin"] == "*"
            assert json.load(r)["count"] == 1
        with urllib.request.urlopen(base + "/frame.jpg?view=mask") as r:
            assert r.read()[:2] == b"\xff\xd8"
        req = urllib.request.Request(base + "/api/snapshot", method="POST", data=b"")
        with urllib.request.urlopen(req) as r:
            assert json.load(r)["ok"]
        assert len(list(tmp_path.glob("panel_*.png"))) == 1
    finally:
        httpd.shutdown()
