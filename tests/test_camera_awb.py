"""
Testy startu RealSenseCamera (rozgrzewka, blokada AWB/ekspozycji) na atrapie
pyrealsense2 - bez kamery.

Uruchomienie z katalogu repo:
    python -m pytest tests/test_camera_awb.py -q
"""
from __future__ import annotations

import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinecone_bot.camera import RealSenseCamera  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402


class _Range:
    def __init__(self, lo, hi):
        self.min, self.max = lo, hi


class FakeColorSensor:
    def __init__(self):
        self.options = {"enable_auto_white_balance": 1, "white_balance": 4600.0,
                        "enable_auto_exposure": 1, "exposure": 156.0}
        self.ranges = {"white_balance": _Range(2800, 6500), "exposure": _Range(1, 10000)}
        self.calls = []

    def supports(self, opt):
        return opt in self.options

    def get_option(self, opt):
        return self.options[opt]

    def set_option(self, opt, value):
        self.calls.append((opt, value))
        self.options[opt] = value

    def get_option_range(self, opt):
        return self.ranges[opt]


class FakeColorFrame:
    def __init__(self, meta):
        self.meta = meta

    def __bool__(self):
        return True

    def supports_frame_metadata(self, key):
        return key in self.meta

    def get_frame_metadata(self, key):
        return self.meta[key]


class FakeFrames:
    def __init__(self, meta):
        self._color = FakeColorFrame(meta)

    def get_color_frame(self):
        return self._color


def make_fake_rs(meta, sensor):
    rs = types.SimpleNamespace()
    rs.option = types.SimpleNamespace(
        enable_auto_white_balance="enable_auto_white_balance", white_balance="white_balance",
        enable_auto_exposure="enable_auto_exposure", exposure="exposure", laser_power="laser_power")
    rs.frame_metadata_value = types.SimpleNamespace(white_balance="md_wb", actual_exposure="md_exp")
    rs.stream = types.SimpleNamespace(color="color", depth="depth")
    rs.format = types.SimpleNamespace(bgr8="bgr8", z16="z16")
    state = {"waits": 0}

    class Intr:
        fx = fy = 600.0
        ppx, ppy, width, height = 320.0, 240.0, 640, 480

    class Profile:
        def get_device(self):
            dev = types.SimpleNamespace()
            dev.query_sensors = lambda: [sensor]
            return dev

        def get_stream(self, _s):
            vsp = types.SimpleNamespace(get_intrinsics=lambda: Intr())
            return types.SimpleNamespace(as_video_stream_profile=lambda: vsp)

    class Pipeline:
        def start(self, _cfg):
            return Profile()

        def wait_for_frames(self, _t):
            state["waits"] += 1
            return FakeFrames(meta)

        def stop(self):
            pass

    rs.pipeline = Pipeline
    rs.config = lambda: types.SimpleNamespace(enable_stream=lambda *a: None)
    rs.context = lambda: types.SimpleNamespace(query_devices=lambda: [object()])
    rs.align = lambda _s: None
    return rs, state


@pytest.fixture
def fake(monkeypatch):
    def _make(meta):
        sensor = FakeColorSensor()
        rs, state = make_fake_rs(meta, sensor)
        monkeypatch.setitem(sys.modules, "pyrealsense2", rs)
        return sensor, state
    return _make


def test_warmup_frames_from_config(fake):
    sensor, state = fake({})
    cfg = Config()
    cfg.camera.warmup_frames = 45
    RealSenseCamera(cfg, depth=False)
    assert state["waits"] == 45
    assert sensor.calls == []  # lock_auto domyslnie wylaczone, auto juz wlaczone - nic do zmiany


def test_no_lock_reenables_auto_left_off_by_previous_process(fake):
    """Opcje zostaja w kamerze po poprzednim procesie: zamrozona ekspozycja trzeba odmrozic."""
    sensor, _ = fake({})
    sensor.options["enable_auto_exposure"] = 0
    sensor.options["enable_auto_white_balance"] = 0
    RealSenseCamera(Config(), depth=False, warmup_frames=1)
    assert sensor.options["enable_auto_exposure"] == 1
    assert sensor.options["enable_auto_white_balance"] == 1


def test_warmup_frames_argument_overrides_config(fake):
    _, state = fake({})
    RealSenseCamera(Config(), depth=False, warmup_frames=3)
    assert state["waits"] == 3


def test_lock_auto_freezes_values_from_last_frame(fake):
    sensor, _ = fake({"md_wb": 5200, "md_exp": 300})
    cfg = Config()
    cfg.camera.lock_auto = True
    cam = RealSenseCamera(cfg, depth=False, warmup_frames=2)
    assert sensor.options["enable_auto_white_balance"] == 0
    assert sensor.options["enable_auto_exposure"] == 0
    assert sensor.options["white_balance"] == 5200
    assert sensor.options["exposure"] == 300
    assert cam.locked == {"white_balance": 5200.0, "exposure": 300.0}
    # auto wylaczone PRZED wpisaniem wartosci, inaczej kamera by ja nadpisala
    assert sensor.calls.index(("enable_auto_white_balance", 0)) < sensor.calls.index(("white_balance", 5200.0))


def test_lock_auto_without_metadata_uses_current_option_and_clamps(fake):
    sensor, _ = fake({})
    sensor.options["white_balance"] = 9000.0  # poza zakresem -> przyciete
    cfg = Config()
    cfg.camera.lock_auto = True
    cam = RealSenseCamera(cfg, depth=False, warmup_frames=1)
    assert cam.locked["white_balance"] == 6500
    assert cam.locked["exposure"] == 156.0


def test_config_json_roundtrip(tmp_path):
    p = tmp_path / "c.json"
    p.write_text('{"camera": {"warmup_frames": 10, "lock_auto": true}}', encoding="utf-8")
    cfg = Config.load(str(p))
    assert cfg.camera.warmup_frames == 10
    assert cfg.camera.lock_auto is True
