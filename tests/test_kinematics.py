"""Testy pinecone_bot.kinematics i jogu XYZ w panelu ramienia (bez sprzetu, bez lerobot)."""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot.arm_panel import FAKE_CALIBRATION, FAKE_NORM_MODES, ArmPanel, FakeSO101, limits_from_calibration  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402
from pinecone_bot.kinematics import IKError, So101Kinematics  # noqa: E402

LIMITS = limits_from_calibration(FAKE_CALIBRATION, FAKE_NORM_MODES)
BENT = {"shoulder_pan": 10.0, "shoulder_lift": -30.0, "elbow_flex": 50.0, "wrist_flex": 40.0,
        "wrist_roll": 0.0, "gripper": 30.0}


def test_zero_pose_is_arm_stretched_forward():
    xyz, pitch = So101Kinematics().tcp({})
    assert xyz[0] == pytest.approx(0.391, abs=0.002)
    assert xyz[1] == pytest.approx(0.0, abs=0.002)
    assert xyz[2] == pytest.approx(0.226, abs=0.002)
    assert pitch == pytest.approx(0.0, abs=0.5)


def test_urdf_directions():
    k = So101Kinematics()
    z0 = k.tcp({})[0]
    assert k.tcp({"shoulder_pan": 20})[0][1] < z0[1] - 0.05      # pan + = w prawo (y ujemne)
    assert k.tcp({"shoulder_lift": 20})[0][2] < z0[2] - 0.05     # lift + = ramie w dol


def test_offset_and_sign_shift_zero():
    k = So101Kinematics(sign={"elbow_flex": -1}, offset_deg={"shoulder_lift": 15.0, "elbow_flex": -20.0})
    ref = So101Kinematics()
    # lerobot lift -15 -> URDF 0; lerobot elbow -20 -> URDF -(-20) - 20 = 0
    assert np.allclose(k.fk({"shoulder_lift": -15.0, "elbow_flex": -20.0}), ref.fk({}), atol=1e-9)


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("sign", [1, -1])
def test_jog_moves_tcp_along_axis_and_keeps_pitch(axis, sign):
    k = So101Kinematics()
    xyz0, pitch0 = k.tcp(BENT)
    delta = [0.0, 0.0, 0.0]
    delta[axis] = sign * 0.01
    new = k.jog_xyz(BENT, delta, LIMITS)
    assert set(new) == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"}
    xyz1, pitch1 = k.tcp({**BENT, **new})
    assert np.allclose(xyz1 - xyz0, delta, atol=0.001)
    assert pitch1 == pytest.approx(pitch0, abs=0.5)


def test_jog_out_of_reach_raises():
    k = So101Kinematics()
    with pytest.raises(IKError):
        k.jog_xyz({"wrist_roll": 0.0, "shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                   "wrist_flex": 0.0}, [0.1, 0.0, 0.0], LIMITS)


# --- panel ---------------------------------------------------------------------

class FakeClock:
    def __init__(self):
        self.t = 0.0

    def now(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s


def make_panel(tmp_path, kin=True):
    cfg = Config()
    clock = FakeClock()
    arm = FakeSO101(pose=dict(BENT))
    panel = ArmPanel(arm, cfg, LIMITS, sleep=clock.sleep, clock=clock.now, manual_only=True,
                     kinematics=So101Kinematics() if kin else None, config_path=str(tmp_path / "cfg.json"))
    assert panel.maybe_read()
    return panel, arm


def test_panel_jog_xyz_moves_tcp_up(tmp_path):
    panel, arm = make_panel(tmp_path)
    z0 = panel.tcp()["z"]
    assert panel.submit({"cmd": "jog_xyz", "axis": "z", "step_mm": 10}) == (True, "w kolejce")
    assert panel.process_one()
    assert panel.last_error is None, panel.last_error
    panel._last_read_attempt = -1e9
    assert panel.maybe_read()
    assert panel.tcp()["z"] == pytest.approx(z0 + 10.0, abs=1.0)
    assert arm.pose["gripper"] == BENT["gripper"]


def test_panel_rejects_bad_xyz_commands(tmp_path):
    panel, _ = make_panel(tmp_path)
    assert not panel.submit({"cmd": "jog_xyz", "axis": "w", "step_mm": 10})[0]
    assert not panel.submit({"cmd": "jog_xyz", "axis": "z", "step_mm": 7})[0]
    no_kin, _ = make_panel(tmp_path, kin=False)
    assert not no_kin.submit({"cmd": "jog_xyz", "axis": "z", "step_mm": 10})[0]
    assert no_kin.snapshot()["xyz_steps"] == []
    assert no_kin.snapshot()["tcp"] is None


def test_panel_snapshot_has_tcp(tmp_path):
    panel, _ = make_panel(tmp_path)
    snap = panel.snapshot()
    assert snap["xyz_steps"] == [5.0, 10.0, 20.0]
    assert set(snap["tcp"]) == {"x", "y", "z", "pitch"}


def test_urdf_zero_saves_offsets_and_makes_current_pose_zero(tmp_path):
    panel, _ = make_panel(tmp_path)
    ok, msg = panel.submit({"cmd": "urdf_zero"})
    assert ok, msg
    saved = json.loads((tmp_path / "cfg.json").read_text())
    assert saved["arm"]["urdf_offset_deg"]["shoulder_lift"] == pytest.approx(30.0)
    assert saved["arm"]["urdf_offset_deg"]["elbow_flex"] == pytest.approx(-50.0)
    assert "wrist_roll" not in saved["arm"]["urdf_offset_deg"]
    # po zerowaniu biezaca poza = ramie wyprostowane (pan tez wyzerowany)
    tcp = panel.tcp()
    assert tcp["x"] == pytest.approx(391, abs=3)
    assert tcp["z"] == pytest.approx(226, abs=3)
    # config wczytany od nowa daje te same offsety
    cfg = Config.load(str(tmp_path / "cfg.json"))
    assert cfg.arm.urdf_offset_deg == saved["arm"]["urdf_offset_deg"]
