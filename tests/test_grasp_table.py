"""Tabela chwytow z uczenia (pinecone_bot/grasp_table.py) i WaypointArm.play."""
from __future__ import annotations

import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from pinecone_bot import arm as arm_mod  # noqa: E402
from pinecone_bot.arm import GRIPPER_OPEN, JOINT_NAMES  # noqa: E402
from pinecone_bot.detector import Detection  # noqa: E402
from pinecone_bot.grasp_table import (  # noqa: E402
    GraspSample,
    grasp_motion,
    load_table,
    nearest_sample,
    pick_detection,
    save_table,
)
from test_arm import FakeArm, waypoint_arm  # noqa: E402  (pytest dodaje tests/ do sys.path)


def pose(**kw) -> dict:
    return {**dict(arm_mod.HOME_POSE), **kw}


def sample(px, py, lift=10.0) -> GraspSample:
    return GraspSample(px=px, py=py, pre=pose(shoulder_lift=lift), grasp=pose(shoulder_lift=lift - 20), note=f"{px},{py}")


def test_table_roundtrip_is_ascii(tmp_path):
    path = str(tmp_path / "grasp_table.json")
    assert load_table(path) == []
    save_table(path, [sample(100, 200), sample(300, 250)])
    with open(path, "rb") as fh:
        fh.read().decode("ascii")
    loaded = load_table(path)
    assert [(s.px, s.py) for s in loaded] == [(100, 200), (300, 250)]
    assert set(loaded[0].grasp) == set(JOINT_NAMES)


def test_load_rejects_missing_joint(tmp_path):
    path = tmp_path / "grasp_table.json"
    path.write_text('{"samples": [{"px": 1, "py": 2, "pre": {"gripper": 1}, "grasp": {}}]}', encoding="ascii")
    with pytest.raises(ValueError):
        load_table(str(path))


def test_nearest_sample_respects_max_px():
    table = [sample(100, 200), sample(300, 250)]
    s, d = nearest_sample(table, 290, 240, max_px=80)
    assert s.px == 300 and d == pytest.approx(14.14, abs=0.01)
    assert nearest_sample(table, 600, 50, max_px=80) is None
    assert nearest_sample([], 0, 0, max_px=80) is None


def test_pick_detection_skips_partial_and_takes_largest():
    small = Detection(px=10, py=10, area=100, bbox=(0, 0, 10, 10))
    big_cut = Detection(px=20, py=20, area=900, bbox=(0, 0, 30, 30), partial=True)
    big = Detection(px=30, py=30, area=400, bbox=(0, 0, 20, 20))
    assert pick_detection([small, big_cut, big]) is big
    assert pick_detection([big_cut]) is None


def test_grasp_motion_opens_above_checks_on_squeeze_and_lifts():
    m = grasp_motion(sample(100, 200), squeeze=0.0)
    labels = [wp.label for wp in m.waypoints]
    assert labels == ["nad szyszka", "w dol", "zacisk", "w gore"]
    assert m.waypoints[0].pose["gripper"] == GRIPPER_OPEN
    assert m.waypoints[1].pose["gripper"] == GRIPPER_OPEN
    assert [wp.check_gripper for wp in m.waypoints] == [False, False, True, False]
    assert m.waypoints[3].pose["gripper"] == 0.0  # trzyma szyszke w gore
    assert grasp_motion(sample(1, 1), slow=2.0).total_seconds == pytest.approx(2 * m.total_seconds)


def test_play_empty_gripper_goes_home():
    fake = FakeArm(gripper_reading=1.0)  # po zacisku odczyt ~pusto
    ctl, _ = waypoint_arm(fake)
    assert ctl.play(grasp_motion(sample(100, 200))) is False
    assert fake.pose["shoulder_lift"] == pytest.approx(arm_mod.HOME_POSE["shoulder_lift"])


def test_play_holding_returns_true():
    fake = FakeArm(gripper_reading=20.0)
    ctl, _ = waypoint_arm(fake)
    assert ctl.play(grasp_motion(sample(100, 200, lift=10.0))) is True
    assert fake.pose["shoulder_lift"] == pytest.approx(10.0)  # konczy nad szyszka, HOME robi wolajacy
