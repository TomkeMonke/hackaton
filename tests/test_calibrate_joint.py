"""Testy matematyki tools/calibrate_joint.py (bez lerobot i bez sprzetu)."""
from __future__ import annotations

import json
import os
import sys

import pytest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, "tools"))

import calibrate_joint as cj  # noqa: E402


def test_unwrap_crosses_encoder_zero():
    assert cj.unwrap([4090, 4095, 3, 10]) == [4090, 4095, 4099, 4106]
    assert cj.unwrap([5, 0, 4093]) == [5, 0, -3]


def test_wrap_offset_range():
    assert cj.wrap_offset(-2119) == 1977
    assert cj.wrap_offset(2047) == 2047
    assert cj.wrap_offset(2048) == -2048
    assert -2048 <= cj.wrap_offset(12345) <= 2047


def test_propose_reproduces_2026_09_25_shoulder_fix():
    # Stary offset -701, Present przechodzil przez zero: -362..1621 (4095 -> 0).
    # Recznie policzona poprawka (legacy/arm_recordings/fix_shoulder_offset.py): offset 1977,
    # zakres 1056..3039 (potem z marginesem 1006..3089).
    sweep = list(range(4096 - 362, 4096, 7)) + list(range(0, 1622, 7))
    new = cj.propose(sweep, old_offset=-701)
    assert new["homing_offset"] == pytest.approx(1977, abs=2)
    assert new["range_min"] == pytest.approx(1056, abs=8)
    assert new["range_max"] == pytest.approx(3039, abs=8)
    assert (new["range_min"] + new["range_max"]) / 2 == pytest.approx(cj.CENTER, abs=2)


def test_propose_centered_range_is_stable():
    # zakres juz wysrodkowany przy offsecie 1977 -> offset i limity prawie bez zmian
    new = cj.propose(list(range(1006, 3090, 5)) + [3089], old_offset=1977)
    assert new["homing_offset"] == pytest.approx(1977, abs=1)
    assert new["range_min"] == pytest.approx(1006, abs=2)
    assert new["range_max"] == pytest.approx(3089, abs=2)


def test_propose_rejects_tiny_or_empty_sweep():
    with pytest.raises(ValueError):
        cj.propose([], old_offset=0)
    with pytest.raises(ValueError):
        cj.propose([2000, 2010, 2020], old_offset=0)


def test_degrees_like_lerobot():
    cal = {"range_min": 1006, "range_max": 3089}
    assert cj.deg_half_range(cal) == pytest.approx(91.6, abs=0.1)
    assert cj.present_to_deg(2047.5, cal) == pytest.approx(0.0)


def test_save_joint_touches_one_entry_and_backs_up(tmp_path):
    path = tmp_path / "so101.json"
    calib = {
        "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 1977, "range_min": 1006, "range_max": 3089},
        "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 1159, "range_min": 1168, "range_max": 3423},
    }
    path.write_text(json.dumps(calib))
    backup = cj.save_joint(str(path), "shoulder_lift", {"homing_offset": 100, "range_min": 1000, "range_max": 3000, "span": 2000})
    new = json.loads(path.read_text())
    assert new["shoulder_lift"] == {"id": 2, "drive_mode": 0, "homing_offset": 100, "range_min": 1000, "range_max": 3000}
    assert new["elbow_flex"] == calib["elbow_flex"]
    assert json.loads(open(backup).read()) == calib


def test_propose_many_skips_joint_that_did_not_move():
    samples = {"shoulder_lift": list(range(1006, 3090, 5)), "gripper": [2000, 2005, 2010]}
    proposals, errors = cj.propose_many(samples, {"shoulder_lift": 1977, "gripper": 0})
    assert set(proposals) == {"shoulder_lift"}
    assert set(errors) == {"gripper"}


def test_save_joints_writes_many_with_one_backup(tmp_path):
    path = tmp_path / "so101.json"
    calib = {
        "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": 1977, "range_min": 1006, "range_max": 3089},
        "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 1159, "range_min": 1168, "range_max": 3423},
        "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 2000, "range_max": 3000},
    }
    path.write_text(json.dumps(calib))
    news = {
        "shoulder_lift": {"homing_offset": 100, "range_min": 1000, "range_max": 3000, "span": 2000},
        "elbow_flex": {"homing_offset": -50, "range_min": 1100, "range_max": 2900, "span": 1800},
    }
    backup = cj.save_joints(str(path), news)
    new = json.loads(path.read_text())
    assert new["shoulder_lift"]["homing_offset"] == 100
    assert new["elbow_flex"] == {"id": 3, "drive_mode": 0, "homing_offset": -50, "range_min": 1100, "range_max": 2900}
    assert new["gripper"] == calib["gripper"]
    assert json.loads(open(backup).read()) == calib
    assert len(list(tmp_path.glob("so101.json.bak-*"))) == 1
