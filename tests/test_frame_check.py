"""Testy tools/frame_check.py - logika bez sprzetu (atrapa ramienia i kinematyki)."""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from pinecone_bot.arm import HOME_POSE, JOINT_NAMES  # noqa: E402
import frame_check as fc  # noqa: E402


class PlanarKin:
    """Plaskie 2-ogniwowe ramie w plaszczyznie X-Z: shoulder_lift i elbow_flex [st].

    Bark obraca ogniwo L1 wokol Y, lokiec dodaje L2. shoulder_pan obraca calosc
    wokol Z (X -> Y). Reszta stawow nie rusza koncowki.
    """

    L1 = 0.12
    L2 = 0.13

    def forward_kinematics(self, q_deg: np.ndarray) -> np.ndarray:
        pan, lift, elbow = np.radians(q_deg[0]), np.radians(q_deg[1]), np.radians(q_deg[2])
        r = self.L1 * np.cos(lift) + self.L2 * np.cos(lift + elbow)
        z = self.L1 * np.sin(lift) + self.L2 * np.sin(lift + elbow)
        T = np.eye(4)
        T[:3, 3] = [r * np.cos(pan), r * np.sin(pan), z]
        return T


def test_pose_to_array_order_and_missing_joint_zero():
    arr = fc.pose_to_array({"shoulder_pan": 10.0, "gripper": 5.0})
    assert arr.shape == (6,)
    assert arr[JOINT_NAMES.index("shoulder_pan")] == 10.0
    assert arr[JOINT_NAMES.index("gripper")] == 5.0
    assert arr[JOINT_NAMES.index("elbow_flex")] == 0.0


def test_describe_delta_words_and_threshold():
    assert fc.describe_delta(np.array([0.081, 0.001, -0.023])) == "+X 8.1 cm, -Z 2.3 cm (w dol)"
    assert fc.describe_delta(np.array([0.0, 0.002, 0.0])).startswith("brak ruchu")
    assert "+Z 1.0 cm (w gore)" in fc.describe_delta(np.array([0.0, 0.0, 0.01]))


def test_run_check_moves_each_joint_and_returns_to_start():
    kin = PlanarKin()
    start = {j: 0.0 for j in JOINT_NAMES}
    start["elbow_flex"] = 20.0
    arm = fc.FakeArm(start)
    moves: list[dict] = []
    orig_move = arm.move

    def spy_move(target):
        moves.append(dict(target))
        orig_move(target)

    arm.move = spy_move  # type: ignore[assignment]
    answers = iter(["", "t", "", "n", "", "uwaga: drga", "s", "q"])
    said: list[str] = []
    report = fc.run_check(arm, kin, fc.ARM_JOINTS, 15.0, ask=lambda _p: next(answers), say=said.append)

    joints = report["joints"]
    assert [r["joint"] for r in joints] == ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
    assert joints[0]["operator_ok"] is True and joints[1]["operator_ok"] is False
    assert joints[2]["operator_ok"] is None and joints[2]["note"] == "uwaga: drga"
    assert joints[3] == {"joint": "wrist_flex", "skipped": True}
    # kazdy sprawdzony staw: ruch o +15 i zmierzony +15 (atrapa), powrot do startu
    for r in joints[:3]:
        assert r["measured_delta_deg"] == 15.0
        assert r["pose_after"][r["joint"]] == start[r["joint"]] + 15.0
    assert arm.read() == start
    assert moves[-1] == start
    # pan przy wyprostowanym ramieniu rusza koncowka w bok (Y), bark i lokiec w X/Z
    assert "+Y" in joints[0]["description"] and "Z" not in joints[0]["description"]
    assert "Z" in joints[1]["description"]
    assert report["fk_urdf_zero_m"][0] > 0.2 and report["delta_deg"] == 15.0
    assert any("FK URDF zero" in s for s in said)


def test_summarize_lists_verdicts():
    report = {"joints": [
        {"joint": "shoulder_pan", "operator_ok": True, "description": "+Y 3 cm", "note": ""},
        {"joint": "elbow_flex", "operator_ok": False, "description": "-Z 2 cm", "note": "poszlo w gore"},
        {"joint": "wrist_roll", "skipped": True},
    ]}
    text = fc.summarize(report)
    assert "shoulder_pan: OK" in text
    assert "ZLE" in text and "poszlo w gore" in text
    assert "wrist_roll: pominiety" in text


def test_fake_arm_and_home_pose_fk_is_finite():
    arm = fc.FakeArm()
    assert arm.read() == HOME_POSE
    xyz = fc.fk_xyz(PlanarKin(), HOME_POSE)
    assert np.all(np.isfinite(xyz))
