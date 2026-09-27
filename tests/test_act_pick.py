"""Testy tools/act_pick.py - skladanie komend etapow (bez sprzetu i bez lerobot)."""
from __future__ import annotations

import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from act_pick import TASK, drop_cmd, main, rollout_cmd, with_preview  # noqa: E402


def test_rollout_keeps_torque_and_matches_dataset_camera():
    cmd = rollout_cmd("/m/act", 12)
    assert "--policy.path=/m/act" in cmd
    assert "--robot.disable_torque_on_disconnect=false" in cmd
    assert "--duration=12" in cmd
    assert "--device=cpu" in cmd  # Pi bez CUDA, nawet gdy wagi uczone na cuda
    assert f"--task={TASK}" in cmd
    cams = next(c for c in cmd if c.startswith("--robot.cameras="))
    assert "wrist:" in cams and "width: 640" in cams and "height: 480" in cams


def test_rollout_runs_through_camera_preview_by_default(capsys):
    assert main(["--policy", "p", "--dry-run", "--skip-drop"], run=None) == 0
    line = capsys.readouterr().out.splitlines()[0]
    assert "tools/cam_preview.py --preview-port=8081 lerobot-rollout --strategy.type=base" in line


def test_with_preview_keeps_lerobot_args_and_can_be_disabled():
    cmd = rollout_cmd("/m/act", 12)
    wrapped = with_preview(cmd, 9000)
    assert wrapped[1:4] == ["tools/cam_preview.py", "--preview-port=9000", "lerobot-rollout"]
    assert wrapped[4:] == cmd[1:]
    assert with_preview(cmd, 0) == cmd


def test_drop_goes_home_after():
    cmd = drop_cmd(port="/dev/x")
    assert cmd[1:] == ["tools/arm_play.py", "--motion", "drop_box", "--port", "/dev/x", "--home-after"]


def test_main_runs_stages_in_order_and_repeats():
    calls = []

    def run(cmd, cwd=None):
        calls.append("lerobot-rollout" if "lerobot-rollout" in cmd else cmd[1])
        return SimpleNamespace(returncode=0)

    assert main(["--policy", "p", "--repeat", "2"], run=run) == 0
    assert calls == ["lerobot-rollout", "tools/arm_play.py", "lerobot-rollout", "tools/arm_play.py"]


def test_main_stops_on_failed_grasp_stage():
    calls = []

    def run(cmd, cwd=None):
        calls.append(cmd)
        return SimpleNamespace(returncode=3)

    assert main(["--policy", "p"], run=run) == 3
    assert len(calls) == 1  # bez wrzutu, gdy rollout padl


def test_dry_run_and_skip_drop_run_nothing(capsys):
    def run(cmd, cwd=None):
        raise AssertionError("dry-run nie moze nic uruchamiac")

    assert main(["--policy", "p", "--dry-run", "--skip-drop"], run=run) == 0
    out = capsys.readouterr().out
    assert "lerobot-rollout" in out and "arm_play" not in out
