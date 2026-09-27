"""Testy tools/record_motion.py - subsample (bez sprzetu)."""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

from pinecone_bot.arm import JOINT_NAMES  # noqa: E402
from record_motion import FIRST_SECONDS, subsample  # noqa: E402


def _pose(v: float) -> dict:
    return {j: v for j in JOINT_NAMES}


def test_subsample_keeps_first_and_last_and_spacing():
    samples = [(i * 0.1, _pose(i)) for i in range(21)]  # 0.0 .. 2.0 s co 0.1 s
    wps = subsample(samples, every=0.5)
    assert wps[0].pose["elbow_flex"] == 0.0 and wps[0].seconds == FIRST_SECONDS
    assert wps[-1].pose["elbow_flex"] == 20.0
    assert [round(w.seconds, 2) for w in wps[1:]] == [0.5, 0.5, 0.5, 0.5]
    assert all(set(w.pose) == set(JOINT_NAMES) for w in wps)


def test_subsample_short_recording_has_two_points():
    samples = [(0.0, _pose(1.0)), (0.1, _pose(2.0))]
    wps = subsample(samples, every=0.5)
    assert len(wps) == 2 and wps[1].seconds >= 0.05


def test_subsample_empty():
    assert subsample([], every=0.25) == []
