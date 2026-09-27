"""tools/estop_server.py musi zabijac kazdy program, ktory jezdzi baza."""
from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
pytest.importorskip("serial")
import estop_server  # noqa: E402


@pytest.mark.parametrize("cmd", [
    "/home/robot/hackaton/.venv/bin/python -m pinecone_bot.zygzak --real --config x.json",
    "/home/robot/hackaton/.venv/bin/python -m pinecone_bot.main --real --no-arm",
    ".venv/bin/python tools/calibrate_drive.py --write",
])
def test_driving_programs_are_targets(cmd):
    assert any(t in cmd for t in estop_server.TARGETS)
