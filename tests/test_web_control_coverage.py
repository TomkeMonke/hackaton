"""Sciezka S w web_control.py (tryb POKRYCIE): kolejne nawroty na przemian w lewo i w prawo."""

import sys
import types

# web_control importuje websockets (nie ma go w CI ani w requirements-pinecone.txt); do logiki fazy niepotrzebny.
sys.modules.setdefault("websockets", types.ModuleType("websockets"))

import web_control  # noqa: E402


def _run(cycles):
    phase, turn_dir = "forward", 1
    turns = []
    for _ in range(cycles * 4):
        if phase in ("turn1", "turn2"):
            turns.append(turn_dir)
        phase, turn_dir = web_control.coverage_advance(phase, turn_dir)
    return phase, turns


def test_both_pivots_of_one_u_turn_go_the_same_way():
    _, turns = _run(3)
    assert turns[0] == turns[1]
    assert turns[2] == turns[3]
    assert turns[4] == turns[5]


def test_consecutive_u_turns_alternate():
    _, turns = _run(4)
    u_turns = turns[::2]
    assert u_turns == [1, -1, 1, -1]


def test_cycle_returns_to_forward():
    phase, _ = _run(2)
    assert phase == "forward"
