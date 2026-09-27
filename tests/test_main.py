"""Wybor urzadzen w main (--dry-run / --real / --real --no-arm), bez sprzetu."""
from __future__ import annotations

import pinecone_bot.arm as arm_mod
import pinecone_bot.base as base_mod
from pinecone_bot.config import Config
from pinecone_bot.main import PrintArm, PrintBase, make_devices


class FakeBase:
    pass


def _no_arm_allowed(cfg, **kw):
    raise AssertionError("--no-arm nie moze otwierac portu ramienia")


def test_no_arm_uses_real_base_and_print_arm(monkeypatch):
    monkeypatch.setattr(base_mod, "make_base", lambda cfg: FakeBase())
    monkeypatch.setattr(arm_mod, "make_arm", _no_arm_allowed)
    base, arm = make_devices(Config(), dry=False, no_arm=True)
    assert isinstance(base, FakeBase)
    assert isinstance(arm, PrintArm)
    assert arm.replay("grasp_mid") is None   # brain sprawdzi chwyt kamera, jak w --dry-run


def test_real_uses_real_arm(monkeypatch):
    sentinel = object()
    monkeypatch.setattr(base_mod, "make_base", lambda cfg: FakeBase())
    monkeypatch.setattr(arm_mod, "make_arm", lambda cfg, **kw: sentinel)
    base, arm = make_devices(Config(), dry=False)
    assert arm is sentinel


def test_dry_run_touches_no_hardware(monkeypatch):
    monkeypatch.setattr(base_mod, "make_base", lambda cfg: (_ for _ in ()).throw(AssertionError("baza")))
    monkeypatch.setattr(arm_mod, "make_arm", _no_arm_allowed)
    base, arm = make_devices(Config(), dry=True, no_arm=True)
    assert isinstance(base, PrintBase) and isinstance(arm, PrintArm)
