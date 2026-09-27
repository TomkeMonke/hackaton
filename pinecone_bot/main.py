"""
Uruchomienie zbieracza szyszek.

  python -m pinecone_bot.main --sim [--show] [--seed N] [--seconds S]   symulacja na laptopie
  python -m pinecone_bot.main --dry-run [--source frames/]               prawdziwa kamera (lub pliki), baza i ramie udawane
  python -m pinecone_bot.main --real [--config pinecone_config.json]     na Pi: kamera + baza + ramie z configu
  python -m pinecone_bot.main --real --no-arm                            na Pi: jazda bez ramienia (pasy, podjazd)

Kolejnosc na sprzecie: tools/calibrate_hsv.py -> tools/calibrate_target.py -> tools/record_waypoints.py -> --real
"""
from __future__ import annotations

import argparse
import sys
import time

from .brain import Brain, WallClock
from .config import Config
from .detector import HsvConeDetector
from .heading import make_heading
from .turn_loop import GyroTurnBase, wrap_with_turn_loop


class PrintBase:
    """Baza 'na sucho': tylko wypisuje komendy. Do testu kamery i regulatora bez jazdy."""

    def __init__(self):
        self.last = (0.0, 0.0)

    def set_speed(self, v, w):
        if (round(v, 3), round(w, 3)) != self.last:
            print(f"  base: v={v:+.3f} m/s  w={w:+.3f} rad/s")
            self.last = (round(v, 3), round(w, 3))

    def stop(self):
        self.set_speed(0.0, 0.0)

    def odometry(self):
        return None

    def close(self):
        pass


class PrintArm:
    def replay(self, name):
        print(f"  arm: replay({name})")
        return None

    def home(self):
        print("  arm: home()")

    def close(self):
        pass


def run_sim(args) -> int:
    from .sim import SimArmSimple, SimCamera, build_sim_robot, calibrate_grasps

    cfg = Config.load(args.config) if args.config else Config()
    if args.heading:
        cfg.heading.source = args.heading
    if args.hover:
        cfg.sim.hover = True
    if args.seed is not None:
        cfg.sim.seed = args.seed
    base, world, clock, heading = build_sim_robot(cfg)
    calibrate_grasps(cfg, world)
    camera = SimCamera(world)
    arm = SimArmSimple(world, clock)
    detector = HsvConeDetector(cfg.detector)

    show = None
    if args.show:
        import cv2

        def show(frame, dets, state, cmd):
            vis = HsvConeDetector.draw_debug(
                frame, dets, cfg.cx, cfg.grasps[len(cfg.grasps) // 2].target_row,
                f"{state.value} v={cmd.v:+.2f} w={cmd.w:+.2f} zebrane={world.collected}")
            cv2.imshow("pinecone sim", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

    print(f"szyszki: {[(round(c.x, 2), round(c.y, 2)) for c in world.cones]}")
    print(f"kalibracja: cx={cfg.cx:.1f} " + " ".join(f"{g.name}={g.target_row:.1f}" for g in cfg.grasps))
    brain = Brain(cfg, camera, detector, base, arm, clock=clock, log_path=args.log, on_frame=show,
                  heading=heading)
    try:
        stats = brain.run(max_seconds=args.seconds)
    except KeyboardInterrupt:
        stats = brain.stats
    print(f"czas sym: {clock.now():.1f} s, zebrane {world.collected}/{len(world.cones)}, "
          f"proby chwytu {world.grasp_attempts}, nieudane {world.failed_grasps}, klatek {stats.frames}")
    return 0 if world.remaining() == 0 else 1


def make_devices(cfg, dry: bool, no_arm: bool = False):
    """
    (base, arm) dla --dry-run / --real. no_arm: prawdziwa baza, ramie tylko drukuje - do testow jazdy
    (pasy, podjazd), gdy ramie jest zdjete albo uszkodzone. Port ramienia nie jest wtedy w ogole otwierany.
    """
    if dry:
        return PrintBase(), PrintArm()
    from .base import make_base
    base = make_base(cfg)
    if no_arm:
        return base, PrintArm()
    from .arm import make_arm
    return base, make_arm(cfg)


def run_real(args, dry: bool) -> int:
    from .camera import make_camera

    cfg = Config.load(args.config)
    if args.heading:
        cfg.heading.source = args.heading
    camera = make_camera(cfg, args.source)
    detector = HsvConeDetector(cfg.detector)
    base, arm = make_devices(cfg, dry, no_arm=args.no_arm)
    if args.no_arm and not dry:
        print("--no-arm: ramie nie dostaje zadnej komendy (tylko wydruk), baza jedzie naprawde")

    show = None
    if args.show:
        import cv2

        def show(frame, dets, state, cmd):
            vis = HsvConeDetector.draw_debug(
                frame, dets, cfg.cx, cfg.grasps[len(cfg.grasps) // 2].target_row,
                f"{state.value} v={cmd.v:+.2f} w={cmd.w:+.2f}")
            cv2.imshow("pinecone", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

    heading = make_heading(cfg, base)
    if heading is not None:
        # kurs startowy = kierunek pierwszego pasa: poczekaj na pierwsze dane, zanim robot ruszy
        t0 = time.monotonic()
        while heading.yaw() is None and time.monotonic() - t0 < cfg.heading.lost_s:
            time.sleep(0.1)
        ok = heading.yaw() is not None
        print(f"kurs: {cfg.heading.source} " + ("OK" if ok else "BRAK DANYCH - pasy z czasu, jesli sie nie pojawi"))
        base = wrap_with_turn_loop(base, heading, cfg)
        if isinstance(base, GyroTurnBase):
            print("obrot: petla predkosci na zyroskopie (heading.rate_*)")
    brain = Brain(cfg, camera, detector, base, arm, clock=WallClock(), log_path=args.log, on_frame=show,
                  heading=heading)
    print("Start. Ctrl+C zatrzymuje baze. Trzymaj wylacznik w rece.")
    try:
        brain.run(max_seconds=args.seconds)
    except KeyboardInterrupt:
        print("przerwano")
    finally:
        base.stop()
        base.close()
        if heading is not None:
            heading.close()
        arm.close()
        camera.close()
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="zbieracz szyszek")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--sim", action="store_true")
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--real", action="store_true")
    p.add_argument("--config", default=None, help="plik JSON z konfiguracja (domyslnie pinecone_config.json)")
    p.add_argument("--source", default=None, help="dry-run: plik/katalog/wideo zamiast RealSense")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--seconds", type=float, default=None, help="limit czasu")
    p.add_argument("--log", default="pinecone_log.csv")
    p.add_argument("--show", action="store_true", help="okno z podgladem")
    p.add_argument("--no-arm", action="store_true",
                   help="--real bez ramienia: baza jedzie, komendy ramienia tylko drukowane")
    p.add_argument("--hover", action="store_true",
                   help="sim: naped jak zmierzony hover (martwa strefa skretu, opozniony zyroskop)")
    p.add_argument("--heading", choices=["none", "phyphox", "odometry"], default=None,
                   help="zrodlo kursu dla pasow (nadpisuje heading.source z configu)")
    args = p.parse_args(argv)
    if args.sim:
        return run_sim(args)
    return run_real(args, dry=args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
