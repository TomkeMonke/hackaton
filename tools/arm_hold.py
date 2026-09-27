"""
Ustaw ramie w nagranej pozie (motions/<nazwa>.json) i trzymaj ja (moment w serwach) do Ctrl+C.

Po co: nagrywanie mapy (tools/record_rgbd.py) musi isc z kamera w tej samej pozie, w ktorej zygzak
potem sie lokalizuje ("patrz" = kamera poziomo na sciany). ./arm.sh move po ruchu puszcza moment.

  python tools/arm_hold.py patrz --config ~/hackaton/pinecone_config.json
Teleop i panel ramienia musza byc zamkniete (port ramienia na wylacznosc).
"""
from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pinecone_bot.arm import make_arm  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", default="patrz", help="motions/<name>.json")
    ap.add_argument("--config", default=None, help="pinecone_config.json robota")
    args = ap.parse_args()
    path = os.path.expanduser(args.config) if args.config else Config.default_path()
    if not os.path.exists(path):
        print("nie ma configu %s - podaj --config ~/hackaton/pinecone_config.json" % path)
        return 2
    cfg = Config.load(path)
    if cfg.arm.driver != "waypoints":
        print("arm.driver w %s to '%s', nie 'waypoints' - to nie config robota" % (path, cfg.arm.driver))
        return 2
    arm = make_arm(cfg, home_on_empty=False)
    try:
        arm.replay(args.name)
        print("ramie w pozie '%s', trzymam. Ctrl+C puszcza (ramie opadnie)." % args.name, flush=True)
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
