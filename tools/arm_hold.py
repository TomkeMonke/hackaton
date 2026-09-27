"""
Ustaw ramie w nagranej pozie (motions/<nazwa>.json) i trzymaj ja (moment w serwach) do Ctrl+C.

Po co: nagrywanie mapy (tools/record_rgbd.py) musi isc z kamera w tej samej pozie, w ktorej zygzak
potem sie lokalizuje ("patrz" = kamera poziomo na sciany). ./arm.sh move po ruchu puszcza moment.

  python tools/arm_hold.py patrz
Teleop i panel ramienia musza byc zamkniete (port ramienia na wylacznosc).
"""
from __future__ import annotations

import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from pinecone_bot.arm import make_arm  # noqa: E402
from pinecone_bot.config import Config  # noqa: E402


def main() -> int:
    name = sys.argv[1] if len(sys.argv) > 1 else "patrz"
    cfg = Config.load(os.environ.get("PINECONE_CONFIG"))
    arm = make_arm(cfg, home_on_empty=False)
    try:
        arm.replay(name)
        print("ramie w pozie '%s', trzymam. Ctrl+C puszcza (ramie opadnie)." % name, flush=True)
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        arm.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
