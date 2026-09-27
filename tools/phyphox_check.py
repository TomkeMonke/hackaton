"""
Sprawdzenie kursu z telefonu (phyphox) zanim robot pojedzie po nim pasy. Nic nie rusza.

Telefon: phyphox -> "Gyroscope (rotation rate)" -> menu (trzy kropki) -> "Allow remote access" -> start.
Telefon plasko na bazie, ekranem do gory. Obroc robota RECZNIE o 90 st w lewo: kurs ma urosnac o ~+90.
Jesli maleje, wpisz "heading": {"sign": -1.0} w pinecone_config.json.

  python tools/phyphox_check.py                       # adres z pinecone_config.json (heading.phyphox_url)
  python tools/phyphox_check.py --url http://172.20.10.1:8080
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pinecone_bot.config import Config  # noqa: E402
from pinecone_bot.heading import PhyphoxGyro  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default=None, help="adres phyphox (domyslnie heading.phyphox_url z configu)")
    p.add_argument("--config", default=None)
    p.add_argument("--seconds", type=float, default=60.0)
    args = p.parse_args()
    cfg = Config.load(args.config)
    if args.url:
        cfg.heading.phyphox_url = args.url
    print(f"phyphox: {cfg.heading.phyphox_url}  znak: {cfg.heading.sign:+.0f}  Ctrl+C konczy")
    gyro = PhyphoxGyro(cfg.heading).start()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            time.sleep(0.2)
            yaw = gyro.yaw()
            if yaw is None:
                print(f"kurs: brak danych  (probek {gyro.samples}, blad: {gyro.last_error})")
            else:
                print(f"kurs: {math.degrees(yaw):+8.1f} st   (probek {gyro.samples})")
    except KeyboardInterrupt:
        pass
    finally:
        gyro.close()
    return 0 if gyro.samples else 2


if __name__ == "__main__":
    sys.exit(main())
