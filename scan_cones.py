"""
Skan szyszek na postoju: stoj, zmierz, zapamietaj - potem dopiero jedz.

Kamera lezy POZIOMO na platformie, ~7.5 cm nad ziemia, patrzy w przod.
Wynikiem nie jest XYZ w ukladzie kamery, tylko to, czego potrzebuje dojazd:

    do przodu [m], w bok [m] (dodatnie = w prawo), kat [st], szerokosc chwytu [m]

Dlaczego skan na postoju, a nie ciagle sledzenie: D415 w trybie 640x480 mierzy
od ~0.31 m. Gdy platforma podjedzie na dystans chwytania, szyszka bedzie blizej
niz to minimum i zniknie z glebi. Pozycje trzeba wiec zdjac raz, z dystansu, i
dalej jechac z pamieci.

Pomiar to mediana z kilku klatek, nie jedna klatka. Wypisywany rozrzut mowi,
czy wynikowi mozna ufac - przy stojacej platformie powinien byc milimetrowy.

Uzycie:
    python scan_cones.py                       # skan i wypis
    python scan_cones.py --frames 30           # dluzszy skan, mniejszy rozrzut
    python scan_cones.py --json cel.json       # zapis celu dla dojazdu
    python scan_cones.py --snapshot skan.png   # klatka z zaznaczonym celem
    python scan_cones.py --watch               # skanuj w kolko, do Ctrl+C
"""

from __future__ import annotations

import argparse
import json
import time

import cv2
import numpy as np
import pyrealsense2 as rs

from detect_floor_objects import (
    TARGET_PRESETS,
    FloorObjectDetector,
    draw,
    start_pipeline,
)

CAMERA_HEIGHT = 0.075  # m; kamera poziomo na platformie
HEIGHT_TOLERANCE = 0.03  # m; powyzej tego uznajemy, ze cos jest nie tak
SCAN_FRAMES = 15
FLOOR_FRAMES = 10
MATCH_RADIUS = 0.04  # m; detekcje blizej siebie niz to = ta sama szyszka
MIN_SEEN_FRACTION = 0.6  # w ilu klatkach cel musi sie pojawic, zeby go uznac


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Skan szyszek na postoju")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--frames", type=int, default=SCAN_FRAMES, help="klatek na skan")
    p.add_argument(
        "--camera-height",
        type=float,
        default=CAMERA_HEIGHT,
        help="deklarowana wysokosc kamery nad ziemia [m], do kontroli dopasowania",
    )
    p.add_argument(
        "--camera-forward-offset",
        type=float,
        default=0.0,
        help="o ile kamera wystaje przed punkt odniesienia platformy [m]",
    )
    p.add_argument(
        "--max-distance",
        type=float,
        default=1.0,
        help="zasieg skanu [m]; ustalone 0.3-1.0 m, dalej rozrzut szybko rosnie",
    )
    p.add_argument("--min-distance", type=float, default=0.3)
    p.add_argument("--max-side", type=float, default=0.6)
    p.add_argument("--json", help="zapisz znalezione cele do pliku JSON")
    p.add_argument("--snapshot", help="zapisz klatke z zaznaczonymi celami")
    p.add_argument("--watch", action="store_true", help="skanuj w kolko do Ctrl+C")
    p.add_argument("--preview", action="store_true", help="pokaz okno podgladu")
    return p.parse_args()


def fit_floor(pipeline, align, detector, frames_count=FLOOR_FRAMES):
    """Kilka klatek na rozgrzanie auto-ekspozycji, potem dopasowanie ziemi."""
    frames = None
    for _ in range(frames_count):
        frames = align.process(pipeline.wait_for_frames())
    if not detector.fit_floor(frames):
        raise SystemExit("Nie udalo sie dopasowac ziemi - za malo punktow w zasiegu.")
    return frames


def scan(pipeline, align, detector, frames_count):
    """
    Zbiera detekcje z kilku klatek i skleja te, ktore dotycza tej samej szyszki.

    Sklejanie po pozycji na ziemi, nie po kolejnosci w liscie - miedzy klatkami
    numeracja klastrow potrafi sie przestawic.
    """
    tracks: list[dict] = []
    last_frames = None

    for index in range(frames_count):
        frames = align.process(pipeline.wait_for_frames())
        last_frames = frames
        for obj in detector.detect(frames):
            position = np.array([obj["forward_m"], obj["lateral_m"]])
            for track in tracks:
                # Jeden slad moze dostac najwyzej jedna detekcje z danej klatki.
                # Bez tego dwa sasiadujace klastry z tej samej klatki wpadaja do
                # tego samego sladu i licznik "widziana w N/M klatkach" pokazuje
                # wiecej trafien niz bylo klatek - a to jest miara, na ktorej
                # opiera sie zaufanie do pomiaru.
                if track["frames"] and track["frames"][-1] == index:
                    continue
                if np.linalg.norm(position - track["anchor"]) < MATCH_RADIUS:
                    track["hits"].append(obj)
                    track["frames"].append(index)
                    break
            else:
                tracks.append({"anchor": position, "hits": [obj], "frames": [index]})

    results = []
    for track in tracks:
        hits = track["hits"]
        if len(hits) < MIN_SEEN_FRACTION * frames_count:
            continue  # migotanie, nie cel

        def med(key):
            return float(np.median([h[key] for h in hits]))

        def spread(key):
            return float(np.std([h[key] for h in hits]))

        results.append(
            {
                "forward_m": med("forward_m"),
                "lateral_m": med("lateral_m"),
                "ground_distance_m": med("ground_distance_m"),
                "distance_m": med("distance_m"),
                "bearing_deg": med("bearing_deg"),
                "width_m": med("width_m"),
                "length_m": med("length_m"),
                "height_m": med("height_m"),
                "angle_deg": med("angle_deg"),
                "seen_in": f"{len(hits)}/{frames_count}",
                "spread_forward_mm": spread("forward_m") * 1000,
                "spread_lateral_mm": spread("lateral_m") * 1000,
                "pixels": hits[-1]["pixels"],
                "fill": med("fill"),
                "area_px": int(np.median([h["area_px"] for h in hits])),
            }
        )

    results.sort(key=lambda r: r["ground_distance_m"])
    return results, last_frames


def report(results, offset, detector=None):
    if not results:
        print("Nie znaleziono szyszek w zasiegu skanu.")
        if detector is not None:
            rej = detector.rejected
            total = sum(rej.values())
            if total:
                print(
                    "  Cos w kadrze bylo, ale odpadlo na bramkach: "
                    f"pole {rej['area']}, szerokosc {rej['width']}, "
                    f"dlugosc {rej['length']}, wypelnienie {rej['fill']}, "
                    f"wystaje ponad pasmo {rej['tall']}"
                )
                print(
                    "  Sprawdz progi presetu 'szyszka' w detect_floor_objects.py "
                    "albo odpal detect_floor_objects.py --target any, zeby zobaczyc "
                    "surowe wymiary tego, co lezy w kadrze."
                )
            else:
                print(
                    "  Zaden klaster nawet nie powstal - nic nie wystaje nad ziemie "
                    "w zasiegu skanu. Szyszka poza zasiegiem, za blisko niz 0.31 m, "
                    "albo poza katem widzenia."
                )
        return
    print(f"Znaleziono {len(results)}:")
    for i, r in enumerate(results):
        forward = r["forward_m"] - offset
        side = "prawo" if r["lateral_m"] >= 0 else "lewo"
        print(
            f"  #{i}  do przodu {forward:.3f} m, w {side} {abs(r['lateral_m']):.3f} m "
            f"(kat {r['bearing_deg']:+.1f} st)"
        )
        print(
            f"      po ziemi {r['ground_distance_m']:.3f} m, "
            f"w linii prostej {r['distance_m']:.3f} m"
        )
        print(
            f"      rozmiar {r['length_m'] * 100:.1f} x {r['width_m'] * 100:.1f} cm, "
            f"wys {r['height_m'] * 100:.1f} cm, chwyt {r['width_m'] * 100:.1f} cm, "
            f"obrot chwytaka {r['angle_deg']:+.1f} st"
        )
        print(
            f"      widziana w {r['seen_in']} klatkach, rozrzut "
            f"{r['spread_forward_mm']:.1f} mm w przod / "
            f"{r['spread_lateral_mm']:.1f} mm w bok"
        )


def main() -> None:
    args = parse_args()
    pipeline, align, depth_scale = start_pipeline(args.width, args.height, args.fps)

    params = dict(TARGET_PRESETS["szyszka"])
    detector = FloorObjectDetector(
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        max_side=args.max_side,
        **params,
    )
    detector.set_depth_scale(depth_scale)

    try:
        print("Dopasowuje ziemie - w kadrze ma byc CZYSTY tor, bez szyszek...")
        fit_floor(pipeline, align, detector)

        # Offset plaszczyzny to zmierzona wysokosc kamery nad ziemia. Porownanie
        # jej z deklarowana jest najtanszym testem, czy dopasowala sie ZIEMIA,
        # a nie sciana albo blat - bez tego caly pomiar jest liczony od zlej
        # plaszczyzny i wyglada wiarygodnie mimo ze jest bzdura.
        measured = detector.offset
        print(
            f"Ziemia dopasowana: kamera {measured * 100:.1f} cm nad nia "
            f"(zadeklarowano {args.camera_height * 100:.1f} cm)"
        )
        if abs(measured - args.camera_height) > HEIGHT_TOLERANCE:
            print(
                "  UWAGA: rozjazd wiekszy niz "
                f"{HEIGHT_TOLERANCE * 100:.0f} cm. Prawdopodobnie dopasowala sie "
                "inna plaszczyzna niz ziemia (sciana, blat, przeszkoda w kadrze) "
                "albo kamera wisi inaczej niz podano. Popraw kadr i uruchom ponownie."
            )

        while True:
            print(f"\nSkanuje {args.frames} klatek...")
            results, frames = scan(pipeline, align, detector, args.frames)
            report(results, args.camera_forward_offset, detector)

            if args.json:
                with open(args.json, "w") as f:
                    json.dump(
                        {
                            "camera_height_m": measured,
                            "camera_forward_offset_m": args.camera_forward_offset,
                            "scanned_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                            "targets": results,
                        },
                        f,
                        indent=2,
                    )
                print(f"Zapisano cele do {args.json}")

            if args.snapshot or args.preview:
                color = np.asanyarray(frames.get_color_frame().get_data())
                view = draw(color.copy(), detector.detect(frames), detector.last_mask, False)
                if args.snapshot:
                    cv2.imwrite(args.snapshot, view)
                    print(f"Zapisano klatke do {args.snapshot}")
                if args.preview:
                    cv2.imshow("skan szyszek", view)
                    cv2.waitKey(1)

            if not args.watch:
                break
    except KeyboardInterrupt:
        print("\nPrzerwano.")
    finally:
        pipeline.stop()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
