"""
Zmierz kolor celu i tla, zamiast go zgadywac.

Bierze detekcje z detektora GLEBI - czyli z czegos, co wie, gdzie naprawde
lezy szyszka - i probkuje pod ta maska kolor w HSV. Osobno probkuje murawe
dookola. Na wyjsciu percentyle obu rozkladow, z ktorych widac, czy da sie je
rozdzielic progiem i ktorym kanalem.

Po co: progi koloru dobrane "na oko" albo z ksiazki nie maja szansy. Pomiar na
tym torze o zmierzchu dal murawe przy H~90 (cyjan w skali OpenCV) i szyszki
przy H~148 (fiolet) - zadnego brazu, a ogony rozkladow odcienia zachodzily na
siebie. Rozdzielala je dopiero JASNOSC: szyszki V~94, murawa V~152.

Prog na jasnosc jest z natury kruchy - slonce albo cien go przesuwa. Przy
kazdej zmianie swiatla przepusc to jeszcze raz i podmien COLOR_GATE w
detect_floor_objects.py.

Uzycie:
    python sample_colors.py
    python sample_colors.py --frames 20 --laser-power 360
"""

from __future__ import annotations

import argparse

import cv2
import numpy as np

from detect_floor_objects import TARGET_PRESETS, FloorObjectDetector, start_pipeline

GROUND_ROWS = (300, 470)  # pas obrazu, w ktorym na pewno jest samo podloze
GROUND_COLS = (100, 540)
MASK_MARGIN_PX = 25  # o tyle odsuwamy sie od obiektow, zeby nie probkowac ich brzegow


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Pomiar HSV celu i podloza")
    p.add_argument("--frames", type=int, default=10, help="ile klatek probkowac")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--laser-power", type=float, help="moc projektora IR 0-360")
    p.add_argument("--max-distance", type=float, default=1.0)
    p.add_argument("--max-side", type=float, default=0.6)
    return p.parse_args()


def describe(name: str, stack: list) -> None:
    if not stack:
        print(f"\n{name}: brak probek")
        return
    data = np.vstack(stack)
    print(f"\n{name} ({len(data)} pikseli)")
    for i, channel in enumerate("HSV"):
        column = data[:, i]
        p5, p25, med, p75, p95 = np.percentile(column, [5, 25, 50, 75, 95])
        print(
            f"  {channel}: p5={p5:5.0f} p25={p25:5.0f} mediana={med:5.0f} "
            f"p75={p75:5.0f} p95={p95:5.0f}"
        )


def main() -> None:
    args = parse_args()
    pipeline, align, depth_scale = start_pipeline(
        args.width, args.height, args.fps, args.laser_power
    )
    detector = FloorObjectDetector(
        min_distance=0.3,
        max_distance=args.max_distance,
        max_side=args.max_side,
        **TARGET_PRESETS["szyszka"],
    )
    detector.set_depth_scale(depth_scale)

    target_pixels: list = []
    ground_pixels: list = []

    try:
        print("Dopasowuje ziemie - w kadrze ma byc CZYSTY tor...")
        for _ in range(12):
            frames = align.process(pipeline.wait_for_frames())
        if not detector.fit_floor(frames):
            raise SystemExit("Nie udalo sie dopasowac ziemi.")
        print(f"Ziemia: kamera {detector.offset * 100:.1f} cm nad nia")
        print(f"Probkuje {args.frames} klatek - poloz cele w kadrze...")

        for _ in range(args.frames):
            frames = align.process(pipeline.wait_for_frames())
            detector.detect(frames)
            color = np.asanyarray(frames.get_color_frame().get_data())
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)

            mask = detector.last_mask.astype(bool)
            if mask.any():
                target_pixels.append(hsv[mask])

            # Podloze: ustalony pas kadru, z odsunieciem od wykrytych obiektow,
            # zeby nie wciagnac ich krawedzi do rozkladu tla.
            ground = np.zeros_like(mask)
            ground[GROUND_ROWS[0] : GROUND_ROWS[1], GROUND_COLS[0] : GROUND_COLS[1]] = True
            grown = cv2.dilate(
                mask.astype(np.uint8),
                np.ones((MASK_MARGIN_PX, MASK_MARGIN_PX), np.uint8),
            ).astype(bool)
            ground &= ~grown
            ground_pixels.append(hsv[ground])
    except RuntimeError as exc:
        print(f"\nKamera przestala odpowiadac: {exc}")
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            pass

    describe("CELE (pod maska detektora glebi)", target_pixels)
    describe("PODLOZE", ground_pixels)
    print(
        "\nSzukaj kanalu, w ktorym percentyle obu rozkladow sie NIE zachodza - "
        "to on rozdziela cel od tla.\nWpisz prog do COLOR_GATE w "
        "detect_floor_objects.py i sprawdz przez --mode color."
    )


if __name__ == "__main__":
    main()
