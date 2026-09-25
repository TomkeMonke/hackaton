"""
Detekcja obiektu na obrazie z RealSense D415 + deprojekcja do punktu 3D.

Strumien koloru i glebi z D415, prog HSV na wybrany kolor, srodek najwiekszej
maski -> `rs.rs2_deproject_pixel_to_point` -> XYZ w ukladzie kamery.
Wejscie do kalibracji kamera -> baza ramienia (osobny task) i dalej pick-and-place.

Uklad wspolrzednych (optyczny RealSense, metry):
    X w prawo, Y w dol, Z w glab sceny - patrzac z perspektywy kamery.
    Punkt (0,0,0) to srodek optyczny strumienia KOLORU, bo glebia jest do niego
    wyrownana (`rs.align`), wiec piksel koloru i piksel glebi to ten sam promien.

Uzycie:
    python detect_object.py --color red                 # podglad + XYZ na zywo
    python detect_object.py --color blue --log out.csv  # do tego zapis CSV
    python detect_object.py --tune                      # suwaki HSV, dobranie progu
    python detect_object.py --color green --no-preview  # bez okna, sam log

    python detect_object.py --show-color-options        # zakresy balansu i ekspozycji
    python detect_object.py --color red --white-balance 4600 --exposure 156

Progi HSV maja sens tylko przy nieplywajacym obrazie: auto-balans bieli przesuwa
odcien (H), auto-ekspozycja jasnosc (V). Dobierz progi i zamroz obie wartosci
tymi samymi flagami, wtedy to samo ustawienie trafi w obiekt nastepnego dnia.

Klawisze w podgladzie:
    q / ESC   wyjscie
    m         przelacz widok maski
    s         zapisz klatke do detect_snapshot.png
    h         wypisz HSV piksela w srodku obrazu (pomaga dobrac kolor)

Zaleznosci: pyrealsense2, opencv-python, numpy (numpy==2.5.3 z constraints.txt).
"""

from __future__ import annotations

import argparse
import csv
import time

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 640, 480, 30

# Progi HSV w skali OpenCV: H 0..179, S 0..255, V 0..255.
# Czerwony owija sie przez 0, wiec ma dwa zakresy.
COLOR_PRESETS = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))],
    "green": [((35, 80, 60), (85, 255, 255))],
    "blue": [((95, 120, 60), (130, 255, 255))],
    "yellow": [((20, 110, 90), (33, 255, 255))],
    "orange": [((10, 130, 90), (20, 255, 255))],
}

MIN_AREA = 400  # piksele; mniejsze plamy to szum, nie obiekt
DEPTH_PATCH = 5  # bok okna (px) do mediany glebi wokol centroidu
PRINT_EVERY = 0.25  # sekundy miedzy wypisami, zeby nie zalac konsoli


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Detekcja obiektu + deprojekcja 3D")
    p.add_argument(
        "--color",
        choices=sorted(COLOR_PRESETS),
        default="red",
        help="preset koloru obiektu",
    )
    p.add_argument("--min-area", type=int, default=MIN_AREA, help="min. pole maski [px]")
    p.add_argument("--width", type=int, default=WIDTH)
    p.add_argument("--height", type=int, default=HEIGHT)
    p.add_argument("--fps", type=int, default=FPS)
    p.add_argument(
        "--white-balance",
        type=float,
        help="zamroz balans bieli na podanej wartosci (D415: 2800..6500, dom. 4600)",
    )
    p.add_argument(
        "--exposure",
        type=float,
        help="zamroz ekspozycje (D415: 1..10000, dom. 156)",
    )
    p.add_argument(
        "--show-color-options",
        action="store_true",
        help="wypisz zakresy balansu i ekspozycji, po czym wyjdz",
    )
    p.add_argument("--log", help="zapis wykrytych punktow do pliku CSV")
    p.add_argument("--tune", action="store_true", help="suwaki HSV zamiast presetu")
    p.add_argument("--no-preview", action="store_true", help="bez okna, sam wypis")
    p.add_argument(
        "--seconds", type=float, help="zakoncz po N sekundach (do testow bez okna)"
    )
    return p.parse_args()


def configure_color_sensor(profile, white_balance=None, exposure=None) -> None:
    """
    Zamraza balans bieli / ekspozycje kamery koloru.

    Progi HSV maja sens tylko wtedy, gdy obraz nie plywa. Auto-balans przesuwa
    odcien (H) miedzy sesjami, a auto-ekspozycja jasnosc (V), wiec progi dobrane
    raz przestaja trafiac w ten sam obiekt przy innym swietle. Bez tych flag nic
    nie ruszamy - kamera zostaje na swoich automatach.
    """
    sensor = profile.get_device().first_color_sensor()

    def set_fixed(auto_option, value_option, value, label):
        if not sensor.supports(value_option):
            print(f"  {label}: nieobslugiwane przez ten sensor")
            return
        rng = sensor.get_option_range(value_option)
        if not rng.min <= value <= rng.max:
            raise SystemExit(
                f"{label}={value} poza zakresem {rng.min:.0f}..{rng.max:.0f} "
                f"(domyslnie {rng.default:.0f})"
            )
        if sensor.supports(auto_option):
            sensor.set_option(auto_option, 0)
        sensor.set_option(value_option, value)
        print(f"  {label}: {sensor.get_option(value_option):.0f} (auto wylaczone)")

    if white_balance is not None:
        set_fixed(
            rs.option.enable_auto_white_balance,
            rs.option.white_balance,
            white_balance,
            "balans bieli",
        )
    if exposure is not None:
        set_fixed(
            rs.option.enable_auto_exposure, rs.option.exposure, exposure, "ekspozycja"
        )


def show_color_options(profile) -> None:
    """Wypisuje zakresy, zeby bylo z czego wybierac wartosci do zamrozenia."""
    sensor = profile.get_device().first_color_sensor()
    print("Opcje kamery koloru:")
    for name in (
        "white_balance",
        "enable_auto_white_balance",
        "exposure",
        "enable_auto_exposure",
    ):
        option = getattr(rs.option, name)
        if not sensor.supports(option):
            print(f"  {name:26} nieobslugiwane")
            continue
        rng = sensor.get_option_range(option)
        print(
            f"  {name:26} teraz={sensor.get_option(option):7.0f} "
            f"min={rng.min:.0f} max={rng.max:.0f} domyslnie={rng.default:.0f}"
        )


def start_pipeline(width: int, height: int, fps: int):
    """Uruchamia strumienie i zwraca (pipeline, align, depth_scale, profile)."""
    if len(rs.context().query_devices()) == 0:
        raise SystemExit(
            "Nie widac kamery RealSense.\n"
            "  - wepnij D415 w port USB (SuperSpeed daje wyzsze tryby)\n"
            "  - sprawdz w realsense-viewer, czy sie pokazuje"
        )

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        raise SystemExit(
            f"Nie udalo sie wystartowac strumienia {width}x{height}@{fps}: {exc}\n"
            "Kamere moze trzymac inny proces (realsense-viewer, rs_preview.py) -\n"
            "urzadzenie jest na wylacznosc, zamknij tamto najpierw.\n"
            "Na laczu USB 2 czesc trybow nie istnieje; 640x480@30 dziala."
        ) from exc

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    return pipeline, rs.align(rs.stream.color), depth_scale, profile


def make_mask(hsv: np.ndarray, ranges: list) -> np.ndarray:
    """Suma zakresow HSV + domkniecie morfologiczne (usuwa szum i dziury)."""
    mask = None
    for low, high in ranges:
        part = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def largest_blob(mask: np.ndarray, min_area: int):
    """Zwraca (centroid_px, pole, kontur) najwiekszej plamy albo None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    u = int(moments["m10"] / moments["m00"])
    v = int(moments["m01"] / moments["m00"])
    return (u, v), area, contour


def patch_depth(depth_image: np.ndarray, u: int, v: int, depth_scale: float) -> float:
    """
    Mediana glebi z okna wokol centroidu, w metrach. 0.0 gdy brak danych.

    Pojedynczy piksel potrafi byc dziura (0) na krawedzi obiektu albo na polysku,
    a mediana z waznych probek jest na to odporna.
    """
    half = DEPTH_PATCH // 2
    h, w = depth_image.shape
    patch = depth_image[
        max(0, v - half) : min(h, v + half + 1),
        max(0, u - half) : min(w, u + half + 1),
    ]
    valid = patch[patch > 0]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid)) * depth_scale


def setup_trackbars(window: str) -> None:
    cv2.namedWindow(window)
    for name, default, maximum in (
        ("H min", 0, 179),
        ("H max", 179, 179),
        ("S min", 120, 255),
        ("S max", 255, 255),
        ("V min", 70, 255),
        ("V max", 255, 255),
    ):
        cv2.createTrackbar(name, window, default, maximum, lambda _: None)


def read_trackbars(window: str) -> list:
    get = lambda name: cv2.getTrackbarPos(name, window)  # noqa: E731
    low = (get("H min"), get("S min"), get("V min"))
    high = (get("H max"), get("S max"), get("V max"))
    return [(low, high)]


def main() -> None:
    args = parse_args()
    preview = not args.no_preview
    tune_window = "HSV"

    pipeline, align, depth_scale, profile = start_pipeline(
        args.width, args.height, args.fps
    )
    print(
        f"Strumien {args.width}x{args.height}@{args.fps}, "
        f"depth_scale={depth_scale} (jednostka * scale = metry)"
    )
    if args.show_color_options:
        show_color_options(profile)
        pipeline.stop()
        return
    configure_color_sensor(profile, args.white_balance, args.exposure)
    if args.tune and preview:
        setup_trackbars(tune_window)
    ranges = COLOR_PRESETS[args.color]

    log_file = log_writer = None
    if args.log:
        log_file = open(args.log, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "u", "v", "area_px", "x_m", "y_m", "z_m"])

    show_mask = False
    last_print = 0.0
    t0 = time.time()
    frames_seen = detections = 0

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            frames_seen += 1

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Po wyrownaniu glebia siedzi w ukladzie koloru, wiec intrinsics
            # bierzemy z wyrownanej ramki glebi - to sa parametry kamery koloru.
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

            if args.tune and preview:
                ranges = read_trackbars(tune_window)

            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask = make_mask(hsv, ranges)
            found = largest_blob(mask, args.min_area)

            point = None
            if found is not None:
                (u, v), area, contour = found
                z = patch_depth(depth_image, u, v, depth_scale)
                if z > 0:
                    point = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], z)
                    detections += 1
                    now = time.time()
                    if now - last_print >= PRINT_EVERY:
                        print(
                            f"px=({u:3d},{v:3d}) pole={area:6.0f}  "
                            f"XYZ = {point[0]:+.3f} {point[1]:+.3f} {point[2]:+.3f} m"
                        )
                        last_print = now
                    if log_writer:
                        log_writer.writerow(
                            [
                                f"{now - t0:.4f}",
                                u,
                                v,
                                int(area),
                                f"{point[0]:.4f}",
                                f"{point[1]:.4f}",
                                f"{point[2]:.4f}",
                            ]
                        )

            if preview:
                view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else color_image.copy()
                if found is not None:
                    (u, v), area, contour = found
                    cv2.drawContours(view, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(view, (u, v), 6, (0, 0, 255), -1)
                    label = (
                        f"{point[0]:+.3f} {point[1]:+.3f} {point[2]:+.3f} m"
                        if point
                        else "brak glebi w tym punkcie"
                    )
                    cv2.putText(
                        view, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2,
                    )
                else:
                    cv2.putText(
                        view, "nie wykryto obiektu", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
                    )
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                cv2.imshow("D415 - detekcja | glebia", np.hstack((view, depth_vis)))

                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("m"):
                    show_mask = not show_mask
                if key == ord("s"):
                    cv2.imwrite("detect_snapshot.png", view)
                    print("zapisano detect_snapshot.png")
                if key == ord("h"):
                    cy, cx = args.height // 2, args.width // 2
                    print(f"HSV w srodku obrazu: {hsv[cy, cx]}")

            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        if preview:
            cv2.destroyAllWindows()
        if log_file:
            log_file.close()
            print(f"Zapisano log: {args.log}")
        elapsed = time.time() - t0
        print(
            f"Koniec. {frames_seen} klatek w {elapsed:.1f}s "
            f"({frames_seen / elapsed:.1f} fps), wykryc: {detections}"
        )
        if args.tune:
            low, high = ranges[0]
            print(f"Dobrane progi HSV: low={tuple(low)} high={tuple(high)}")


if __name__ == "__main__":
    main()
