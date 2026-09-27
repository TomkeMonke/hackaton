"""Podglad na zywo z RealSense (D435 na robocie) przez przegladarke (MJPEG po HTTP).

Uzycie na Pi:
    python rs_mjpeg_server.py [--port 8080] [--depth-res 848x480] [--align] [--colormap viewer|fixed]

Domyslnie tak jak RealSense Viewer z fabrycznymi ustawieniami (librealsense common/subdevice-model.cpp):
glebia w natywnej rozdzielczosci D435 (848x480), NIE wyrownana do koloru, bez filtrow post-processingu
(Viewer wlacza je tylko z pliku konfiguracji), kolory z rs.colorizer z domyslnymi opcjami (Jet
z wyrownaniem histogramu, brak danych = czarny). Obok obraz z kamery kolorowej 640x480.

Stare ustawienia podgladu: --depth-res 424x240 --align --colormap fixed. Nizsza rozdzielczosc glebi ma
mniejszy minimalny zasieg (MinZ skaluje sie z rozdzielczoscia), --align nakleja glebie na obraz koloru
(D435: kolor ma wezszy kat, wiec glebia wychodzi przycieta), skala liniowa 0..--max-mm zlewa podloge
w jeden gradient.

Kamera odpadla z USB (dmesg: uvcvideo status -71, USB disconnect): podglad pokazuje "BRAK KAMERY"
zamiast zamrozonej ostatniej klatki i co 2 s probuje polaczyc sie od nowa.

Potem w przegladarce na laptopie: http://<IP_PI>:8080/
Zatrzymanie: Ctrl+C.
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH, HEIGHT, FPS = 640, 480, 30
DEPTH_WIDTH, DEPTH_HEIGHT = 848, 480  # natywna glebia D435, domyslny profil w RealSense Viewer
ALIGN = False  # Viewer pokazuje glebie niewyrownana
MAX_MM = 1500  # odleglosc, przy ktorej kolormapa sie nasyca (tylko --colormap fixed)
COLORMAP = "viewer"  # "viewer" = rs.colorizer jak w RealSense Viewer, "fixed" = liniowo 0..MAX_MM
RETRY_SECONDS = 2.0

latest_jpg = None
latest_lock = threading.Lock()
stop_event = threading.Event()


def publish(image):
    global latest_jpg
    ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        with latest_lock:
            latest_jpg = buf.tobytes()


def status_image(lines):
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(img, line[:48], (16, 60 + 40 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255) if i == 0 else (200, 200, 200), 2)
    return img


def side_by_side(color_image, depth_colormap):
    """Obraz koloru i glebia obok siebie; glebia skalowana do wysokosci koloru (bez zmiany proporcji)."""
    h = color_image.shape[0]
    if depth_colormap.shape[0] != h:
        w = round(depth_colormap.shape[1] * h / depth_colormap.shape[0])
        depth_colormap = cv2.resize(depth_colormap, (w, h), interpolation=cv2.INTER_NEAREST)
    return np.hstack((color_image, depth_colormap))


def run_pipeline():
    """Jedno polaczenie z kamera: wysyla klatki az do bledu (np. kamera odpadla z USB)."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
    profile = pipeline.start(config)
    try:
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        alpha = 255.0 / (MAX_MM / 1000.0 / depth_scale)
        align = rs.align(rs.stream.color) if ALIGN else None
        colorizer = rs.colorizer()  # domyslne opcje = te same co w RealSense Viewer
        print(f"RealSense: color {WIDTH}x{HEIGHT}, depth {DEPTH_WIDTH}x{DEPTH_HEIGHT}, "
              f"align {ALIGN}, colormap {COLORMAP}", flush=True)
        while not stop_event.is_set():
            frames = pipeline.wait_for_frames(3000)
            if align is not None:
                frames = align.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            color_image = np.asanyarray(color_frame.get_data())
            if COLORMAP == "viewer":
                rgb = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                depth_colormap = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=alpha), cv2.COLORMAP_JET
                )
            publish(side_by_side(color_image, depth_colormap))
    finally:
        try:
            pipeline.stop()
        except RuntimeError:
            pass
        print("RealSense: stream stopped", flush=True)


def capture_loop():
    publish(status_image(["LACZE Z KAMERA..."]))
    while not stop_event.is_set():
        try:
            run_pipeline()
        except RuntimeError as exc:
            print(f"RealSense: {exc} - ponawiam za {RETRY_SECONDS:.0f} s", flush=True)
            publish(status_image(["BRAK KAMERY", str(exc), "sprawdz kabel USB3 (lsusb | grep 8086)",
                                  f"ponawiam co {RETRY_SECONDS:.0f} s"]))
            stop_event.wait(RETRY_SECONDS)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            body = (
                b"<html><body style='margin:0;background:#111'>"
                b"<img src='/stream' style='width:100%'></body></html>"
            )
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/stream":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            try:
                while not stop_event.is_set():
                    with latest_lock:
                        frame = latest_jpg
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(1 / FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_response(404)
        self.end_headers()


def main():
    global DEPTH_WIDTH, DEPTH_HEIGHT, ALIGN, MAX_MM, COLORMAP
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--depth-res", default=f"{DEPTH_WIDTH}x{DEPTH_HEIGHT}",
                        help="rozdzielczosc glebi, np. 848x480 (jak Viewer), 640x480, 424x240 (mniejszy MinZ)")
    parser.add_argument("--align", action="store_true",
                        help="wyrownaj glebie do obrazu koloru (Viewer tego nie robi)")
    parser.add_argument("--colormap", choices=("viewer", "fixed"), default=COLORMAP,
                        help="viewer = jak RealSense Viewer (wyrownanie histogramu), fixed = liniowo 0..--max-mm")
    parser.add_argument("--max-mm", type=int, default=MAX_MM,
                        help="odleglosc [mm] dla czerwonego koloru (tylko --colormap fixed)")
    args = parser.parse_args()

    DEPTH_WIDTH, DEPTH_HEIGHT = (int(v) for v in args.depth_res.lower().split("x"))
    ALIGN = args.align
    MAX_MM = args.max_mm
    COLORMAP = args.colormap

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"Otworz w przegladarce: http://<IP_PI>:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()


if __name__ == "__main__":
    main()
