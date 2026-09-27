"""Podglad kamery w przegladarce W TYM SAMYM procesie co lerobot (record / rollout / teleoperate).

RealSense (i kazda kamera USB) moze byc otwarta tylko przez jeden proces. Osobny
`rs_mjpeg_server.py` gryzie sie wiec z `lerobot-record` i `lerobot-rollout`
(`Couldn't resolve requests` / `No device connected`). Ten skrypt uruchamia komende
lerobot w swoim procesie i obok niej serwer MJPEG, ktory PODGLADA klatki tej samej
kamery (`read_latest()` - nie zabiera klatek petli sterowania, nie otwiera kamery drugi raz).

Uzycie na Pi (opcje podgladu PRZED nazwa komendy, dalej argumenty lerobot bez zmian):
    .venv/bin/python tools/cam_preview.py lerobot-record --robot.type=so101_follower ...
    .venv/bin/python tools/cam_preview.py --preview-port 8081 --preview-fps 10 lerobot-rollout ...

Potem w przegladarce: http://<IP_PI>:8081/  (8080 = rs_mjpeg_server, 8000 = jazda, 8010 = ramie).
Ctrl+C trafia do lerobot jak zwykle; serwer podgladu konczy sie razem z procesem.
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CAMERAS: list = []  # kamery lerobot utworzone w tym procesie (wypelnia install_hook)
_cameras_lock = threading.Lock()


def register(cam) -> None:
    with _cameras_lock:
        CAMERAS.append(cam)


def install_hook() -> None:
    """Kazda kamera lerobot (RealSense, OpenCV, ...) zapisuje sie do CAMERAS przy tworzeniu.

    Import leniwy: lerobot jest tylko na Pi / maszynie z ramieniem.
    """
    from lerobot.cameras.camera import Camera

    if getattr(Camera.__init__, "_cam_preview", False):
        return
    orig_init = Camera.__init__

    def init(self, config, *args, **kwargs):
        orig_init(self, config, *args, **kwargs)
        register(self)

    init._cam_preview = True
    Camera.__init__ = init


def grab_jpeg(cam, quality: int = 70):
    """Ostatnia klatka kamery jako JPEG albo None (kamera jeszcze nie gotowa / rozlaczona)."""
    import cv2

    try:
        frame = cam.read_latest(max_age_ms=2000)
    except Exception:
        return None
    if frame is None or getattr(frame, "ndim", 0) != 3:
        return None
    mode = getattr(cam, "color_mode", None)
    if getattr(mode, "value", mode) == "rgb":  # lerobot domyslnie oddaje RGB, JPEG z cv2 chce BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _index_html() -> bytes:
    with _cameras_lock:
        names = [str(c) for c in CAMERAS]
    if not names:
        body = "<p>Brak kamer (lerobot jeszcze sie laczy). Odswiezam...</p>"
        head = '<meta http-equiv="refresh" content="2">'
    else:
        head = ""
        body = "".join(
            f'<figure><img src="/cam/{i}.mjpg" onerror="retry(this)"><figcaption>{n}</figcaption></figure>'
            for i, n in enumerate(names))
    return (f"<!doctype html><html><head><title>Podglad kamer</title>{head}"
            "<style>body{background:#111;color:#ccc;font-family:sans-serif;margin:8px}"
            "img{max-width:100%;display:block}figure{margin:0 0 12px}</style>"
            "<script>function retry(img){var s=img.src.split('?')[0];"
            "setTimeout(function(){img.src=s+'?t='+Date.now()},1000)}</script>"
            f"</head><body>{body}</body></html>").encode()


def make_handler(fps: float, quality: int):
    period = 1.0 / max(fps, 0.5)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # nie zasmiecaj logu lerobot
            pass

        def _camera(self):
            name = self.path.split("?")[0].rsplit("/", 1)[-1]
            try:
                idx = int(name.split(".")[0])
                with _cameras_lock:
                    return CAMERAS[idx], name.split(".", 1)[1]
            except (ValueError, IndexError):
                return None, None

        def do_GET(self):
            if self.path.split("?")[0] in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", _index_html())
                return
            cam, ext = self._camera() if self.path.startswith("/cam/") else (None, None)
            if cam is None:
                self._send(404, "text/plain", b"nie ma takiej kamery")
            elif ext == "jpg":
                jpg = grab_jpeg(cam, quality)
                if jpg is None:
                    self._send(503, "text/plain", b"brak klatki")
                else:
                    self._send(200, "image/jpeg", jpg)
            elif ext == "mjpg":
                self._stream(cam)
            else:
                self._send(404, "text/plain", b"?")

        def _send(self, code, ctype, data):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _stream(self, cam):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            misses = 0
            try:
                while True:
                    jpg = grab_jpeg(cam, quality)
                    if jpg is None:
                        misses += 1
                        if misses > 50 and not getattr(cam, "is_connected", True):
                            return  # kamera zamknieta (koniec rollout) - przegladarka sprobuje ponownie
                        time.sleep(0.2)
                        continue
                    misses = 0
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     + f"Content-Length: {len(jpg)}\r\n\r\n".encode() + jpg + b"\r\n")
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    return Handler


def start_server(host: str = "0.0.0.0", port: int = 8081, fps: float = 10, quality: int = 70):
    """Serwer MJPEG w watku-demonie. Zwraca serwer (server.server_address[1] = port)."""
    server = ThreadingHTTPServer((host, port), make_handler(fps, quality))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, name="cam_preview", daemon=True).start()
    return server


def resolve_entry(name: str):
    """Funkcja main() komendy lerobot (np. lerobot-rollout) z entry points pakietu."""
    from importlib.metadata import entry_points

    eps = entry_points(group="console_scripts", name=name)
    if not eps:
        raise SystemExit(f"Nie znam komendy {name!r} (np. lerobot-record, lerobot-rollout, lerobot-teleoperate)")
    return next(iter(eps)).load()


def main(argv=None, resolve=resolve_entry, hook=install_hook) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--preview-port", type=int, default=8081)
    parser.add_argument("--preview-host", default="0.0.0.0")
    parser.add_argument("--preview-fps", type=float, default=10, help="klatek/s w podgladzie (CPU Pi)")
    parser.add_argument("--preview-quality", type=int, default=70, help="jakosc JPEG 1-100")
    parser.add_argument("entry", help="komenda lerobot, np. lerobot-rollout")
    parser.add_argument("args", nargs=argparse.REMAINDER, help="argumenty komendy bez zmian")
    args = parser.parse_args(argv)

    entry = resolve(args.entry)
    hook()
    try:
        server = start_server(args.preview_host, args.preview_port, args.preview_fps, args.preview_quality)
        print(f"[cam_preview] podglad: http://{socket.gethostname()}.local:{server.server_address[1]}/", flush=True)
    except OSError as e:  # port zajety: podglad jest dodatkiem, robot jedzie dalej
        server = None
        print(f"[cam_preview] BEZ podgladu, port {args.preview_port}: {e} (fuser -k {args.preview_port}/tcp)",
              file=sys.stderr, flush=True)

    sys.argv = [args.entry] + args.args  # lerobot (draccus) czyta argumenty z sys.argv
    try:
        rc = entry()
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
    return rc if isinstance(rc, int) else 0


if __name__ == "__main__":
    sys.exit(main())
