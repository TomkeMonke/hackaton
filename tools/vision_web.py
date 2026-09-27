"""
Serwer kamery dla panelu: podglad MJPEG z ramkami szyszek + liczba szyszek w JSON.

Uzycie na Pi (z katalogu repo):
    python tools/vision_web.py                         # RealSense, http://<IP_PI>:8020
    python tools/vision_web.py --source sim            # symulator, bez sprzetu (pokaz na laptopie)
    python tools/vision_web.py --source frames/        # klatki z dysku
    python tools/vision_web.py --source http://127.0.0.1:8081/cam/front.jpg   # obraz z innego procesu

Adresy:
    /                     prosta strona podgladu
    /stream?view=overlay  MJPEG; view: overlay | raw | mask | depth
    /frame.jpg?view=...   jedna klatka
    /api/detections       {"count", "partial", "nearest_m", "detections", "fps", "history", ...}
    POST /api/snapshot    zapis surowej klatki do frames/panel/ (strojenie HSV)

Kamere RealSense moze trzymac tylko jeden proces: nie odpalac rownolegle z rs_mjpeg_server.py,
pinecone_bot.main ani lerobot-record (wtedy --source <URL podgladu lerobot>).
GET-y maja CORS "*": tylko odczyt obrazu i liczb, bez sterowania.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from pinecone_bot.config import Config  # noqa: E402
from pinecone_bot.detector import HsvConeDetector  # noqa: E402
from pinecone_bot.vision_feed import VIEWS, HttpJpegCamera, VisionFeed, sim_camera  # noqa: E402

HTTP_PORT = 8020
SNAP_DIR = os.path.join(REPO_ROOT, "frames", "panel")

INDEX = b"""<!doctype html><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Kamera</title><body style="margin:0;background:#111;color:#eee;font-family:sans-serif">
<img src="/stream" style="width:100%;display:block"><div id="n" style="padding:8px 12px;font-size:20px"></div>
<script>setInterval(async()=>{try{const s=await (await fetch('/api/detections')).json();
document.getElementById('n').textContent='szyszki: '+s.count+(s.partial?' (+'+s.partial+' uciete)':'')+'  '+s.fps+' fps';}catch(e){}},500)</script>"""


def make_camera_from_arg(cfg: Config, source: str | None):
    if source is None:
        from pinecone_bot.camera import make_camera
        return make_camera(cfg, None, depth=True)
    if source == "sim":
        return sim_camera(cfg)
    if source.startswith(("http://", "https://")):
        return HttpJpegCamera(source)
    from pinecone_bot.camera import FileCamera
    return FileCamera(source)


def make_handler(feed: VisionFeed, snap_dir: str = SNAP_DIR):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # panel odpytuje co 300 ms; bez tego konsola tonie
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload: dict) -> None:
            self._send(code, json.dumps(payload).encode("ascii"), "application/json")

        def do_GET(self):
            url = urlparse(self.path)
            view = (parse_qs(url.query).get("view") or ["overlay"])[0]
            if view not in VIEWS:
                view = "overlay"
            if url.path == "/":
                self._send(200, INDEX, "text/html; charset=utf-8")
            elif url.path == "/api/detections":
                self._json(200, feed.summary())
            elif url.path == "/frame.jpg":
                _, data = feed.jpeg(view)
                if data is None:
                    self._json(503, {"ok": False, "msg": feed.error or "brak klatki"})
                else:
                    self._send(200, data, "image/jpeg")
            elif url.path == "/stream":
                self._stream(view)
            else:
                self._json(404, {"ok": False, "msg": "nie ma"})

        def _stream(self, view: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seq = -1
            try:
                while True:
                    seq = feed.wait_frame(seq, timeout=2.0)
                    _, data = feed.jpeg(view)
                    if data is None:
                        time.sleep(0.5)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(data + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                pass  # przegladarka zamknela karte

        def do_POST(self):
            if urlparse(self.path).path != "/api/snapshot":
                self._json(404, {"ok": False, "msg": "nie ma"})
                return
            frame = feed.raw_frame()
            if frame is None:
                self._json(503, {"ok": False, "msg": "brak klatki"})
                return
            os.makedirs(snap_dir, exist_ok=True)
            name = time.strftime("panel_%Y%m%d_%H%M%S.png")
            cv2.imwrite(os.path.join(snap_dir, name), frame)
            self._json(200, {"ok": True, "msg": f"zapisano frames/panel/{name}"})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=None,
                        help="brak = RealSense; 'sim'; plik/katalog/wideo; http://...jpg")
    parser.add_argument("--host", default=os.environ.get("ROBOT_HOST", "0.0.0.0"))
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    parser.add_argument("--hz", type=float, default=10.0, help="ile klatek na sekunde przez detektor")
    parser.add_argument("--quality", type=int, default=70, help="jakosc JPEG")
    args = parser.parse_args()

    cfg = Config.load()
    camera = make_camera_from_arg(cfg, args.source)
    # symulator rysuje szyszki brazem pod domyslny prog z config.py; prog z pinecone_config.json
    # jest dobrany do prawdziwych szyszek (odcien 130..179) i na symulatorze nic by nie widzial
    det_cfg = Config().detector if args.source == "sim" else cfg.detector
    feed = VisionFeed(camera, HsvConeDetector(det_cfg), hz=args.hz, jpeg_quality=args.quality,
                      source=args.source or "realsense")
    feed.start()
    httpd = ThreadingHTTPServer((args.host, args.http_port), make_handler(feed))
    httpd.daemon_threads = True
    print(f"kamera: http://<IP>:{args.http_port} (zrodlo {feed.source}, {args.hz} Hz)", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("koniec")
    finally:
        httpd.server_close()
        feed.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
