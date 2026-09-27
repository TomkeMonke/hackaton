"""
Awaryjny STOP w przegladarce na czas testow podwozia (base_test.py, pinecone_bot.main).

Panel web_control.py trzyma port Xiao, wiec nie moze chodzic rownolegle z testami.
Ten serwer portu nie trzyma: STOP zabija procesy testowe (bez nich Xiao i tak staje
po 500 ms na watchdogu), a potem sam otwiera port i wysyla "a0 b0", zeby stanac od razu.

Uzycie na Pi (z katalogu repo):
    python tools/estop_server.py            # http://<IP_PI>:8001
Spacja albo Enter na stronie = STOP.
"""
from __future__ import annotations

import os
import signal
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

PORT = int(os.environ.get("ESTOP_PORT", "8001"))
DRIVE_PORT = os.environ.get("ROBOT_DRIVE_PORT", "/dev/robot-drive")
TARGETS = ("tools/base_test.py", "pinecone_bot.main")

PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>STOP</title><style>
body{margin:0;height:100vh;display:flex;flex-direction:column;background:#111;color:#eee;font-family:sans-serif}
button{flex:1;margin:16px;border:0;border-radius:24px;background:#d11;color:#fff;font-size:18vw;font-weight:700}
button:active{background:#900}#s{padding:0 16px 16px;font-size:18px;min-height:1.5em}
</style></head><body><button id="b">STOP</button><div id="s">spacja / Enter = STOP</div><script>
const s=document.getElementById('s');
async function stop(){s.textContent='zatrzymuje...';
 try{const r=await fetch('/stop',{method:'POST'});s.textContent=await r.text();}
 catch(e){s.textContent='BLAD polaczenia - wylacz zasilanie silnikow recznie!';}}
document.getElementById('b').onclick=stop;
addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Enter'){e.preventDefault();stop();}});
</script></body></html>"""


def find_targets() -> list[tuple[int, str]]:
    found = []
    me = os.getpid()
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "python" in cmd and any(t in cmd for t in TARGETS):
            found.append((int(pid), cmd.strip()))
    return found


def emergency_stop() -> str:
    killed = []
    for pid, cmd in find_targets():
        try:
            os.kill(pid, signal.SIGKILL)
            killed.append(pid)
        except ProcessLookupError:
            pass
    time.sleep(0.2)  # port zwalnia sie po smierci procesu
    try:
        with serial.Serial(DRIVE_PORT, 115200, timeout=0, write_timeout=0.5) as ser:
            for _ in range(5):
                ser.write(b"a0 b0\n")
                time.sleep(0.02)
        sent = "a0 b0 wyslane"
    except (serial.SerialException, OSError) as exc:
        sent = f"nie wyslano a0 b0 ({exc}); Xiao i tak staje po 500 ms bez komend"
    msg = f"STOP {time.strftime('%H:%M:%S')}: zabite {killed or 'nic'}, {sent}"
    print(msg, flush=True)
    return msg


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(PAGE)

    def do_POST(self):
        if self.path != "/stop":
            self.send_error(404)
            return
        body = emergency_stop().encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    print(f"STOP na http://0.0.0.0:{PORT}  (port Xiao: {DRIVE_PORT})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
