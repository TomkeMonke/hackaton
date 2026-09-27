"""
Uslugi panelu zbiorczego (tools/robot_panel.py): start/stop procesow, logi, stan, dorobek.

Service pilnuje jednego procesu (np. web_control.py): uruchamia go z przechwyceniem wyjscia,
trzyma ostatnie linie w pamieci i dopisuje je do logs/<nazwa>.log. Stan liczy tez z portu:
jesli port odpowiada, a proces nie jest nasz, usluga chodzi "zewnetrznie" (odpalona recznie
w SSH) - panel wtedy jej nie startuje drugi raz, bo port i urzadzenie sa zajete.

Zatrzymanie: najpierw SIGINT (web_control.py wysyla wtedy "a0 b0" do Xiao, arm_web.py
zamyka magistrale serw), po czasie terminate, na koncu kill.
"""
from __future__ import annotations

import io
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import zipfile
from collections import deque
from dataclasses import dataclass, field

MAX_LINES = 600
MAX_EVENTS = 200


@dataclass
class ServiceSpec:
    name: str                  # klucz w API i nazwa pliku logu
    title: str                 # napis w panelu
    argv: list[str]            # polecenie (pierwszy element zwykle sys.executable)
    port: int | None = None    # port, po ktorym widac, ze usluga zyje
    env: dict = field(default_factory=dict)
    link: int | None = None    # port strony uslugi do otwarcia w nowej karcie
    note: str = ""


def port_open(port: int | None, host: str = "127.0.0.1", timeout: float = 0.3) -> bool:
    if not port:
        return False
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class EventLog:
    """Kronika zdarzen panelu (start/stop uslug, wyjscia procesow)."""

    def __init__(self, wall=time.time, maxlen: int = MAX_EVENTS):
        self._wall = wall
        self._items: deque[dict] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._n = 0

    def add(self, kind: str, text: str) -> None:
        with self._lock:
            self._n += 1
            self._items.append({"id": self._n, "t": round(self._wall(), 3), "kind": kind, "text": text})

    def since(self, last_id: int = 0) -> list[dict]:
        with self._lock:
            return [e for e in self._items if e["id"] > last_id]


class Service:
    def __init__(self, spec: ServiceSpec, cwd: str, log_dir: str, events: EventLog | None = None,
                 probe=port_open, wall=time.time):
        self.spec = spec
        self.cwd = cwd
        self.log_dir = log_dir
        self.events = events or EventLog()
        self._probe = probe
        self._wall = wall
        self._lines: deque[str] = deque(maxlen=MAX_LINES)
        self._lock = threading.Lock()
        self.proc: subprocess.Popen | None = None
        self.started_at: float | None = None
        self.exit_code: int | None = None
        self._stopping = False

    @property
    def log_path(self) -> str:
        return os.path.join(self.log_dir, f"{self.spec.name}.log")

    def _append(self, line: str) -> None:
        stamped = time.strftime("%H:%M:%S ", time.localtime(self._wall())) + line.rstrip("\r\n")
        with self._lock:
            self._lines.append(stamped)
            try:
                os.makedirs(self.log_dir, exist_ok=True)
                with open(self.log_path, "a", encoding="utf-8", errors="replace") as fh:
                    fh.write(stamped + "\n")
            except OSError:
                pass  # pelny dysk nie moze zatrzymac uslugi; linia zostaje w pamieci

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> tuple[bool, str]:
        if self.running():
            return False, f"{self.spec.title}: juz chodzi"
        if self._probe(self.spec.port):
            return False, (f"{self.spec.title}: port {self.spec.port} zajety - usluga chodzi poza panelem "
                           "(okno SSH?). Zamknij ja tam albo korzystaj z niej tak, jak jest.")
        env = dict(os.environ)
        env.update({k: str(v) for k, v in self.spec.env.items()})
        env["PYTHONUNBUFFERED"] = "1"
        kwargs = {}
        if os.name == "nt":
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        try:
            self.proc = subprocess.Popen(
                self.spec.argv, cwd=self.cwd, env=env, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                errors="replace", bufsize=1, **kwargs)
        except OSError as exc:
            self.proc = None
            self.events.add("error", f"{self.spec.title}: nie wystartowal ({exc})")
            return False, f"{self.spec.title}: nie wystartowal ({exc})"
        self.started_at = self._wall()
        self.exit_code = None
        self._stopping = False
        self._append(f"=== start: {' '.join(self.spec.argv)}")
        threading.Thread(target=self._pump, args=(self.proc,), daemon=True,
                         name=f"log-{self.spec.name}").start()
        self.events.add("start", f"{self.spec.title}: start")
        return True, f"{self.spec.title}: start"

    def _pump(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            self._append(line)
        code = proc.wait()
        self.exit_code = code
        self._append(f"=== koniec, kod {code}")
        if self._stopping:
            self.events.add("stop", f"{self.spec.title}: zatrzymany")
        else:
            self.events.add("error", f"{self.spec.title}: proces sie zakonczyl (kod {code}) - zajrzyj w log")

    def stop(self, timeout: float = 4.0) -> tuple[bool, str]:
        proc = self.proc
        if proc is None or proc.poll() is not None:
            return False, f"{self.spec.title}: nie chodzi z panelu"
        self._stopping = True
        try:
            if os.name == "nt":
                proc.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                proc.send_signal(signal.SIGINT)
            proc.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, OSError, ValueError):
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
        return True, f"{self.spec.title}: zatrzymany"

    def restart(self) -> tuple[bool, str]:
        if self.running():
            self.stop()
            deadline = time.monotonic() + 3.0
            while self._probe(self.spec.port) and time.monotonic() < deadline:
                time.sleep(0.1)  # port zwalnia sie chwile po wyjsciu procesu
        return self.start()

    def tail(self, n: int = 200) -> list[str]:
        with self._lock:
            lines = list(self._lines)
        return lines[-n:] if n > 0 else lines

    def status(self) -> dict:
        ours = self.running()
        up = self._probe(self.spec.port) if self.spec.port else ours
        if ours:
            state = "running" if up or not self.spec.port else "starting"
        elif up:
            state = "external"
        elif self.exit_code not in (None, 0) and not self._stopping:
            state = "crashed"
        else:
            state = "stopped"
        return {
            "name": self.spec.name,
            "title": self.spec.title,
            "state": state,
            "port": self.spec.port,
            "link": self.spec.link,
            "note": self.spec.note,
            "pid": self.proc.pid if ours and self.proc else None,
            "uptime_s": round(self._wall() - self.started_at) if ours and self.started_at else None,
            "exit_code": self.exit_code,
            "cmd": " ".join(os.path.basename(a) if i == 0 else a for i, a in enumerate(self.spec.argv)),
        }


# --- zdrowie Pi -----------------------------------------------------------

def _read(path: str) -> str | None:
    try:
        with open(path, encoding="ascii", errors="replace") as fh:
            return fh.read()
    except OSError:
        return None


def parse_meminfo(text: str) -> dict:
    vals = {}
    for line in text.splitlines():
        m = re.match(r"(\w+):\s+(\d+)", line)
        if m:
            vals[m.group(1)] = int(m.group(2))
    total, avail = vals.get("MemTotal"), vals.get("MemAvailable")
    if not total or avail is None:
        return {}
    return {"mem_total_mb": round(total / 1024), "mem_used_pct": round(100 * (1 - avail / total), 1)}


def health(devices=("/dev/robot-drive", "/dev/robot-arm", "/dev/robot-leader")) -> dict:
    """Temperatura, obciazenie, pamiec, urzadzenia USB. Poza Linuksem pola sa None."""
    out: dict = {"host": socket.gethostname(), "platform": sys.platform}
    t = _read("/sys/class/thermal/thermal_zone0/temp")
    out["cpu_temp_c"] = round(int(t) / 1000, 1) if t and t.strip().isdigit() else None
    try:
        la = os.getloadavg()
        out["load1"] = round(la[0], 2)
    except (AttributeError, OSError):
        out["load1"] = None
    out["cpus"] = os.cpu_count()
    out.update(parse_meminfo(_read("/proc/meminfo") or ""))
    up = _read("/proc/uptime")
    out["uptime_s"] = round(float(up.split()[0])) if up else None
    thr = _read("/sys/devices/platform/soc/soc:firmware/get_throttled")
    out["throttled"] = thr.strip() if thr else None  # != 0x0 = spadek napiecia albo przegrzanie
    out["devices"] = {d: os.path.exists(d) for d in devices} if sys.platform.startswith("linux") else {}
    return out


# --- dorobek (liczby do sekcji "co zbudowalismy") --------------------------

def count_tests(tests_dir: str) -> int:
    n = 0
    for name in sorted(os.listdir(tests_dir)) if os.path.isdir(tests_dir) else []:
        if name.startswith("test_") and name.endswith(".py"):
            with open(os.path.join(tests_dir, name), encoding="utf-8", errors="replace") as fh:
                n += sum(1 for line in fh if re.match(r"\s*def test_", line))
    return n


def count_code_lines(dirs: list[str]) -> int:
    n = 0
    for d in dirs:
        for root, _dirs, files in os.walk(d):
            if "__pycache__" in root:
                continue
            for f in files:
                if f.endswith((".py", ".js", ".html", ".ino")):
                    with open(os.path.join(root, f), encoding="utf-8", errors="replace") as fh:
                        n += sum(1 for line in fh if line.strip())
    return n


def dataset_episodes(roots: list[str]) -> dict:
    """{nazwa: liczba epizodow} z meta/info.json datasetow lerobot."""
    out = {}
    for root in roots:
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            info = os.path.join(root, name, "meta", "info.json")
            try:
                with open(info, encoding="utf-8") as fh:
                    out[name] = int(json.load(fh).get("total_episodes", 0))
            except (OSError, ValueError, TypeError):
                continue
    return out


def git_facts(repo: str) -> dict:
    def run(*args):
        try:
            r = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, timeout=5)
            return r.stdout.strip() if r.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None

    commits = run("rev-list", "--count", "HEAD")
    authors = run("shortlog", "-sn", "HEAD")
    merges = run("rev-list", "--count", "--merges", "HEAD")
    return {
        "commits": int(commits) if commits and commits.isdigit() else None,
        "authors": len(authors.splitlines()) if authors else None,
        "merged_prs": int(merges) if merges and merges.isdigit() else None,
    }


def project_facts(repo: str) -> dict:
    motions_dir = os.path.join(repo, "motions")
    motions = sorted(f[:-5] for f in os.listdir(motions_dir) if f.endswith(".json")) \
        if os.path.isdir(motions_dir) else []
    tools_dir = os.path.join(repo, "tools")
    tools = sorted(f for f in os.listdir(tools_dir) if f.endswith(".py")) if os.path.isdir(tools_dir) else []
    facts = {
        "tests": count_tests(os.path.join(repo, "tests")),
        "code_lines": count_code_lines([os.path.join(repo, "pinecone_bot"), tools_dir]),
        "modules": sorted(f[:-3] for f in os.listdir(os.path.join(repo, "pinecone_bot"))
                          if f.endswith(".py") and f != "__init__.py"),
        "tools": len(tools),
        "motions": motions,
        "datasets": dataset_episodes([os.path.join(repo, "datasets"), os.path.expanduser("~/datasets")]),
    }
    facts.update(git_facts(repo))
    return facts


# --- paczka logow ---------------------------------------------------------

def logs_zip(services: list[Service], extra_files: list[str], status: dict | None = None) -> bytes:
    """ZIP: pelne logi uslug (plik albo pamiec), dodatkowe pliki (np. pinecone_log.csv), stan."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for s in services:
            if os.path.exists(s.log_path):
                zf.write(s.log_path, f"logs/{s.spec.name}.log")
            else:
                zf.writestr(f"logs/{s.spec.name}.log", "\n".join(s.tail(0)) + "\n")
        for path in extra_files:
            if os.path.isfile(path):
                zf.write(path, os.path.basename(path))
        if status is not None:
            zf.writestr("status.json", json.dumps(status, indent=2))
    return buf.getvalue()
