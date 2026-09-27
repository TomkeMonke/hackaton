"""
Mapa RTAB-Map z nagrania `tools/record_rgbd.py` - jedna komenda na laptopie (Windows, bez GUI).

  python tools/rtabmap_build.py C:/Users/pawel/mapy/ogrod1
  python tools/rtabmap_build.py KATALOG --bin C:/Users/pawel/tools/bin --max-depth 4

Kroki (jak w docs/LOG.md 2026-09-25): rtabmap-dataRecorder -> rtabmap-reprocess -odom ->
rtabmap-detectMoreLoopClosures -> rtabmap-export, potem podglad `render_map_preview.py`.
Wynik w KATALOG: map.db, map_cloud.ply, map_poses.txt, map_preview.png.

RTAB-Map 0.23.8 win64 (zip z GitHuba introlab/rtabmap). Pulapka: narzedzia padaja z 0xC0000135
bez msvcr110.dll/msvcp110.dll (VC++ 2012) - wystarczy skopiowac oba (x64) do katalogu bin.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_BIN = os.path.join(os.path.expanduser("~"), "tools", "bin")

# strojenie odometrii z docs/LOG.md (najwiekszy spojny kawalek poprzedniego nagrania)
ODOM_ARGS = ["--RGBD/LinearUpdate", "0", "--RGBD/AngularUpdate", "0",
             "--Odom/ResetCountdown", "1", "--Vis/MinInliers", "12", "--Vis/CorType", "1",
             "--Vis/MaxFeatures", "2000"]


def source_ini(dataset: str) -> str:
    """Ini dla rtabmap-dataRecorder: zrodlo "RGB-D images" (driver 7), sciezki bezwzgledne."""
    d = os.path.abspath(dataset).replace("\\", "/")
    return "\n".join([
        "[Camera]",
        "type=0",
        "imageRate=0",
        "calibrationName=%s/calib/rs_color.yaml" % d,
        "RGBD\\driver=7",
        "RGBDImages\\path_rgb=%s/rgb" % d,
        "RGBDImages\\path_depth=%s/depth" % d,
        "RGBDImages\\scale=1",
        "RGBDImages\\start_pos=0",
        "Images\\stamps=%s/stamps.txt" % d,
        "",
    ])


def dataset_problem(dataset: str) -> str | None:
    """Niekompletna kopia (zerwane scp): RTAB-Map wtedy po cichu nie importuje nic."""
    n_rgb = len(os.listdir(os.path.join(dataset, "rgb")))
    n_depth = len(os.listdir(os.path.join(dataset, "depth")))
    with open(os.path.join(dataset, "stamps.txt"), encoding="utf-8") as f:
        n_st = sum(1 for line in f if line.strip())
    if not (n_rgb == n_depth == n_st):
        return "rgb %d, depth %d, stamps %d - kopia niekompletna, dociagnij brakujace pliki" % (
            n_rgb, n_depth, n_st)
    return None


def lost_frames(reprocess_log: str) -> tuple[int, int]:
    """(zgubione, wszystkie) z linii 'Processed N/M frames (... lost=true)' rtabmap-reprocess."""
    lines = re.findall(r"Processed (\d+)/(\d+) frames \(.*?lost=(true|false)\)", reprocess_log)
    total = int(lines[-1][1]) if lines else 0
    return sum(1 for _n, _m, lost in lines if lost == "true"), total


def graph_summary(info_log: str) -> dict:
    """Wezly w pamieci, pozy w grafie i liczba map (sesji odometrii) z rtabmap-info."""
    out = {}
    m = re.search(r"WM:\s+(\d+) nodes", info_log)
    out["nodes"] = int(m.group(1)) if m else 0
    m = re.search(r"Global graph:\s+(\d+) poses", info_log)
    out["poses"] = int(m.group(1)) if m else 0
    m = re.search(r"Maps in graph:\s+(\d+)/(\d+)", info_log)
    out["maps"] = int(m.group(2)) if m else 0
    return out


def run(cmd: list[str], cwd: str) -> str:
    print(">", " ".join(os.path.basename(c) if i == 0 else c for i, c in enumerate(cmd)), flush=True)
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, errors="replace")
    log = p.stdout + p.stderr
    if p.returncode == 0xC0000135 or p.returncode == -1073741515:
        raise RuntimeError("%s: brak DLL (0xC0000135) - skopiuj msvcr110.dll i msvcp110.dll (x64) "
                           "do katalogu bin RTAB-Map" % cmd[0])
    return log


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", help="katalog z record_rgbd.py (rgb/, depth/, calib/, stamps.txt)")
    ap.add_argument("--bin", default=DEFAULT_BIN, help="katalog bin RTAB-Map")
    ap.add_argument("--max-depth", type=float, default=4.0,
                    help="Vis/MaxDepth w m (na zewnatrz dalekie drzewa daja smieci)")
    ap.add_argument("--no-loops", action="store_true", help="pomin detectMoreLoopClosures")
    args = ap.parse_args()

    ds = os.path.abspath(args.dataset)
    for need in ("rgb", "depth", "calib/rs_color.yaml", "stamps.txt"):
        if not os.path.exists(os.path.join(ds, need)):
            print("brak %s w %s" % (need, ds))
            return 2
    problem = dataset_problem(ds)
    if problem:
        print(problem)
        return 2
    exe = lambda name: os.path.join(args.bin, name + ".exe")  # noqa: E731
    if not os.path.isfile(exe("rtabmap-reprocess")):
        print("nie ma RTAB-Map w %s (--bin)" % args.bin)
        return 2

    with open(os.path.join(ds, "rtabmap_source.ini"), "w", encoding="utf-8", newline="\n") as f:
        f.write(source_ini(ds))
    for old in ("raw.db", "map.db"):
        if os.path.exists(os.path.join(ds, old)):
            os.remove(os.path.join(ds, old))

    run([exe("rtabmap-dataRecorder"), "-hide", "rtabmap_source.ini", "raw.db"], ds)
    log = run([exe("rtabmap-reprocess"), "-odom"] + ODOM_ARGS
              + ["--Vis/MaxDepth", str(args.max_depth), "raw.db", "map.db"], ds)
    lost, total = lost_frames(log)
    print("odometria: zgubiona w %d z %d klatek" % (lost, total))
    if not args.no_loops:
        run([exe("rtabmap-detectMoreLoopClosures"), "--inter", "-r", "100", "-a", "180",
             "-i", "3", "map.db"], ds)
    run([exe("rtabmap-export"), "--cloud", "--poses", "--voxel", "0.01", "map.db"], ds)
    info = graph_summary(run([exe("rtabmap-info"), "map.db"], ds))
    print("graf: %d poz, %d wezlow, map (sesji odometrii): %d" % (info["poses"], info["nodes"],
                                                                 info["maps"]))
    if info["maps"] > 1:
        print("UWAGA: %d osobnych kawalkow - odometria sie zgubila i nie skleilo sie wszystko. "
              "Nagraj wolniej, z powrotem na start." % info["maps"])

    ply, poses = os.path.join(ds, "map_cloud.ply"), os.path.join(ds, "map_poses.txt")
    if os.path.exists(ply):
        subprocess.run([sys.executable, os.path.join(ROOT, "render_map_preview.py"), ply]
                       + ([poses] if os.path.exists(poses) else [])
                       + ["-o", os.path.join(ds, "map_preview.png"),
                          "--title", os.path.basename(ds)])
        print("gotowe: %s, %s" % (ply, os.path.join(ds, "map_preview.png")))
    else:
        print("brak chmury - sprawdz logi powyzej")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
