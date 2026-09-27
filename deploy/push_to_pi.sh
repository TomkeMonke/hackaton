#!/usr/bin/env bash
# Push the pinecone_bot/ deterministic stack to the Raspberry Pi.
#
# Run FROM THE LAPTOP (or any dev machine with the repo checked out), from
# the repo root:
#     bash deploy/push_to_pi.sh
#
# Copies: pinecone_bot/, tools/, motions/, sequences/, tests/, requirements-pinecone.txt,
# arm_control.py, web_control.py, frontend.html, arm_panel.html/.js
# and pinecone_config.json (if it exists locally). Uses rsync when
# available (fast, deletes files removed locally, resumable); falls back to
# scp -r otherwise (plain copy, no delete).
#
# Config via env vars:
#     PI_HOST          ssh target, default "pi@raspberrypi.local"
#     PI_DIR           remote repo dir, default "~/hackaton"
#     PUSH_ANY_BRANCH  set to 1 to skip the non-master branch warning below
#
# Examples:
#     PI_HOST=robot@192.168.1.42 bash deploy/push_to_pi.sh
#     PI_HOST=pi@raspberrypi.local PI_DIR=~/hackaton bash deploy/push_to_pi.sh
#
# This script does NOT run deploy/setup_pi.sh or install anything on the
# Pi - it only copies files. Run the printed next command on the Pi
# afterwards.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOST="${PI_HOST:-pi@raspberrypi.local}"
PI_DIR="${PI_DIR:-~/hackaton}"

CURRENT_BRANCH="$(git -C "$REPO_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
if [[ "$CURRENT_BRANCH" != "master" && "${PUSH_ANY_BRANCH:-0}" != "1" ]]; then
  echo "UWAGA: wysylasz branch '$CURRENT_BRANCH', a na Pi powinien byc master. Ctrl+C aby przerwac (5 s)..."
  sleep 5
fi

step() { printf '\n==> %s\n' "$*"; }

cd "$REPO_DIR"

# arm_control.py: WaypointArm i tools/arm_web.py; web_control.py + frontend.html + arm_panel.*: panel jazdy i ramienia;
# sequences/: zhardkodowane sekwencje jazda + ramie z panelu
ITEMS=(pinecone_bot tools motions sequences tests so101_urdf requirements-pinecone.txt
       arm_control.py web_control.py frontend.html arm_panel.html arm_panel.js)
if [[ -f pinecone_config.json ]]; then
  ITEMS+=(pinecone_config.json)
else
  echo "Note: no local pinecone_config.json - skipping (Pi keeps its own if it has one)."
fi

step "Target: $PI_HOST:$PI_DIR"

if command -v rsync >/dev/null 2>&1; then
  step "Copying with rsync"
  ssh "$PI_HOST" "mkdir -p $PI_DIR"
  rsync -avz --progress --delete \
    --exclude '.venv' \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude 'frames/' \
    "${ITEMS[@]}" \
    "$PI_HOST:$PI_DIR/"
else
  step "rsync not found, falling back to scp -r (no delete, no exclude filters"
  echo "    - remove stray .venv/__pycache__/frames/ on the Pi by hand if needed)"
  ssh "$PI_HOST" "mkdir -p $PI_DIR"
  for item in "${ITEMS[@]}"; do
    scp -r "$item" "$PI_HOST:$PI_DIR/"
  done
fi

step "Done"
echo "Next, on the Pi:"
echo "    ssh $PI_HOST"
echo "    cd $PI_DIR"
echo "    uv pip install --python .venv/bin/python -r requirements-pinecone.txt"
echo "    python -m pytest tests -q"
echo "    python -m pinecone_bot.main --dry-run     # --show tylko z pulpitem (nie z opencv-python-headless)"
