#!/usr/bin/env bash
# One-shot setup of the robot's Raspberry Pi 5 (Raspberry Pi OS 64-bit).
#
# Run ON THE PI, as the normal user (not sudo), from the repo root:
#     bash deploy/setup_pi.sh
#
# Safe to re-run: every step skips what is already done.
# Covers: system packages, serial port access, udev names (/dev/robot-arm,
# /dev/robot-drive, RealSense), uv + Python 3.12 venv, requirements-pi.txt,
# pyrealsense2 (prebuilt wheel if one exists), and the robot-web autostart service.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY_VERSION=3.12
NEED_RELOGIN=""

step() { printf '\n==> %s\n' "$*"; }

if [[ $EUID -eq 0 ]]; then
  echo "Run as the normal user (e.g. robot), not with sudo."
  exit 1
fi
if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Expected 64-bit Raspberry Pi OS (aarch64), got $(uname -m)."
  exit 1
fi

step "System packages"
sudo apt-get update
sudo apt-get install -y git curl build-essential cmake pkg-config \
  libusb-1.0-0-dev libssl-dev libudev-dev python3-dev htop tmux

step "Serial port access (dialout group)"
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER"
  NEED_RELOGIN=1
fi
# ModemManager probes new ttyACM devices with AT commands and can grab the Xiao/arm.
if dpkg -s modemmanager >/dev/null 2>&1; then
  sudo apt-get purge -y modemmanager
fi

step "Stable device names: /dev/robot-arm, /dev/robot-drive, RealSense permissions"
sudo cp "$REPO_DIR/deploy/99-robot.rules" /etc/udev/rules.d/
if [[ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]]; then
  curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
    | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules >/dev/null
fi
sudo udevadm control --reload-rules
sudo udevadm trigger

step "uv (Python installer)"
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="$HOME/.local/bin:$PATH"

step "Python $PY_VERSION venv + libraries (lerobot pulls PyTorch, this takes a while)"
cd "$REPO_DIR"
[[ -d .venv ]] || uv venv --python "$PY_VERSION" .venv
uv pip install --python .venv/bin/python -r requirements-pi.txt
# No screen on the Pi: keep only headless OpenCV (the GUI build conflicts with it).
uv pip uninstall --python .venv/bin/python opencv-python 2>/dev/null || true

step "RealSense Python bindings (pyrealsense2)"
if .venv/bin/python -c "import pyrealsense2" 2>/dev/null; then
  echo "pyrealsense2 already importable."
elif uv pip install --python .venv/bin/python pyrealsense2; then
  echo "pyrealsense2 installed from a prebuilt wheel."
else
  echo "No prebuilt pyrealsense2 for this Pi: build librealsense from source"
  echo "(setup guide, section 'Kamera RealSense D415', 'Proba 2'). Everything else still works."
fi

step "Autostart service robot-web (web control panel)"
sed -e "s|^User=.*|User=$USER|" -e "s|/home/robot/hackaton|$REPO_DIR|g" \
  "$REPO_DIR/deploy/robot-web.service" | sudo tee /etc/systemd/system/robot-web.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable robot-web
sudo systemctl restart robot-web

step "Done"
IP="$(hostname -I | awk '{print $1}')"
echo "Panel:   http://$(hostname).local:8000   or   http://$IP:8000"
echo "Logs:    journalctl -u robot-web -f"
echo "Status:  systemctl status robot-web"
ls -l /dev/robot-* 2>/dev/null || echo "No /dev/robot-* yet: plug in the arm and the Xiao."
if [[ ! -f "$HOME/.cache/huggingface/lerobot/calibration/robots/so_follower/so101.json" ]]; then
  echo "Arm calibration missing: copy so101.json from the laptop (setup guide, 'Plik kalibracji ramienia')."
fi
if [[ -n "$NEED_RELOGIN" ]]; then
  echo "Log out and back in (or reboot) so scripts you start by hand can open serial ports."
fi
