#!/bin/bash
# Set up Spit-electronics on a Raspberry Pi.
# Assumes Node-Red is already installed (run nodered.org installer first if not).
# Safe to re-run.
#
# Usage:
#   ./RPi/install.sh              # configure Spit; apt update only (no upgrade)
#   ./RPi/install.sh --upgrade    # also run `apt upgrade -y` (system-wide)

set -euo pipefail

RUN_UPGRADE=0
for arg in "$@"; do
  case "$arg" in
    --upgrade) RUN_UPGRADE=1 ;;
    -h|--help)
      sed -n '2,9p' "$0"
      exit 0
      ;;
    *)
      echo "!! Unknown argument: $arg"
      sed -n '2,9p' "$0"
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FLOWS_PATH="$REPO_ROOT/RPi/node-red/flows.json"
CREDS_PATH="$REPO_ROOT/RPi/node-red/flows_cred.json"
START_SCRIPT="$REPO_ROOT/RPi/bash_scripts/start_spit.sh"
SETTINGS="$HOME/.node-red/settings.js"

if [ "$EUID" -eq 0 ]; then
  echo "!! Don't run this as root. Run as the 'pi' user; the script will sudo where needed."
  exit 1
fi

echo "==> Spit install from $REPO_ROOT"

echo "==> apt update"
sudo apt update
if [ "$RUN_UPGRADE" -eq 1 ]; then
  echo "==> apt upgrade (system-wide, slow)"
  sudo apt upgrade -y
else
  echo "    (skipping apt upgrade; pass --upgrade to run it)"
fi

echo "==> Installing Mosquitto"
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto.service

echo "==> Installing Python + pip"
sudo apt install -y python3 python3-pip

echo "==> Installing Python packages"
PIP_FLAGS=""
# Bookworm+ marks the system Python as externally managed (PEP 668); pip needs
# --break-system-packages to install into it. Older Pi OS doesn't recognise the flag.
if compgen -G "/usr/lib/python3*/EXTERNALLY-MANAGED" >/dev/null; then
  PIP_FLAGS="--break-system-packages"
fi
pip3 install $PIP_FLAGS paho-mqtt pySerialTransfer numpy flask

echo "==> Checking for Node-Red"
if ! command -v node-red >/dev/null; then
  echo "!! Node-Red not found. Install it via:"
  echo "   bash <(curl -sL https://raw.githubusercontent.com/node-red/linux-installers/master/deb/update-nodejs-and-nodered)"
  echo "   then re-run this script."
  exit 1
fi
sudo systemctl enable nodered.service

if [ ! -f "$SETTINGS" ]; then
  echo "==> Bootstrapping Node-Red to create settings.js"
  sudo systemctl restart nodered.service
  for _ in $(seq 1 30); do
    [ -f "$SETTINGS" ] && break
    sleep 1
  done
  if [ ! -f "$SETTINGS" ]; then
    echo "!! settings.js was not created after 30s. Inspect 'journalctl -u nodered' and re-run."
    exit 1
  fi
fi

echo "==> Pointing Node-Red at repo flows"
if grep -qE "^[[:space:]]*flowFile:[[:space:]]*'$FLOWS_PATH'" "$SETTINGS"; then
  echo "   already set."
else
  cp "$SETTINGS" "$SETTINGS.bak.$(date +%s)"
  if grep -qE '^[[:space:]]*(//[[:space:]]*)?flowFile:' "$SETTINGS"; then
    sed -i -E "s|^[[:space:]]*(//[[:space:]]*)?flowFile:.*|    flowFile: '$FLOWS_PATH',|" "$SETTINGS"
  else
    sed -i "/module.exports = {/a\\    flowFile: '$FLOWS_PATH'," "$SETTINGS"
  fi
  if grep -qE '^[[:space:]]*(//[[:space:]]*)?credentialsFile:' "$SETTINGS"; then
    sed -i -E "s|^[[:space:]]*(//[[:space:]]*)?credentialsFile:.*|    credentialsFile: '$CREDS_PATH',|" "$SETTINGS"
  else
    sed -i "/module.exports = {/a\\    credentialsFile: '$CREDS_PATH'," "$SETTINGS"
  fi
fi

echo "==> Restarting Node-Red"
sudo systemctl restart nodered.service

chmod +x "$START_SCRIPT"

echo "==> Crontab entry for $START_SCRIPT"
if crontab -l 2>/dev/null | grep -qF "$START_SCRIPT"; then
  echo "   already in crontab."
else
  (crontab -l 2>/dev/null; echo "@reboot sleep 10 $START_SCRIPT >> /home/pi/crontab_log.txt 2>&1") | crontab -
fi

echo "==> Installing spit-viewer systemd service"
# Substitute the absolute repo path into the templated unit file. Re-installing
# is cheap (sed + write + reload), so we always overwrite — keeps the unit in
# sync if REPO_ROOT moves between runs.
VIEWER_UNIT_SRC="$REPO_ROOT/RPi/viewer/spit-viewer.service"
VIEWER_UNIT_DST="/etc/systemd/system/spit-viewer.service"
if [ ! -f "$VIEWER_UNIT_SRC" ]; then
  echo "!! Missing $VIEWER_UNIT_SRC — skipping viewer service install."
else
  sudo sed "s|__REPO_ROOT__|$REPO_ROOT|g" "$VIEWER_UNIT_SRC" | sudo tee "$VIEWER_UNIT_DST" >/dev/null
  sudo systemctl daemon-reload
  sudo systemctl enable --now spit-viewer.service
  echo "   viewer running at http://$(hostname -I | awk '{print $1}'):5000"
fi

echo
echo "==> Done. Reboot the Pi to verify auto-start, then check:"
echo "     sudo systemctl status nodered mosquitto spit-viewer"
echo "     tail -f /home/pi/crontab_log.txt"
