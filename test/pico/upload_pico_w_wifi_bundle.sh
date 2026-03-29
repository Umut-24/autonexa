#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyACM0}"
BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/pico_w_wifi_bundle" && pwd)"

if ! command -v mpremote >/dev/null 2>&1; then
  echo "ERROR: mpremote is not installed."
  echo "Install with: python3 -m pip install --user mpremote"
  exit 1
fi

echo "Uploading Pico W Wi-Fi direct control bundle to ${PORT}"
mpremote connect "${PORT}" fs cp "${BUNDLE_DIR}/boot.py" :boot.py
mpremote connect "${PORT}" fs cp "${BUNDLE_DIR}/config.py" :config.py
mpremote connect "${PORT}" fs cp "${BUNDLE_DIR}/main.py" :main.py
mpremote connect "${PORT}" reset

echo "Upload complete."
echo "If AP mode is enabled, connect phone to SSID from config.py and use:"
echo "  http://192.168.4.1:5001"
