#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

command -v pio >/dev/null || { echo "STLINK_INSTALL_FAIL: pio not found" >&2; exit 2; }

echo "[1/2] Flash application via ST-Link"
pio run -e APP_STLINK -t upload

echo "[2/2] Flash bootloader via ST-Link"
pio run -e BOOTLOADER_STLINK -t upload

echo "STLINK_INSTALL_PASS"
