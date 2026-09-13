#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PIO_HOME="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
OPENOCD="$PIO_HOME/packages/tool-openocd/bin/openocd"
OCD_SCRIPTS="$PIO_HOME/packages/tool-openocd/openocd/scripts"
APP_BIN="$ROOT/.pio/build/APP_STLINK/firmware.bin"
BOOT_BIN="$ROOT/.pio/build/BOOTLOADER_STLINK/firmware.bin"

[[ -x "$OPENOCD" ]] || { echo "STLINK_INSTALL_FAIL: OpenOCD not found" >&2; exit 2; }
command -v pio >/dev/null || { echo "STLINK_INSTALL_FAIL: pio not found" >&2; exit 2; }
python3 "$ROOT/tools/stlink_target_guard.py" --openocd "$OPENOCD" --scripts "$OCD_SCRIPTS" --speed 100
echo "[1/7] Build relocated application"
pio run -e APP_STLINK

echo "[2/7] Build resident bootloader"
pio run -e BOOTLOADER_STLINK

[[ -f "$APP_BIN" && -f "$BOOT_BIN" ]] || {
  echo "STLINK_INSTALL_FAIL: build products missing" >&2
  exit 2
}

mkdir -p "$ROOT/backups"
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$ROOT/backups/f103_full_before_bootloader_${STAMP}.bin"
OCD=("$OPENOCD" -s "$OCD_SCRIPTS" -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg)

echo "[3/7] Backup complete 256-KiB flash before any write"
"${OCD[@]}" -c "init; reset halt; dump_image {$BACKUP} 0x08000000 0x40000; shutdown"
SIZE="$(stat -c '%s' "$BACKUP")"
[[ "$SIZE" == "262144" ]] || {
  echo "STLINK_INSTALL_FAIL: backup size $SIZE, expected 262144" >&2
  exit 3
}
sha256sum "$BACKUP"

echo "[4/7] Program relocated application BIN at explicit 0x08002800 and verify"
"${OCD[@]}" -c "program {$APP_BIN} 0x08002800 verify; shutdown"
echo "[5/7] Clear OTA metadata then program immutable bootloader BIN and verify"
# A previous interrupted/recovery OTA must not trap a freshly installed,
# verified application in the resident bootloader. Erase metadata only;
# preserve the 4-KiB EEPROM region at 0x0803F000.
"${OCD[@]}" -c "init; reset halt; flash erase_address 0x0803E800 0x800; shutdown"
"${OCD[@]}" -c "program {$BOOT_BIN} 0x08000000 verify reset; shutdown"

echo "[6/7] Verify application boots after metadata clear"
"${OCD[@]}" -c "init; reset run; sleep 250; halt; reg pc; reset run; shutdown"
echo "[7/7] Installation complete"
echo "STLINK_INSTALL_PASS"
echo "BACKUP=$BACKUP"
echo "Next: verify COMM_FW_VERSION over direct USART3 USB-UART before any motor command."
