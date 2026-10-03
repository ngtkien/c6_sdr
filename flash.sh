#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"
while [[ "$WORKSPACE_ROOT" != "/" && ! -f "$WORKSPACE_ROOT/zephyr-env.sh" ]]; do
    WORKSPACE_ROOT="$(dirname "$WORKSPACE_ROOT")"
done

# Source Zephyr environment
if [[ -f "$WORKSPACE_ROOT/zephyr-env.sh" ]]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE_ROOT/zephyr-env.sh"
fi

PORT="${1:-${ESPPORT:-/dev/ttyUSB0}}"
BAUD="${BAUD:-460800}"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"

MCUBOOT_BIN="$BUILD_DIR/mcuboot/zephyr/zephyr.bin"
APP_BIN="$BUILD_DIR/c6_sdr/zephyr/zephyr.signed.bin"

if [[ ! -f "$MCUBOOT_BIN" || ! -f "$APP_BIN" ]]; then
    echo "ERROR: Binaries not found in $BUILD_DIR. Please run ./build.sh first." >&2
    exit 1
fi

echo "=========================================="
echo " Flashing ESP32-P4 C6-SDR front-end"
echo " Port:       $PORT"
echo " Baud:       $BAUD"
echo " MCUboot:    $MCUBOOT_BIN (0x2000)"
echo " App:        $APP_BIN (0x20000)"
echo "=========================================="

esptool \
    --chip esp32p4 \
    --port "$PORT" \
    --baud "$BAUD" \
    write-flash \
    --flash-mode dio \
    --flash-freq 80m \
    --flash-size 16MB \
    0x2000 "$MCUBOOT_BIN" \
    0x20000 "$APP_BIN"

echo ""
echo "Flashing finished successfully!"
echo "Starting serial monitor on $PORT (Ctrl+] to exit)..."
echo ""

cd "$BUILD_DIR/c6_sdr"
west espressif monitor --port "$PORT" || true
