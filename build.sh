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
else
    echo "ERROR: zephyr-env.sh not found. Run ./zephyr-bootstrap/setup-zephyr.sh from workspace root." >&2
    exit 1
fi

BOARD="${BOARD:-esp32p4_function_ev_v1/esp32p4/hpcore}"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"

echo "=========================================="
echo " Building ESP32-P4 -> C6 SDR"
echo " Board:     $BOARD"
echo " Build Dir: $BUILD_DIR"
echo "=========================================="

west build \
    --sysbuild \
    --pristine=auto \
    --board "$BOARD" \
    --build-dir "$BUILD_DIR" \
    "$SCRIPT_DIR" "$@"

echo ""
echo "Build complete! To flash to ESP32-P4, run:"
echo "  ./flash.sh            # default port /dev/ttyUSB0"
echo "  ./flash.sh /dev/ttyX  # or ESPPORT=/dev/ttyX ./flash.sh"
echo ""
echo "Flash c6_firmware on the C6 first (hosted v1.4.7 + SDR engine):"
echo "  c6_firmware/build/network_adapter.bin"
