#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT/firmware/esp32"

DEVICE="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_30:ED:A0:2A:CE:10-if00"

pio run -t upload --upload-port "$DEVICE"