#!/bin/bash
set -e

RULES_SRC="/home/fresh/loki_ws_copy/udev/esp32-vesc.rules"
RULES_DST="/etc/udev/rules.d/99-esp32-vesc.rules"

if [ ! -f "$RULES_SRC" ]; then
    echo "ERROR: rules file not found at $RULES_SRC"
    exit 1
fi

echo "Symlinking udev rules..."
sudo ln -sf "$RULES_SRC" "$RULES_DST"

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Waiting for devices to settle..."
sudo udevadm settle
sleep 1

echo ""
echo "Current symlinks:"
ls -l /dev/esp32_1 /dev/esp32_2 /dev/vesc_1 /dev/vesc_2 2>/dev/null || \
    echo "(one or more symlinks missing — device may not be plugged in yet, or unplug/replug and rerun)"