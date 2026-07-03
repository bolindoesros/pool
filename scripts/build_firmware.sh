#!/usr/bin/env bash
set -e

cd firmware/esp32

if [ "$CLEAN_BUILD" = "1" ] || [ "$1" = "--clean" ] || [ "$1" = "--nuke" ]; then
  echo "Nuclear cleaning PlatformIO firmware build..."
  rm -rf .pio
fi

pio run./