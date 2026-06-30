#!/usr/bin/env bash
# Compile-verify the firmware for the ESP32-S3 Feather (2MB PSRAM).
# Filters the ESP32 core's internal warnings and shows only sketch-relevant
# output plus the final pass/fail + size summary.
#
# Usage:  ./verify.sh            # compile only (verify)
#         ./verify.sh upload     # compile + upload (auto-detects port)
set -euo pipefail

# Resolve to the real-case, symlink-free path. macOS is case-insensitive, so
# `cd ~/airroaster` works but hands arduino-cli a lowercase folder name, which
# it can't match against airRoaster.ino. realpath canonicalizes the case from
# the filesystem (unlike `pwd -P`, which trusts the stale $PWD env hint).
cd "$(realpath "$(dirname "$0")")"

FQBN="esp32:esp32:adafruit_feather_esp32s3"
SKETCH="airRoaster.ino"

echo "==> Compiling $SKETCH for $FQBN"
# --warnings all surfaces sketch issues; we grep our own file out of the noise.
out="$(arduino-cli compile --fqbn "$FQBN" --warnings all "$SKETCH" 2>&1)" || {
  echo "$out" | grep -iE "error:|airRoaster" || true
  echo "==> BUILD FAILED"
  exit 1
}

# Surface any warning/error that references our sketch (should be none).
sketch_msgs="$(echo "$out" | grep -iE "airRoaster.*(warning|error)" || true)"
if [[ -n "$sketch_msgs" ]]; then
  echo "==> Sketch warnings/errors:"
  echo "$sketch_msgs"
else
  echo "==> No warnings or errors in the sketch."
fi

echo "$out" | grep -E "Sketch uses|Global variables" || true
echo "==> BUILD OK"

if [[ "${1:-}" == "upload" ]]; then
  # Prefer the port arduino-cli identifies as our FQBN; otherwise fall back to
  # the first USB serial port. Never match the Bluetooth ports (cu.BLTH,
  # cu.Bluetooth-*), which also report protocol "serial" but aren't the board.
  list="$(arduino-cli board list 2>/dev/null)"
  port="$(awk -v fqbn="$FQBN" '$1 ~ /^\/dev\// && index($0, fqbn) {print $1; exit}' <<<"$list")"
  if [[ -z "$port" ]]; then
    port="$(awk '$1 ~ /^\/dev\/cu\.(usbmodem|usbserial|wchusbserial|SLAB)/ {print $1; exit}' <<<"$list")"
  fi
  if [[ -z "$port" ]]; then echo "==> No ESP32 serial port detected"; echo "$list"; exit 1; fi
  echo "==> Uploading to $port"
  arduino-cli upload --fqbn "$FQBN" -p "$port" "$SKETCH"
fi
