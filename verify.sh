#!/usr/bin/env bash
# Compile-verify the firmware for the ESP32-S3 Feather (2MB PSRAM).
# Filters the ESP32 core's internal warnings and shows only sketch-relevant
# output plus the final pass/fail + size summary.
#
# Usage:  ./verify.sh            # compile only (verify)
#         ./verify.sh upload     # compile + upload (auto-detects port)
set -euo pipefail

cd "$(dirname "$0")"

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
  port="$(arduino-cli board list 2>/dev/null | awk '/serial/{print $1; exit}')"
  if [[ -z "$port" ]]; then echo "==> No serial board detected"; exit 1; fi
  echo "==> Uploading to $port"
  arduino-cli upload --fqbn "$FQBN" -p "$port" "$SKETCH"
fi
