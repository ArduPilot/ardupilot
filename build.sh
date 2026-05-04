#!/bin/bash
# Builds firmware for targets listed below if no args passed.
# If args are passed, builds firmware for those targets only.

set -euo pipefail

# List of flight controller targets (ArduPilot hwdef board names)
TARGETS=(
    "speedybeef4v4"
    "MatekH743"
)

# Vehicles to build for each target
VEHICLES=(
    "copter"
    "plane"
)

if [ "$#" -gt 0 ]; then
    TARGETS=("$@")
fi

WAF=./modules/waf/waf-light
OUT_DIR=ardupilot
rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

for TARGET in "${TARGETS[@]}"; do
    echo "::group::Configure $TARGET"
    "$WAF" configure --board="$TARGET"
    echo "::endgroup::"

    echo "::group::Clean $TARGET"
    "$WAF" clean
    echo "::endgroup::"

    for VEHICLE in "${VEHICLES[@]}"; do
        echo "::group::Build $TARGET / $VEHICLE"
        "$WAF" "$VEHICLE"
        echo "::endgroup::"
    done

    BIN_DIR="build/$TARGET/bin"
    DEST="$OUT_DIR/$TARGET"
    mkdir -p "$DEST"
    # Collect flashable artifacts: .apj (ArduPilot uploader), .bin (raw),
    # *_with_bl.hex (bootloader + firmware combined, for from-scratch flashing).
    shopt -s nullglob
    cp "$BIN_DIR"/*.apj "$DEST"/ 2>/dev/null || true
    cp "$BIN_DIR"/*.bin "$DEST"/ 2>/dev/null || true
    cp "$BIN_DIR"/*_with_bl.hex "$DEST"/ 2>/dev/null || true
    shopt -u nullglob

    echo "Artifacts for $TARGET:"
    ls -la "$DEST"
done

echo "All builds finished."
