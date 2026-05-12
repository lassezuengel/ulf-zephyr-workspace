#!/usr/bin/env bash
set -euo pipefail

# Build two images: initiator and follower (separate build dirs)
ROOT=$(readlink -f "$(cd "$(dirname "$0")" && pwd)/../..")
EXPDIR="$ROOT/experiments/glossy-clock-sync"
EXTRA_MODULES="$ROOT/drivers;$ROOT/lib"
WEST_BIN="${WEST_BIN:-$ROOT/venv/bin/west}"
BOARD="${BOARD:-decawave_dwm3001cdk}"

echo "Building initiator image..."
"$WEST_BIN" build -p always -b "$BOARD" -d "$EXPDIR/build-init" -s "$EXPDIR" -- "-DCONF_FILE=prj_initiator.conf;prj.conf;uart.conf" "-DEXTRA_ZEPHYR_MODULES=$EXTRA_MODULES"

echo "Building follower image..."
"$WEST_BIN" build -p always -b "$BOARD" -d "$EXPDIR/build-follower" -s "$EXPDIR" -- "-DCONF_FILE=prj_follower.conf;prj.conf;uart.conf" "-DEXTRA_ZEPHYR_MODULES=$EXTRA_MODULES"

echo "Builds complete."
echo "Initiator ELF: $EXPDIR/build-init/zephyr/zephyr.elf"
echo "Follower ELF:  $EXPDIR/build-follower/zephyr/zephyr.elf"
