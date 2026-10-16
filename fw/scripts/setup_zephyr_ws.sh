#!/usr/bin/env bash
# Creates the Zephyr workspace fw/ builds against, outside the repo.
# Linux or WSL only: native_sim does not exist on Windows.
#
#   fw/scripts/setup_zephyr_ws.sh            -> ~/zephyr-ws, Zephyr v4.4.2
#   ZEPHYR_WS=/opt/zws fw/scripts/setup_zephyr_ws.sh
#
# Safe to rerun. Afterwards, fw/scripts/env.sh puts it on the path.

set -euo pipefail

WS=${ZEPHYR_WS:-$HOME/zephyr-ws}
ZEPHYR_REV=${ZEPHYR_REV:-v4.4.2}

mkdir -p "$WS"
if [ ! -x "$WS/.venv/bin/python" ]; then
	python3 -m venv "$WS/.venv"
fi
# shellcheck disable=SC1091
. "$WS/.venv/bin/activate"
pip install -q --upgrade pip west

if [ ! -d "$WS/.west" ]; then
	west init -m https://github.com/zephyrproject-rtos/zephyr --mr "$ZEPHYR_REV" "$WS"
fi
cd "$WS"

# Only the HAL the ESP32 needs. native_sim needs no modules at all, and the
# full manifest is several GB of vendor HALs nothing here uses.
west config manifest.project-filter -- '-.*,+hal_espressif'
west update --narrow -o=--depth=1

# Base for west build, build-test and run-test for twister, esptool for
# flashing the ESP32.
pip install -q -r zephyr/scripts/requirements-base.txt \
	-r zephyr/scripts/requirements-build-test.txt \
	-r zephyr/scripts/requirements-run-test.txt esptool

# Espressif ships the radio and bootloader pieces as binary blobs, outside git
west blobs fetch hal_espressif

# Host gcc builds native_sim, so the only cross toolchain needed is the ESP32 one
west sdk install -t xtensa-espressif_esp32_zephyr-elf

echo "workspace ready: $WS"
