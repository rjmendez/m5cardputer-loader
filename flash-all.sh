#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 p0rtm0lester
# Flash the M5Cardputer Loader.
#   ./flash-all.sh app    -> flash ONLY the loader app (0x10000). Fast; for app-code changes.
#                            The hooked bootloader + partitions stay untouched.
#   ./flash-all.sh full   -> flash hooked bootloader + partitions + wifi creds + app,
#                            then clear bootflag/otadata for a clean boot to the loader.
#                            Use after build-bootloader.sh or a partition-table change.
#
# Prereq for app:  pio run -e cardputer
# Prereq for full: ./build-bootloader.sh   (produces bl_build/bootloader.bin)
set -e
PORT="${PORT:-/dev/ttyACM0}"
P="$(cd "$(dirname "$0")" && pwd)"
ESPTOOL="$HOME/.platformio/packages/tool-esptoolpy/esptool.py"
FW="$P/.pio/build/cardputer/firmware.bin"
PT="$P/.pio/build/cardputer/partitions.bin"
BL="$P/bl_build/bootloader.bin"
WIFI="${WIFI:-/tmp/wifi_nvs_4000.bin}"   # optional 0x4000 NVS creds image (namespace "loader")
                                         # NOTE: contains your WiFi SSID/PSK in the clear.
                                         # It is shredded after a successful 'full' flash
                                         # unless you set KEEP_WIFI=1.
FLASH="--flash_mode dio --flash_freq 80m --flash_size 8MB"

mode="${1:-app}"
[ -f "$FW" ] || { echo "build the app first:  pio run -e cardputer"; exit 1; }

if [ "$mode" = "app" ]; then
    python3 "$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 921600 \
        --before default_reset --after hard_reset write_flash $FLASH 0x10000 "$FW"
elif [ "$mode" = "full" ]; then
    [ -f "$BL" ] || { echo "build the bootloader first:  ./build-bootloader.sh"; exit 1; }
    ARGS="0x0 $BL 0x8000 $PT 0x10000 $FW"
    wifi_used=0
    [ -f "$WIFI" ] && { ARGS="0x0 $BL 0x8000 $PT 0x9000 $WIFI 0x10000 $FW"; wifi_used=1; }
    python3 "$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 921600 \
        --before default_reset --after no_reset write_flash $FLASH $ARGS
    # clear bootflag (0xd000) + otadata (0xe000) -> hook boots the loader (factory)
    python3 "$ESPTOOL" --chip esp32s3 --port "$PORT" --before default_reset --after hard_reset \
        erase_region 0xd000 0x3000
    # The creds image is now on the device; don't leave the plaintext SSID/PSK on disk
    # (especially under a world-readable /tmp). Set KEEP_WIFI=1 to retain it.
    if [ "$wifi_used" = 1 ] && [ "${KEEP_WIFI:-0}" != 1 ]; then
        command -v shred >/dev/null 2>&1 && shred -u "$WIFI" 2>/dev/null || rm -f "$WIFI"
        echo "wiped creds image: $WIFI  (set KEEP_WIFI=1 to keep it)"
    fi
else
    echo "usage: $0 [app|full]"; exit 1
fi
echo "done ($mode)."
