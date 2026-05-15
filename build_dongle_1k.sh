#!/bin/sh
cd "$(dirname "$0")/zmk" || exit 1

BOARD=efogtech_dongle_1k
for arg in "$@"; do
    case "$arg" in
        --holyiot) BOARD=holyiot_dongle_1k ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

west build --pristine always -s ../dongle-1k-firmware -b "$BOARD" -- \
    "-DZEPHYR_EXTRA_MODULES=$(pwd)/../endgame-trackball-config"
