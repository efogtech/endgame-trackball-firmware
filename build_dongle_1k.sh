#!/bin/sh
cd "$(dirname "$0")/zmk" || exit 1

west build --pristine always -s ../dongle-1k-firmware -b efogtech_dongle_1k -- \
    "-DZEPHYR_EXTRA_MODULES=$(pwd)/../endgame-trackball-config"
