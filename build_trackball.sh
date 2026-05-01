#!/bin/sh
cd "$(dirname "$0")/zmk" || exit 1

west build --pristine always -s app -b efogtech_trackball_0 \
  -S studio-rpc-usb-uart -S zmk-usb-logging -- \
  -DZMK_EXTRA_MODULES="$(pwd)/../endgame-trackball-config;$(pwd)/../zmk-pmw3610-driver;$(pwd)/../zmk-paw3395-driver;$(pwd)/../zmk-pointer-2s-mixer;$(pwd)/../zmk-axis-clamper;$(pwd)/../zmk-report-rate-limit;$(pwd)/../zmk-ec11-ish-driver;$(pwd)/../zmk-auto-hold;$(pwd)/../zmk-behavior-follower;$(pwd)/../zmk-keymap-shell;$(pwd)/../zmk-acceleration-curves;$(pwd)/../zmk-rotate-plane;$(pwd)/../zmk-runtime-config;$(pwd)/../zmk-adaptive-feedback;$(pwd)/../zmk-bistable-behavior;$(pwd)/../zmk-ble-shell;$(pwd)/../zmk-esb-endpoint" \
  -DCONFIG_ZMK_STUDIO=y -DZMK_CONFIG="$(pwd)/../endgame-trackball-config/config"
