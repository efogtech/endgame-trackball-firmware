#!/bin/bash
set -euo pipefail

WORKSPACE=/workspace
ZMK_DIR="$WORKSPACE/zmk"

EXTRA_MODULES=""
add_module() {
    local dir="$WORKSPACE/$1"
    if [ -d "$dir" ]; then
        EXTRA_MODULES="${EXTRA_MODULES:+${EXTRA_MODULES};}${dir}"
    else
        echo "Warning: module directory not found: $dir" >&2
    fi
}

add_module "endgame-trackball-config"
add_module "zmk-pmw3610-driver"
add_module "zmk-paw3395-driver"
add_module "zmk-pointer-2s-mixer"
add_module "zmk-axis-clamper"
add_module "zmk-report-rate-limit"
add_module "zmk-ec11-ish-driver"
add_module "zmk-auto-hold"
add_module "zmk-adaptive-feedback"
add_module "zmk-behavior-follower"
add_module "zmk-keymap-shell"
add_module "zmk-acceleration-curves"
add_module "zmk-rotate-plane"
add_module "zmk-input-processor-rotate-plane"
add_module "zmk-runtime-config"
add_module "zmk-bistable-behavior"
add_module "zmk-ble-shell"

echo "Building ZMK firmware for Endgame Trackball..."
echo "Extra modules: $EXTRA_MODULES"
echo ""

cd "$ZMK_DIR"

echo "--------------"
echo $(pwd)
echo "--------------"
echo ""

cd /workspace/zmk

if [ ! -d ".west" ]; then
    echo "Initializing west workspace..."
    west init -l app && west update
fi

west zephyr-export

west build \
    --pristine always \
    -s app \
    -b efogtech_trackball_0 \
    -S studio-rpc-usb-uart \
    -S zmk-usb-logging \
    -- \
    -DZMK_EXTRA_MODULES="${EXTRA_MODULES}" \
    -DCONFIG_ZMK_STUDIO=y \
    -DZMK_CONFIG="${WORKSPACE}/endgame-trackball-config/config"

FIRMWARE="$ZMK_DIR/build/zephyr/zmk.uf2"

echo ""
echo "Build successful!"
echo "Firmware: $FIRMWARE"

if [ -d "/output" ]; then
    cp "$FIRMWARE" /output/zmk.uf2
    echo "Firmware copied to /output/zmk.uf2"
fi
