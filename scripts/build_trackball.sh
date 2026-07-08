#!/usr/bin/env bash

cd "$(dirname "$0")/../zmk" || exit 1

EXTRA_MODULES=""

add_module() {
    local dir="$(pwd)/../$1"
    if [ -d "$dir" ]; then
        EXTRA_MODULES="${EXTRA_MODULES:+${EXTRA_MODULES};}${dir}"
    else
        echo "Warning: module directory not found: $dir" >&2
    fi
}

add_module "endgame-trackball-config"
add_module "zmk-pmw3610-driver"
add_module "zmk-paw3395-driver"
add_module "zmk-paw3950-driver"
add_module "zmk-pointer-2s-mixer"
add_module "zmk-axis-clamper"
add_module "zmk-report-rate-limit"
add_module "zmk-ec11-ish-driver"
add_module "zmk-auto-hold"
add_module "zmk-behavior-follower"
add_module "zmk-keymap-shell"
add_module "zmk-acceleration-curves"
add_module "zmk-rotate-plane"
add_module "zmk-runtime-config"
add_module "zmk-adaptive-feedback"
add_module "zmk-bistable-behavior"
add_module "zmk-ble-shell"
add_module "zmk-esb-endpoint"
add_module "zmk-feedback-common"
add_module "zmk-shell-relay-core"

echo "Building ZMK firmware for Endgame Trackball..."
echo "Extra modules: $EXTRA_MODULES"
echo ""

west build \
    --pristine always \
    -s app \
    -b efogtech_trackball_0 \
    -S studio-rpc-usb-uart \
    -S zmk-usb-logging \
    -- \
    -DZMK_EXTRA_MODULES="${EXTRA_MODULES}" \
    -DCONFIG_ZMK_STUDIO=y \
    -DZMK_CONFIG="$(pwd)/../endgame-trackball-config/config"