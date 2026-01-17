## Endgame Trackball (firmware)

### Prerequisites
1. Git
2. Protobuf
3. Zephyr SDK
4. West

### Building

#### Locally
1. `git submodule update --init`
2. `cd zmk/` 
3. `west init -l app`
4. `west update`

Now, staying in the `zmk` directory, build with west:
```shell
west build -s app -b efogtech_trackball_0 -S studio-rpc-usb-uart -S zmk-usb-logging -- -DZMK_EXTRA_MODULES="$(pwd)/../endgame-trackball-config;$(pwd)/../zmk-pmw3610-driver;$(pwd)/../zmk-pointer-2s-mixer;$(pwd)/../zmk-axis-clamper;$(pwd)/../zmk-report-rate-limit;$(pwd)/../zmk-ec11-ish-driver;$(pwd)/../zmk-auto-hold;$(pwd)/../zmk-trigger-on-boot;$(pwd)/../zmk-behavior-follower;$(pwd)/../zmk-keymap-shell;$(pwd)/../zmk-acceleration-curves" -DCONFIG_ZMK_STUDIO=y -DZMK_CONFIG="$(pwd)/../endgame-trackball-config/config"
```

Look for `build/zephyr/zmk.uf2` — it's your firmware.

#### GitHub Actions
Fork the `endgame-trackball-config` repository. Any commit will trigger an action that will build the firmware for you.

### Troubleshooting

1. Flash the [debug firmware](https://github.com/efogtech/endgame-trackball-config/tree/debug).
2. [Check out your logs](https://zmk.dev/docs/development/usb-logging#viewing-logs) via USB.

### Also see
1. Parent repo: https://github.com/efogtech/endgame-trackball
2. ZMK docs: https://zmk.dev/docs/
3. Zephyr 3.5 docs: https://docs.zephyrproject.org/3.5.0/
