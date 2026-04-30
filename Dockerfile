# ZMK firmware build image for Endgame Trackball
#
# Prerequisites (on the host before building this image):
#   git submodule update --init --recursive
#
# Build the image:
#   docker build -t endgame-firmware .
#
# Run the build and extract firmware:
# mkdir -p output
# docker run --rm -v "$(pwd):/workspace" -v "$(pwd)/output:/output" endgame-firmware

# Firmware will be at output/zmk.uf2

FROM zmkfirmware/zmk-build-arm:stable

WORKDIR /workspace

COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

VOLUME ["/output"]

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
