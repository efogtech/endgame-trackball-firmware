cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dongle_esb_hid)

zephyr_compile_options(-Wno-address-of-packed-member)

# ESB + MPSL come from the zmk-esb-endpoint module's vendored tree.
set(ESB_MODULE_DIR ${CMAKE_SOURCE_DIR}/../zmk-esb-endpoint)
set(NRFXLIB_DIR    ${ESB_MODULE_DIR}/vendor/nrfxlib)
set(NRF_ESB_DIR    ${ESB_MODULE_DIR}/vendor/nrf-esb)

if(CONFIG_FPU AND CONFIG_FPU_SHARING)
    set(MPSL_FLOAT_DIR "hard-float")
elseif(CONFIG_FPU)
    set(MPSL_FLOAT_DIR "softfp-float")
else()
    set(MPSL_FLOAT_DIR "soft-float")
endif()

set(MPSL_LIB     ${NRFXLIB_DIR}/mpsl/lib/nrf52/${MPSL_FLOAT_DIR}/libmpsl.a)
set(MPSL_FEM_LIB ${NRFXLIB_DIR}/mpsl/fem/common/lib/nrf52/${MPSL_FLOAT_DIR}/libmpsl_fem_common.a)
if(NOT EXISTS ${MPSL_FEM_LIB})
    message(FATAL_ERROR "MPSL FEM library not found: ${MPSL_FEM_LIB}")
endif()
zephyr_link_libraries(${MPSL_FEM_LIB})
zephyr_link_libraries(${MPSL_LIB})

if(CONFIG_HAS_HW_NRF_PPI)
    set(NRF_ESB_PPI_SRC ${NRF_ESB_DIR}/subsys/esb_ppi.c)
else()
    set(NRF_ESB_PPI_SRC ${NRF_ESB_DIR}/subsys/esb_dppi.c)
endif()

target_sources(app PRIVATE
    ${NRF_ESB_DIR}/subsys/esb.c
    ${NRF_ESB_PPI_SRC}
    ${ESB_MODULE_DIR}/src/esb/channel_hop.c
    src/main.c
    src/esb_prx.c
    src/usb_hid.c
    src/led_status.c
    src/shell_relay.c
    src/bench.c
    src/channel_hop_dongle.c
)

target_include_directories(app PRIVATE
    ${NRF_ESB_DIR}/include
    ${NRF_ESB_DIR}/subsys
    ${ESB_MODULE_DIR}/include
)

zephyr_include_directories(${NRFXLIB_DIR}/mpsl/include)
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include)
zephyr_include_directories(${NRFXLIB_DIR}/mpsl/fem/include/protocol)
