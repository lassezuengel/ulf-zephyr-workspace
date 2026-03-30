# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR}/../include)
zephyr_library_sources_ifdef(CONFIG_IEEE802154_DSKIEL_DW1000
    timesync_glossy.c
    ranging_mtm.c
    ranging_mm.c
    uwb_driver_registry.c
    dw1000/ieee802154_dw1000.c
    dw1000/uwb_driver_dw1000.c
    uwb_utils/uwb_timestamp_utils.c
    uwb_utils/uwb_frame_utils.c
)

zephyr_library_sources_ifdef(CONFIG_DW1000_RANGE_BIAS_CORRECTION
    dw1000/dw1000_range_tables.c
)

# DW1000 SPI implementation selection
if(CONFIG_IEEE802154_DSKIEL_DW1000)
    # Choose between Zephyr and nRFx SPI implementation
    # Both implementations provide the same dw1000_spi.h interface

    # Option 1: Use nRFx SPI (better performance - simplified to use only standard SPI)
    zephyr_library_sources(dw1000/dw1000_nrfx_spi.c)

    # Option 2: Use Zephyr SPI (fallback for compatibility)
    # zephyr_library_sources(dw1000/dw1000_zephyr_spi.c)

    zephyr_library_include_directories(dw1000)
endif()

# Add uwb_utils include directory
zephyr_library_include_directories(uwb_utils)

# Add DW3000 driver subdirectory
if(CONFIG_IEEE802154_DW3000)
    zephyr_library_sources(
        dw3000/ieee802154_dw3000_minimal.c
        dw3000/uwb_driver_dw3000.c
        dw3000/deca_interface.c
        dw3000/deca_compat.c
        dw3000/deca_rsl.c
        dw3000/dw3000/dw3000_device.c
        dw3000/dw3000_hw.c
        dw3000/dw3000_spi.c
        dw3000/deca_port.c
        dw3000/qmath/src/qmath.c
        uwb_utils/uwb_timestamp_utils.c
        uwb_utils/uwb_frame_utils.c
        uwb_driver_registry.c
    )
    zephyr_library_include_directories(
        dw3000
        dw3000/dw3000
        dw3000/qmath/include
    )
endif()
