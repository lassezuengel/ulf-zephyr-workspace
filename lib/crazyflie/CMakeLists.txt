# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Only compile spi_connector.c if CONFIG_CRAZYFLIE is enabled
# The Kconfig already depends on $(dt_compat_enabled,synchrofly-hci-slave)
if(CONFIG_CRAZYFLIE)
    zephyr_library_sources(spi_connector.c)
endif()
