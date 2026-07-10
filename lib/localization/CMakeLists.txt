# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(least_squares.c)
zephyr_library_sources(position_payload.c)

if(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
    zephyr_library_sources(particle_filter.c)
endif()
