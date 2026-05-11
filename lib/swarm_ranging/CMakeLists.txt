# Copyright (c) 2025 SynchroFly
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
    swarm_ranging_core.c
    swarm_uwb_interface.c
    ranging_table.c
    neighbor_management.c
    velocity_mock.c
)
