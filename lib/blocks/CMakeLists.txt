# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(block_mtm.c block_mm.c block_glossy.c block_time_sync_check.c block_announcement.c block_idle.c)

if(CONFIG_SYNCHROFLY_BLOCK_LS_POSITION)
    zephyr_library_sources(block_ls_position.c)
endif()

if(CONFIG_SYNCHROFLY_BLOCK_PF_POSITION)
    zephyr_library_sources(block_pf_position.c)
endif()

if(CONFIG_SYNCHROFLY_BLOCK_SWARM_RANGING)
    zephyr_library_sources(block_swarm_ranging.c)
endif()

if(CONFIG_SYNCHROFLY_BLOCK_MULOC)
    zephyr_library_sources(block_muloc.c)
endif()

if(CONFIG_SYNCHROFLY_BLOCK_CIR_READ)
    zephyr_library_sources(block_cir_read.c)
endif()

if(CONFIG_SYNCHROFLY_BLOCK_BUTLER)
    zephyr_library_sources(block_butler.c)
endif()

# Conditionally include shell source files only when CONFIG_SHELL is enabled
# Disable these shell modules for timing-sensitive applications
# if(CONFIG_SHELL)
#     zephyr_library_sources(mtm_shell.c glossy_shell.c)
# endif()
