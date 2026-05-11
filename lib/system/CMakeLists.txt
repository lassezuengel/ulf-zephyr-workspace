# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(node.c hw.c block_heap.c)
zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_RADIO_ARBITER radio_arbiter.c)

# Conditionally include shell source files only when CONFIG_SHELL is enabled
# Disable these shell modules for timing-sensitive applications
# if(CONFIG_SHELL)
#     zephyr_library_sources(node_shell.c)
# endif()
