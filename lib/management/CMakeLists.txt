# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(network.c network_settings.c superframe_settings.c)

# Conditionally include shell source files only when CONFIG_SHELL is enabled
# Disable these shell modules for timing-sensitive applications
# if(CONFIG_SHELL)
#     zephyr_library_sources(network_shell.c)
# endif()

