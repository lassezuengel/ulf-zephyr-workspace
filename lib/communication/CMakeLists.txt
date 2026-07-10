# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MAC_QUEUE)
  zephyr_sources(mac_queue.c)

  if(CONFIG_MAC_QUEUE_ENABLE_SHELL)
    zephyr_sources(mac_queue_shell.c)
  endif()
endif()