# Bluetooth LE Peripheral Library

zephyr_library()

zephyr_library_sources(
  bt_peripheral.c
  bt_services.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_NODE_CONFIG_SERVICE
  services/node_config_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_NEIGHBOR_TABLE_SERVICE
  services/neighbor_table_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_NETWORK_SETTINGS_SERVICE
  services/network_settings_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_UWB_RADIO_SERVICE
  services/uwb_radio_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_SUPERFRAME_CONFIG_SERVICE
  services/superframe_config_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_IDENTIFICATION_SERVICE
  services/identification_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_CRTP_SERVICE
  services/crtp_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_AB_TESTING_SERVICE
  services/ab_testing_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_SYSTEM_PROFILING_SERVICE
  services/system_profiling_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_SYSTEM_CONTROL_SERVICE
  services/system_control_service.c
)

zephyr_library_sources_ifdef(CONFIG_SYNCHROFLY_BT_FIRMWARE_STATS_SERVICE
  services/firmware_stats_service.c
)

zephyr_library_include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)
