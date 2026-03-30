zephyr_library()

# Always compile - the source has no-op fallbacks when DT node is missing
zephyr_library_sources(timesync_debug_gpio.c)
