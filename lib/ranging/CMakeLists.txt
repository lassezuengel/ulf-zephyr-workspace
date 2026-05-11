# Copyright (c) 2021, Legrand North America, LLC.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
    twr.c
    twr_mm.c
    digest.c
    phase_recovery.c
    phase_utils.c
    ambiguity_resolution.c
    time_series_store.c
    time_series_filters.c
    radial_velocity.c
)

# Savitzky-Golay filter - only include when enabled (saves ~13KB RAM)
if(CONFIG_MM_RANGING_USE_SAVITZKY_GOLAY)
  zephyr_library_include_directories(
      ../../external/savitzky-golay/include
      ../../external/savitzky-golay/include/iterative
  )
  zephyr_library_sources(
    ../../external/savitzky-golay/src/iterative/savgolFilter.c
  )
endif()
