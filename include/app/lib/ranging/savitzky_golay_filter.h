#ifndef SAVITZKY_GOLAY_FILTER_H
#define SAVITZKY_GOLAY_FILTER_H

#include <app/lib/ranging/measurement_filter.h>

/**
 * @file savitzky_golay_filter.h
 * @brief Savitzky-Golay filter implementation for phase smoothing
 * 
 * Implements a Savitzky-Golay filter that matches the Python scipy.signal.savgol_filter
 * behavior, including proper phase unwrapping/wrapping to handle 2π discontinuities.
 * 
 * The filter uses the external Savitzky-Golay library as a submodule and adapts it
 * to our generic filter interface. Default configuration matches the Python reference:
 * - Window length: 10 (configurable via Kconfig)
 * - Polynomial order: 3 (configurable via Kconfig)
 * - Derivative order: 0 (smoothing only)
 */

/**
 * @brief Savitzky-Golay filter operations
 * 
 * Provides the filter_ops_t interface for the Savitzky-Golay filter.
 * This allows the filter to be used interchangeably with other filters
 * in the generic measurement filter system.
 */
extern const filter_ops_t savitzky_golay_filter_ops;

// Configuration defaults (can be overridden in Kconfig)
#ifndef CONFIG_SAVITZKY_GOLAY_WINDOW_SIZE
#define CONFIG_SAVITZKY_GOLAY_WINDOW_SIZE 10
#endif

#ifndef CONFIG_SAVITZKY_GOLAY_POLY_ORDER  
#define CONFIG_SAVITZKY_GOLAY_POLY_ORDER 3
#endif

#ifndef CONFIG_SAVITZKY_GOLAY_FILTER_LOG_LEVEL
#define CONFIG_SAVITZKY_GOLAY_FILTER_LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#endif

#endif // SAVITZKY_GOLAY_FILTER_H