/**
 * @file platform_zephyr.h
 * @brief Platform abstraction layer: FreeRTOS → Zephyr RTOS
 *
 * This header provides compatibility macros and types to port FreeRTOS-based
 * swarm ranging code to Zephyr RTOS.
 */

#ifndef PLATFORM_ZEPHYR_H
#define PLATFORM_ZEPHYR_H

#include <zephyr/kernel.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Time Conversion and Timing
 * ============================================================================ */

/** Time type (milliseconds as uint32_t) */
typedef uint32_t Time_t;

/** Convert milliseconds to ticks (1:1 ratio since SYS_CLOCK_TICKS_PER_SEC=1000) */
#define M2T(ms) ((Time_t)(ms))

/** Get current tick count (in ms) */
#define xTaskGetTickCount() k_uptime_get_32()

/** Delay for given ticks */
#define vTaskDelay(ticks) k_msleep(ticks)

/** Max delay value */
#define portMAX_DELAY K_FOREVER

/* ============================================================================
 * Thread/Task Abstraction
 * ============================================================================ */

/** Task handle type */
typedef struct k_thread* TaskHandle_t;

/** Task function type */
typedef void (*TaskFunction_t)(void *);

/** Helper macro to define thread stack */
#define SWARM_THREAD_STACK_DEFINE(name, size) \
    K_THREAD_STACK_DEFINE(name##_stack, size); \
    struct k_thread name##_data

/** Create a new thread/task */
static inline TaskHandle_t swarm_task_create(
    TaskFunction_t task_func,
    const char *name,
    size_t stack_size,
    void *parameters,
    int priority)
{
    /* Note: Stack must be defined externally using SWARM_THREAD_STACK_DEFINE */
    /* This is a simplified version - actual implementation needs stack reference */
    return NULL; /* Will be properly implemented in swarm_ranging_core.c */
}

/* ============================================================================
 * Message Queue Abstraction
 * ============================================================================ */

/** Queue handle type */
typedef struct k_msgq* QueueHandle_t;

/** Helper macro to define message queue */
#define SWARM_MSGQ_DEFINE(name, item_size, max_items) \
    K_MSGQ_DEFINE(name, item_size, max_items, 4)

/** Create queue - uses predefined msgq */
#define xQueueCreate(size, item_size) NULL /* Use SWARM_MSGQ_DEFINE instead */

/** Send to queue (blocking) */
static inline int xQueueSend(QueueHandle_t queue, const void *item, k_timeout_t timeout)
{
    if (queue == NULL) return -1;
    return k_msgq_put(queue, item, timeout) == 0 ? 1 : 0;
}

/** Send from ISR */
static inline int xQueueSendFromISR(QueueHandle_t queue, const void *item,
                                     int *pxHigherPriorityTaskWoken)
{
    if (queue == NULL) return -1;
    return k_msgq_put(queue, item, K_NO_WAIT) == 0 ? 1 : 0;
}

/** Receive from queue */
static inline int xQueueReceive(QueueHandle_t queue, void *item, k_timeout_t timeout)
{
    if (queue == NULL) return -1;
    return k_msgq_get(queue, item, timeout) == 0 ? 1 : 0;
}

/* ============================================================================
 * Mutex/Semaphore Abstraction
 * ============================================================================ */

/** Semaphore/Mutex handle type */
typedef struct k_mutex* SemaphoreHandle_t;

/** Create mutex */
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    /* Note: Mutex must be defined statically and initialized with k_mutex_init */
    return NULL; /* Placeholder - use static mutexes in practice */
}

/** Take semaphore/mutex */
static inline int xSemaphoreTake(SemaphoreHandle_t mu, k_timeout_t timeout)
{
    if (mu == NULL) return 0;
    return k_mutex_lock(mu, timeout) == 0 ? 1 : 0;
}

/** Give semaphore/mutex */
static inline int xSemaphoreGive(SemaphoreHandle_t mu)
{
    if (mu == NULL) return 0;
    return k_mutex_unlock(mu) == 0 ? 1 : 0;
}

/* ============================================================================
 * Timer Abstraction
 * ============================================================================ */

/** Timer handle type */
typedef struct k_timer* TimerHandle_t;

/** Timer callback type */
typedef void (*TimerCallbackFunction_t)(TimerHandle_t timer);

/** Create timer (simplified - use K_TIMER_DEFINE in practice) */
static inline TimerHandle_t xTimerCreate(
    const char *name,
    k_timeout_t period,
    bool auto_reload,
    void *timer_id,
    TimerCallbackFunction_t callback)
{
    /* Note: Timer must be defined statically using K_TIMER_DEFINE */
    return NULL; /* Placeholder */
}

/** Start timer */
static inline int xTimerStart(TimerHandle_t timer, k_timeout_t timeout)
{
    if (timer == NULL) return 0;
    k_timer_start(timer, timeout, timeout); /* Auto-reload */
    return 1;
}

/* ============================================================================
 * Assertions
 * ============================================================================ */

#define ASSERT(x) __ASSERT(x, "Assertion failed: " #x)

/* ============================================================================
 * Memory Attributes (STM32 specific - no-op on Zephyr)
 * ============================================================================ */

#define NO_DMA_CCM_SAFE_ZERO_INIT /* No equivalent in Zephyr */

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_ZEPHYR_H */
