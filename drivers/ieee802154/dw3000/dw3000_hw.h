#ifndef DW3000_HW_H
#define DW3000_HW_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Function pointer type for custom interrupt handlers
typedef void (*dw3000_irq_handler_t)(void);

int dw3000_hw_init(void);
int dw3000_hw_init_interrupt(void);
void dw3000_hw_fini(void);
void dw3000_hw_reset(void);
void dw3000_hw_wakeup(void);
void dw3000_hw_wakeup_pin_low(void);
void dw3000_hw_interrupt_enable(void);
void dw3000_hw_interrupt_disable(void);
bool dw3000_hw_interrupt_is_enabled(void);

// Runtime interrupt handler management
void dw3000_hw_set_interrupt_handler(dw3000_irq_handler_t handler);
void dw3000_hw_clear_interrupt_handler(void);

#ifdef __cplusplus
}
#endif

#endif
