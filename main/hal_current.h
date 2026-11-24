#ifndef HAL_CURRENT_H
#define HAL_CURRENT_H

/* Current sensor HAL (INA219 at I2C address 0x40).
 *
 * Implementations:
 *   hal_current_esp32.c  — I2C + INA219 register access
 *   hal_current_mock.c   — host mock for unit tests
 *
 * hal_current_is_stalled() returns false when HAL_CURRENT_STALL_THRESHOLD_MA
 * is 0 (see hal_config.h TODO(datasheet)). */

#include <stdbool.h>
#include <stdint.h>

bool    hal_current_init(void);
/* Return motor current in milliamps. Returns -1 on read error. */
int32_t hal_current_read_ma(void);
/* True when current > HAL_CURRENT_STALL_THRESHOLD_MA (and threshold != 0). */
bool    hal_current_is_stalled(void);

#endif /* HAL_CURRENT_H */
