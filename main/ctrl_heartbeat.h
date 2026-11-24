#ifndef CTRL_HEARTBEAT_H
#define CTRL_HEARTBEAT_H

#include <stdint.h>
#include <stdbool.h>

/* Returns true when task_control heartbeat has not changed for longer than
 * timeout_ms. Extracted for host-test coverage without FreeRTOS. */
static inline bool ctrl_heartbeat_frozen(uint32_t current_hb, uint32_t prev_hb,
                                          uint32_t elapsed_ms, uint32_t timeout_ms)
{
    return (current_hb == prev_hb) && (elapsed_ms > timeout_ms);
}

#endif /* CTRL_HEARTBEAT_H */
