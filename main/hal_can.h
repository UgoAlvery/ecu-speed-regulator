#ifndef HAL_CAN_H
#define HAL_CAN_H

/* CAN/TWAI HAL — 500 kbit/s, 11-bit IDs, classic CAN (8-byte payload max).
 *
 * Implementations:
 *   hal_can_esp32.c  — ESP32 TWAI driver
 *   hal_can_mock.c   — host mock (in-memory loopback queue)
 *
 * CAN frame IDs reuse MSG_* constants from protocol.h where the payload fits
 * in 8 bytes.  STATS (48 bytes) cannot fit in one CAN frame; it is not sent
 * over CAN in the current design.
 *
 * HAL_CAN_NO_ACK_MODE (hal_config.h): set true for single-node bench tests.
 * Must be false on the real two-node bus. */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;      /* 11-bit standard CAN identifier */
    uint8_t  data[8];
    uint8_t  dlc;     /* data length code: 0–8 */
} hal_can_frame_t;

bool hal_can_init(void);
/* Send a frame. Returns false on driver error or bus-off. */
bool hal_can_send(const hal_can_frame_t *frame);
/* Receive a frame. Returns false on timeout or if no frame is available.
 * timeout_ms == 0: non-blocking poll. */
bool hal_can_receive(hal_can_frame_t *frame, uint32_t timeout_ms);
/* True when the TWAI controller is in bus-off state (too many errors). */
bool hal_can_is_bus_off(void);

#endif /* HAL_CAN_H */
