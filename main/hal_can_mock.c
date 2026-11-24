#include "hal_can.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOCK_QUEUE_DEPTH  16u

/* Simple in-memory FIFO: sent frames are readable back (loopback). */
static hal_can_frame_t s_queue[MOCK_QUEUE_DEPTH];
static uint32_t s_head = 0;
static uint32_t s_tail = 0;
static bool     s_initialized = false;
static bool     s_bus_off     = false;

/* ── Test hooks ────────────────────────────────────────────────────────────*/

void hal_can_mock_set_bus_off(bool bus_off)
{
    s_bus_off = bus_off;
}

void hal_can_mock_reset_state(void)
{
    s_head        = 0;
    s_tail        = 0;
    s_initialized = false;
    s_bus_off     = false;
}

/* ── Interface ─────────────────────────────────────────────────────────────*/

bool hal_can_init(void)
{
    s_head        = 0;
    s_tail        = 0;
    s_bus_off     = false;
    s_initialized = true;
    return true;
}

bool hal_can_send(const hal_can_frame_t *frame)
{
    if (!s_initialized || s_bus_off || !frame) return false;
    const uint32_t next = (s_tail + 1u) % MOCK_QUEUE_DEPTH;
    if (next == s_head) return false;  /* queue full */
    s_queue[s_tail] = *frame;
    s_tail = next;
    return true;
}

bool hal_can_receive(hal_can_frame_t *frame, uint32_t timeout_ms)
{
    (void)timeout_ms;  /* mock: non-blocking regardless of timeout */
    if (!s_initialized || !frame) return false;
    if (s_head == s_tail) return false;  /* queue empty */
    *frame = s_queue[s_head];
    s_head = (s_head + 1u) % MOCK_QUEUE_DEPTH;
    return true;
}

bool hal_can_is_bus_off(void)
{
    return s_bus_off;
}
