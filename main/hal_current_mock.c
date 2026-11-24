#include "hal_current.h"
#include "hal_config.h"

#include <stdbool.h>
#include <stdint.h>

static bool    s_initialized  = false;
static int32_t s_current_ma   = 0;
static bool    s_read_error   = false;

/* ── Test hooks ────────────────────────────────────────────────────────────*/

void hal_current_mock_set_current_ma(int32_t ma)
{
    s_current_ma = ma;
}

void hal_current_mock_set_read_error(bool error)
{
    s_read_error = error;
}

void hal_current_mock_reset_state(void)
{
    s_initialized = false;
    s_current_ma  = 0;
    s_read_error  = false;
}

/* ── Interface ─────────────────────────────────────────────────────────────*/

bool hal_current_init(void)
{
    s_initialized = true;
    return true;
}

int32_t hal_current_read_ma(void)
{
    if (!s_initialized) return -1;
    if (s_read_error)   return -1;
    return s_current_ma;
}

bool hal_current_is_stalled(void)
{
    if (HAL_CURRENT_STALL_THRESHOLD_MA == 0u) return false; /* shortcut: TODO(datasheet) */
    const int32_t ma = hal_current_read_ma();
    if (ma < 0) return false;  /* read error: fail safe, don't cry stall */
    return (uint32_t)ma > HAL_CURRENT_STALL_THRESHOLD_MA;
}
