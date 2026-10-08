#include "ecu_command.h"

#include <stddef.h>

void ecu_command_init(ecu_command_t *cmd)
{
    if (cmd == NULL) return;
    cmd->mode            = ECU_MODE_OFF;
    cmd->output          = 0.0f;
    cmd->failsafe_locked = false;
}

bool ecu_command_try_set_mode(ecu_command_t *cmd, const ecu_mode_t mode)
{
    if (cmd == NULL) return false;
    if (mode == ECU_MODE_AUTO && cmd->failsafe_locked) {
        return false;
    }
    cmd->mode = mode;
    return true;
}

static bool emit_output(const float output, const ecu_output_emit_fn emit, void *ctx)
{
    return (emit != NULL) && emit(output, ctx);
}

bool ecu_command_commit_output(ecu_command_t *cmd, const float requested,
                               const ecu_output_emit_fn emit, void *ctx)
{
    if (cmd == NULL) return false;
    cmd->output = (cmd->mode == ECU_MODE_AUTO) ? requested : 0.0f;
    return emit_output(cmd->output, emit, ctx);
}

bool ecu_command_enter_failsafe(ecu_command_t *cmd, const bool lock,
                                const ecu_output_emit_fn emit, void *ctx)
{
    if (cmd == NULL) return false;
    if (lock) {
        cmd->failsafe_locked = true;
    }
    cmd->output = 0.0f;
    cmd->mode   = ECU_MODE_OFF;
    return emit_output(0.0f, emit, ctx);
}

void ecu_command_set_failsafe_locked(ecu_command_t *cmd, const bool locked)
{
    if (cmd == NULL) return;
    cmd->failsafe_locked = locked;
}
