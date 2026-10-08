idf_component_register(
        SRCS
        "main.c"
        "ecu_state.c"
        "ecu_command.c"
        "protocol.c"
        "pid.c"
        "task_rx.c"
        "task_tx.c"
        "task_control.c"
        "task_telemetry.c"
        "failsafe.c"
        "failsafe_line.c"
        INCLUDE_DIRS "."
        REQUIRES driver
)