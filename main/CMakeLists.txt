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
        "task_timing.c"
        "failsafe.c"
        "failsafe_line.c"
        "hal_encoder_esp32.c"
        "hal_pwm_esp32.c"
        INCLUDE_DIRS "."
        REQUIRES driver esp_timer
)