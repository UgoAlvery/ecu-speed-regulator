# Optional feature flags. Pass -D<FLAG>=ON to idf.py build.
# Example: idf.py -DECU_DRIVE_HW_PWM=ON -DHW_PROTECTION_CONFIRMED=ON build
option(ECU_DRIVE_HW_PWM        "Drive motor via LEDC PWM"             OFF)
option(ECU_USE_ENCODER_SPEED   "Read speed from PCNT encoder"         OFF)
option(HW_PROTECTION_CONFIRMED "Confirm hardware protection installed" OFF)

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

# Translate CMake options to preprocessor defines for the main component.
set(ECU_COMPILE_DEFS "")
if(ECU_DRIVE_HW_PWM)
    list(APPEND ECU_COMPILE_DEFS "ECU_DRIVE_HW_PWM=1")
endif()
if(ECU_USE_ENCODER_SPEED)
    list(APPEND ECU_COMPILE_DEFS "ECU_USE_ENCODER_SPEED=1")
endif()
if(HW_PROTECTION_CONFIRMED)
    list(APPEND ECU_COMPILE_DEFS "HW_PROTECTION_CONFIRMED=1")
endif()
if(ECU_COMPILE_DEFS)
    target_compile_definitions(__idf_main PRIVATE ${ECU_COMPILE_DEFS})
endif()