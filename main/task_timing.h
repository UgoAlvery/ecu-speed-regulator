#ifndef ECU_TASK_TIMING_H
#define ECU_TASK_TIMING_H

/* Software timing probes: loop jitter, failsafe SW latency, heartbeat timeout.
 * All functions are task-context only (no ISR). Call timing_print_csv() from
 * task_telemetry every TIMING_CSV_PERIOD_S seconds to dump results on UART0. */

#define TIMING_CSV_PERIOD_S  10u

/* Call at the very top of the task_control main loop, every 100 ms cycle. */
void timing_record_control_tick(void);

/* Bracket the ecu_state_enter_failsafe() call in trigger_failsafe(). */
void timing_record_failsafe_trigger(void);
void timing_record_failsafe_done(void);

/* Call in poll_uart_silence() when the 2 s timeout fires, with the observed
 * silence duration in microseconds (convert from ticks before calling). */
void timing_record_heartbeat_silence_us(long long silence_us);

/* Dump one CSV block on UART0 via ESP_LOGI. Safe to call from any task. */
void timing_print_csv(void);

#endif /* ECU_TASK_TIMING_H */
