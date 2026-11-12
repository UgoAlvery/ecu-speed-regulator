# Fault Table — ECU Speed Regulator

> **Convention**: "Detected by" = the code path that fires. "Expected reaction" = what the code
> does, derivable from reading the source. "Measured time" = TODO until the hardware campaign;
> fill from task_timing CSV output on UART0 (see `main/task_timing.h`).

---

## Fault classes

| ID | Fault | Detection mechanism | Expected reaction | Measured time |
|----|-------|---------------------|-------------------|---------------|
| F-01 | UART silence > 2 s | `task_failsafe / poll_uart_silence()`: `(now − last_rx_tick) ≥ timeout_ticks` checked every 100 ms polling period | Mode → OFF, output → 0, OUTPUT(0) queued immediately, ALARM(0x85) sent to front of queue_tx, LED GPIO25 on | TODO (target campaign) |
| F-02 | GPIO failsafe line opens (edge) | ISR `gpio_isr_handler` → semaphore → `task_failsafe` wakes, calls `failsafe_line_on_edge()` → TRIP | Same as F-01 with `failsafe_locked = true` (AUTO blocked until line re-closes) | TODO |
| F-03 | GPIO failsafe line open at boot | `task_failsafe` reads level before loop starts: `line_is_open()` → `failsafe_line_on_level(open=true)` | Same as F-02 (ALARM "GPIO line open at boot") | TODO |
| F-04 | GPIO line bounces after open | Subsequent edges/levels land on `latched = true` in `failsafe_line_on_edge/level()` → NONE returned | No additional ALARM; failsafe state unchanged (single ALARM per open event) | N/A (code path only) |
| F-05 | CRC error on received frame | `protocol_decode()` returns false → `s_count_crc_err++` in `task_rx`; frame discarded | Frame silently dropped, last valid values preserved, `rx_crc_error` counter incremented in STATS | N/A (host-tested) |
| F-06 | Nonce replay / duplicate | `protocol_nonce_check_and_update()` returns false → `s_count_replay++` | Frame silently dropped, `rx_replay` counter incremented in STATS | N/A (host-tested) |
| F-07 | Frame with unknown type | `dispatch_frame()` default case → `s_count_rx_invalid++` | Frame discarded, last valid values preserved | N/A |
| F-08 | SETPOINT out of range [0, 220] km/h | `protocol_parse_f32_bounded()` → PROTOCOL_VALUE_OUT_OF_RANGE → `s_count_rx_out_of_range++` | Frame discarded, last valid setpoint preserved | N/A (host-tested) |
| F-09 | SPEED out of range [0, 242] km/h | Same as F-08 | Same as F-08 | N/A (host-tested) |
| F-10 | NaN/Inf in SETPOINT or SPEED payload | `protocol_parse_f32()` → false (isfinite check) → PROTOCOL_VALUE_INVALID | Frame discarded, `rx_invalid` counter incremented | N/A (host-tested) |
| F-11 | PID output NaN/Inf | `pid_compute()`: `isnan(output)` guard → returns `out_min` (0.0) | OUTPUT = 0 emitted this cycle; PID state unchanged | N/A (host-tested) |
| F-12 | MODE_SET(AUTO) while failsafe locked | `ecu_state_try_set_mode()` → false if `failsafe_locked` | Mode unchanged, `mode_refused` counter incremented in STATS | N/A |
| F-13 | queue_tx full at OUTPUT emit | `ecu_state_commit_output()` → emit returns false → `s_count_tx_dropped++` | OUTPUT not emitted this cycle; retried next 100 ms cycle by task_control | N/A |
| F-14 | queue_tx full at ALARM emit | `xQueueSendToFront()` returns != pdTRUE → `s_count_alarm_dropped++` | ALARM lost; failsafe state already set (safe); `alarm_dropped` counter in STATS | N/A |
| F-15 | queue_tx full at OUTPUT(0) from failsafe | `ecu_state_enter_failsafe()` → emit returns false → `s_count_output_dropped++` | OUTPUT(0) not emitted now; task_control emits it on next cycle (≤ 100 ms); state already OFF | TODO (worst-case line time) |
| F-16 | Task blocks > 2 s (any task) | ESP-IDF Task Watchdog (TWDT) subscribed in each task; triggers reboot after 2 s | Reboot; ECU restarts in mode OFF, output 0; GPIO line level re-checked at boot | N/A (TWDT hardware) |

---

## Column definitions

| Column | Meaning |
|--------|---------|
| **Detection mechanism** | Code path / condition that identifies the fault. Source file and function given. |
| **Expected reaction** | Actions the firmware takes, derived from code reading. No measurement required. |
| **Measured time** | Latency measured on hardware with `task_timing` probes (see `main/task_timing.h`). Fill after the target campaign. `N/A` = no latency to measure (logic-only or already bounded by test). |

---

## Instrumented probes (task_timing.h)

| Probe | What it measures | Covers |
|-------|-----------------|--------|
| `timing_record_control_tick()` | Period deviation of the 100 ms control loop | Loop jitter, indirectly affects F-13 (queue full timing) |
| `timing_record_failsafe_trigger/done()` | µs from start to end of `ecu_state_enter_failsafe()` | F-01, F-02 SW latency |
| `timing_record_heartbeat_silence_us()` | Observed silence duration when UART timeout fires | F-01 detection time |

---

*Table last updated: 2026-11-12. Fill "Measured time" cells after running the hardware campaign.*
