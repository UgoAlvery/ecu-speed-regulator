# results/

This directory stores ecu_tester run reports produced on the target hardware.

## File naming

```
ecu_tester_<YYYYMMDD>_<HHmm>_<tag>.txt
```

Example: `ecu_tester_20261115_1430_initial.txt`

## Report template

Copy `template.txt` to a new file and fill in the values after each run.

## Expected columns (from ecu_tester.py phases)

| Phase | Key metrics |
|-------|-------------|
| Normal operation (30 s) | OUTPUT count, mean period, max gap |
| Stress test | rx_crc_error, rx_replay, rx_dropped after flood |
| Post-stress recovery | OUTPUT resumed within 100 ms, PID active |
| Failsafe (2.5 s silence) | Time to OUTPUT=0, ALARM received y/n |
| Signal de vie (whole session) | max_output_gap ≤ 300 ms (EM-01) |
