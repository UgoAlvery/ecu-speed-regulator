CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O0 -g \
         -I../../main \
         -fsanitize=address,undefined
LDFLAGS = -fsanitize=address,undefined -lm

MAIN_DIR = ../../main

.PHONY: all run clean

all: test_protocol test_pid test_failsafe_line test_ecu_command \
     test_hal_pwm test_hal_encoder test_hal_current test_safety_paths \
     test_hal_encoder_realistic test_hal_encoder_overflow

test_protocol: test_protocol.c $(MAIN_DIR)/protocol.c $(MAIN_DIR)/protocol.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_protocol.c $(MAIN_DIR)/protocol.c $(LDFLAGS)

test_pid: test_pid.c $(MAIN_DIR)/pid.c $(MAIN_DIR)/pid.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_pid.c $(MAIN_DIR)/pid.c $(LDFLAGS)

test_failsafe_line: test_failsafe_line.c $(MAIN_DIR)/failsafe_line.c $(MAIN_DIR)/failsafe_line.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_failsafe_line.c $(MAIN_DIR)/failsafe_line.c $(LDFLAGS)

test_ecu_command: test_ecu_command.c $(MAIN_DIR)/ecu_command.c $(MAIN_DIR)/ecu_command.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_ecu_command.c $(MAIN_DIR)/ecu_command.c $(LDFLAGS)

test_hal_pwm: test_hal_pwm.c $(MAIN_DIR)/hal_pwm_mock.c \
              $(MAIN_DIR)/hal_pwm.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_hal_pwm.c $(MAIN_DIR)/hal_pwm_mock.c $(LDFLAGS)

test_hal_encoder: test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c \
                  $(MAIN_DIR)/hal_encoder.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c $(LDFLAGS)

test_hal_current: test_hal_current.c $(MAIN_DIR)/hal_current_mock.c \
                  $(MAIN_DIR)/hal_current.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_hal_current.c $(MAIN_DIR)/hal_current_mock.c $(LDFLAGS)

test_safety_paths: test_safety_paths.c $(MAIN_DIR)/hal_pwm_mock.c \
                   $(MAIN_DIR)/hal_pwm.h $(MAIN_DIR)/hal_config.h \
                   $(MAIN_DIR)/ctrl_heartbeat.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_safety_paths.c $(MAIN_DIR)/hal_pwm_mock.c $(LDFLAGS)

# FIT0522 realistic values: 64 CPR, 120:1 gear, 100 mm wheel (π×0.1 m circ)
# TODO(datasheet): replace with measured values once available.
ENC_REALISTIC = -DHAL_ENC_TEST_REALISTIC=1 \
                -DHAL_ENC_PULSES_PER_MOTOR_REV=64u \
                -DHAL_ENC_GEAR_RATIO=120u \
                -DHAL_ENC_WHEEL_CIRC_M=0.314f

# Overflow scenario: CPR×gear (65536×65537=4 295 032 832) > UINT32_MAX.
# Without uint64_t cast the product wraps to 65536, giving ~1000× wrong speed.
# HAL_ENC_SKIP_STATIC_ASSERT suppresses the _Static_assert that would correctly
# reject these extreme values in production but blocks the host overflow test.
ENC_OVERFLOW  = -DHAL_ENC_TEST_OVERFLOW=1 \
                -DHAL_ENC_PULSES_PER_MOTOR_REV=65536u \
                -DHAL_ENC_GEAR_RATIO=65537u \
                -DHAL_ENC_WHEEL_CIRC_M=0.314f \
                -DHAL_ENC_SKIP_STATIC_ASSERT=1

test_hal_encoder_realistic: test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c \
                             $(MAIN_DIR)/hal_encoder.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) $(ENC_REALISTIC) -o $@ test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c $(LDFLAGS)

test_hal_encoder_overflow: test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c \
                           $(MAIN_DIR)/hal_encoder.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) $(ENC_OVERFLOW) -o $@ test_hal_encoder.c $(MAIN_DIR)/hal_encoder_mock.c $(LDFLAGS)

run: all
	@status=0; \
	./test_protocol              || status=1; \
	./test_pid                   || status=1; \
	./test_failsafe_line         || status=1; \
	./test_ecu_command           || status=1; \
	./test_hal_pwm               || status=1; \
	./test_hal_encoder           || status=1; \
	./test_hal_current           || status=1; \
	./test_safety_paths          || status=1; \
	./test_hal_encoder_realistic || status=1; \
	./test_hal_encoder_overflow  || status=1; \
	exit $$status

clean:
	rm -f test_protocol test_pid test_failsafe_line test_ecu_command \
	      test_hal_pwm test_hal_encoder test_hal_current test_safety_paths \
	      test_hal_encoder_realistic test_hal_encoder_overflow
