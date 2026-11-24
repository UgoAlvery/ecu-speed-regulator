CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O0 -g \
         -I../../main \
         -fsanitize=address,undefined
LDFLAGS = -fsanitize=address,undefined -lm

MAIN_DIR = ../../main

.PHONY: all run clean

all: test_protocol test_pid test_failsafe_line test_ecu_command \
     test_hal_pwm test_hal_encoder test_hal_current test_safety_paths

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
                   $(MAIN_DIR)/hal_pwm.h $(MAIN_DIR)/hal_config.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_safety_paths.c $(MAIN_DIR)/hal_pwm_mock.c $(LDFLAGS)

run: all
	@status=0; \
	./test_protocol      || status=1; \
	./test_pid           || status=1; \
	./test_failsafe_line || status=1; \
	./test_ecu_command   || status=1; \
	./test_hal_pwm       || status=1; \
	./test_hal_encoder   || status=1; \
	./test_hal_current   || status=1; \
	./test_safety_paths  || status=1; \
	exit $$status

clean:
	rm -f test_protocol test_pid test_failsafe_line test_ecu_command \
	      test_hal_pwm test_hal_encoder test_hal_current test_safety_paths
