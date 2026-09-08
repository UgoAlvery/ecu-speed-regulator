CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -O0 -g \
         -I../../main \
         -fsanitize=address,undefined
LDFLAGS = -fsanitize=address,undefined

MAIN_DIR = ../../main

.PHONY: all run clean

all: test_protocol test_pid

test_protocol: test_protocol.c $(MAIN_DIR)/protocol.c $(MAIN_DIR)/protocol.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_protocol.c $(MAIN_DIR)/protocol.c $(LDFLAGS)

test_pid: test_pid.c $(MAIN_DIR)/pid.c $(MAIN_DIR)/pid.h test_runner.h
	$(CC) $(CFLAGS) -o $@ test_pid.c $(MAIN_DIR)/pid.c $(LDFLAGS)

run: all
	@status=0; \
	./test_protocol || status=1; \
	./test_pid      || status=1; \
	exit $$status

clean:
	rm -f test_protocol test_pid