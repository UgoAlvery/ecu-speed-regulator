# Fuzzing hôte de protocol_decode() — libFuzzer (clang), cf. technical_reference.md
# et roadmap_projet_ecu.md, axe 3 item 4.
#
# Cibles :
#   make fuzz  [SECONDS=60]  — build + génère le corpus si absent + fuzz N secondes
#   make seeds                — (re)génère le corpus de départ dans corpus/
#   make repro CRASH=<file>   — rejoue un cas de la trouvaille findings/ sous ASan/UBSan
#   make clean                — supprime binaires, corpus et findings générés

CC = clang

CFLAGS_FUZZ = -std=c11 -Wall -Wextra -Wpedantic -g -O1 \
              -I../../main \
              -fsanitize=fuzzer,address,undefined
CFLAGS_GEN  = -std=c11 -Wall -Wextra -Wpedantic -g -O1 \
              -I../../main

MAIN_DIR = ../../main
SECONDS ?= 60

.PHONY: all fuzz seeds repro clean

all: fuzz_protocol_decode

fuzz_protocol_decode: fuzz_protocol_decode.c $(MAIN_DIR)/protocol.c $(MAIN_DIR)/protocol.h
	$(CC) $(CFLAGS_FUZZ) -o $@ fuzz_protocol_decode.c $(MAIN_DIR)/protocol.c

gen_seeds: gen_seeds.c $(MAIN_DIR)/protocol.c $(MAIN_DIR)/protocol.h
	$(CC) $(CFLAGS_GEN) -o $@ gen_seeds.c $(MAIN_DIR)/protocol.c

seeds: gen_seeds
	mkdir -p corpus
	./gen_seeds

fuzz: fuzz_protocol_decode seeds
	mkdir -p findings
	./fuzz_protocol_decode -max_total_time=$(SECONDS) -artifact_prefix=findings/ corpus/

repro: fuzz_protocol_decode
	@test -n "$(CRASH)" || (echo "Usage: make repro CRASH=findings/crash-xxxx" && exit 1)
	./fuzz_protocol_decode $(CRASH)

clean:
	rm -rf fuzz_protocol_decode gen_seeds corpus findings *.o
