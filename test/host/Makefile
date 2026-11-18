# Host build of the portable parts of the firmware: the protocol framing and the
# controller core, tested against the simulator's behaviour spec with plain gcc.
#
#   make -C test/host            build and run every test
#   make -C test/host core_sim   the core behind a pty for scorbot_protocol_cli or the operator (ACL)
#   make -C test/host e2e OPERATOR=/tmp/opbuild/bin/erv   the real operator against core_sim
#
# No ESP-IDF needed; nothing here touches hardware.
CC ?= gcc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Werror -Wno-unused-parameter
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
PROTO := $(ROOT)/components/scorbot_proto
CORE := $(ROOT)/components/scorbot_core
INC := -I$(PROTO)/include -I$(PROTO)/third_party/nanopb -I$(PROTO)/generated -I$(CORE)/include -I.
DEFS := -DPB_ENABLE_MALLOC=0 -DPB_FIELD_32BIT=0
BUILD ?= build

PROTO_SRC := $(PROTO)/src/cobs.c $(PROTO)/src/framing.c $(PROTO)/src/envelope.c \
             $(PROTO)/third_party/nanopb/pb_common.c $(PROTO)/third_party/nanopb/pb_decode.c \
             $(PROTO)/third_party/nanopb/pb_encode.c $(PROTO)/generated/scorbot/v1/scorbot.pb.c
CORE_SRC := $(wildcard $(CORE)/src/*.c)

all: test

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_proto: test_proto.c minitest.h $(PROTO_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(DEFS) -o $@ test_proto.c $(PROTO_SRC) -lm

$(BUILD)/test_core: test_core.c plant.c plant.h minitest.h $(CORE_SRC) $(PROTO_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(DEFS) -o $@ test_core.c plant.c $(CORE_SRC) $(PROTO_SRC) -lm

$(BUILD)/test_acl: test_acl.c plant.c plant.h minitest.h $(CORE_SRC) $(PROTO_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(DEFS) -o $@ test_acl.c plant.c $(CORE_SRC) $(PROTO_SRC) -lm

$(BUILD)/test_pixels: test_pixels.c minitest.h $(CORE_SRC) $(PROTO_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(DEFS) -o $@ test_pixels.c $(CORE_SRC) $(PROTO_SRC) -lm

$(BUILD)/core_sim: core_sim.c plant.c plant.h $(CORE_SRC) $(PROTO_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(DEFS) -o $@ core_sim.c plant.c $(CORE_SRC) $(PROTO_SRC) -lm -lutil

test: $(BUILD)/test_proto $(BUILD)/test_core $(BUILD)/test_acl $(BUILD)/test_pixels
	./$(BUILD)/test_proto
	./$(BUILD)/test_core
	./$(BUILD)/test_acl
	./$(BUILD)/test_pixels

test_pixels: $(BUILD)/test_pixels
	./$(BUILD)/test_pixels

test_acl: $(BUILD)/test_acl
	./$(BUILD)/test_acl

# End to end: the real operator (built out of tree, see e2e_operator.py) drives the
# core's Eshed-dialect front-end behind core_sim's pty while the script plays commander.
OPERATOR ?= /tmp/opbuild/bin/erv
$(BUILD)/ptyshim.so: ptyshim.c | $(BUILD)
	$(CC) -shared -fPIC -O1 -o $@ ptyshim.c -ldl

e2e: $(BUILD)/core_sim $(BUILD)/ptyshim.so
	python3 e2e_operator.py --operator $(OPERATOR)

test_proto: $(BUILD)/test_proto
	./$(BUILD)/test_proto

core_sim: $(BUILD)/core_sim

clean:
	rm -rf $(BUILD)

.PHONY: all test test_proto test_acl test_pixels core_sim e2e clean
