CC      := gcc
CFLAGS  := -Wall -Wextra -O2 -I../src
LDFLAGS :=
LDLIBS  :=

BUILD_DIR := build
SRC_DIR   := .

SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
BINS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%,$(SRCS))

.PHONY: all clean

all: $(BINS)

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/mcp23009_%: EXTRA_SRCS := ../src/lib_mcp23009.c
$(BUILD_DIR)/mcp23s08_%: EXTRA_SRCS := ../src/lib_mcp23s08.c
$(BUILD_DIR)/mcp23s09_%: EXTRA_SRCS := ../src/lib_mcp23s09.c

$(BUILD_DIR)/%: $(BUILD_DIR)/%.o
	$(CC) $< $(EXTRA_SRCS) -o $@ $(LDFLAGS) $(LDLIBS)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build directory"