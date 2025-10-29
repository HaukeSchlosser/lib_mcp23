CC = gcc
CFLAGS = -Wall -O2
LDFLAGS = -lmcp23

SRC_DIR = .
BUILD_DIR = build
TARGET = $(BUILD_DIR)/mcp_test

INCLUDES = -I../includes

.PHONY: all clean

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): mcp_test.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned test files"