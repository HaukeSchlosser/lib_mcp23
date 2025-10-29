CC       = gcc
CFLAGS   = -Wall -O2 -fPIC
LDFLAGS  = -shared -Wl,--no-undefined
INCLUDES = -Iincludes

SRC_DIR   = src
BUILD_DIR = build
TARGET    = $(BUILD_DIR)/libmcp23.so

SRCS = \
  $(SRC_DIR)/mcp23.c \
  $(SRC_DIR)/mcp23s08.c \
  $(SRC_DIR)/mcp23s09.c \
  $(SRC_DIR)/mcp23009.c \
  $(SRC_DIR)/mcp_spi.c

INSTALL_LIB_DIR = /usr/local/lib
INSTALL_INC_DIR = /usr/local/include/mcp23

.PHONY: all clean install uninstall

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(LDFLAGS) -o $@ $(SRCS)

install: $(TARGET)
	sudo mkdir -p $(INSTALL_LIB_DIR) $(INSTALL_INC_DIR)
	sudo cp $(TARGET) $(INSTALL_LIB_DIR)
	sudo cp includes/*.h $(INSTALL_INC_DIR)
	sudo ldconfig
	@echo "Library installed to $(INSTALL_LIB_DIR)"
	@echo "Headers  installed to $(INSTALL_INC_DIR)"

uninstall:
	sudo rm -f $(INSTALL_LIB_DIR)/$(notdir $(TARGET))
	sudo rm -rf $(INSTALL_INC_DIR)
	sudo ldconfig
	@echo "Library uninstalled"

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build files"