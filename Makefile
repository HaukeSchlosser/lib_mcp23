CC       = gcc
CFLAGS   = -Wall -O2 -fPIC
LD       = $(CC)
LDFLAGS  = -shared

SRC_DIR   = src
BUILD_DIR = build

LIBS      = mcp23s08 mcp23s09 mcp23009

SRCS      = $(addprefix $(SRC_DIR)/lib_, $(addsuffix .c, $(LIBS)))
OBJS      = $(addprefix $(BUILD_DIR)/lib_, $(addsuffix .o, $(LIBS)))

TARGETS_US = $(addprefix $(BUILD_DIR)/lib_, $(addsuffix .so, $(LIBS)))
TARGETS_NO = $(addprefix $(BUILD_DIR)/lib,  $(addsuffix .so, $(LIBS)))
TARGETS    = $(TARGETS_US) $(TARGETS_NO)

HEADERS   = $(addprefix $(SRC_DIR)/lib_, $(addsuffix .h, $(LIBS)))

PREFIX     = /usr/local
LIBDIR     = $(PREFIX)/lib
INCLUDEDIR = $(PREFIX)/include

.PHONY: all install uninstall clean

all: $(TARGETS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/lib_%.o: $(SRC_DIR)/lib_%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/lib_%.so: $(BUILD_DIR)/lib_%.o
	$(LD) $(LDFLAGS) -o $@ $<

$(BUILD_DIR)/lib%.so: $(BUILD_DIR)/lib_%.o
	$(LD) $(LDFLAGS) -o $@ $<

install: $(TARGETS)
	@echo "Use 'sudo make install' if you need root permissions"
	mkdir -p "$(DESTDIR)$(LIBDIR)" "$(DESTDIR)$(INCLUDEDIR)"
	@for lib in $(LIBS); do \
		install -m 0755 "$(BUILD_DIR)/lib_$${lib}.so" "$(DESTDIR)$(LIBDIR)/lib_$${lib}.so"; \
		install -m 0755 "$(BUILD_DIR)/lib$${lib}.so"  "$(DESTDIR)$(LIBDIR)/lib$${lib}.so"; \
		install -m 0644 "$(SRC_DIR)/lib_$${lib}.h"    "$(DESTDIR)$(INCLUDEDIR)/lib_$${lib}.h"; \
		echo "Installed: $(DESTDIR)$(LIBDIR)/lib_$${lib}.so and lib$${lib}.so"; \
	done

uninstall:
	@echo "Use 'sudo make uninstall' if you need root permissions"
	@for lib in $(LIBS); do \
		rm -f "$(DESTDIR)$(LIBDIR)/lib_$${lib}.so"; \
		rm -f "$(DESTDIR)$(LIBDIR)/lib$${lib}.so"; \
		rm -f "$(DESTDIR)$(INCLUDEDIR)/lib_$${lib}.h"; \
		echo "Uninstalled: $(DESTDIR)$(LIBDIR)/lib_$${lib}.so and lib$${lib}.so"; \
	done

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build files"