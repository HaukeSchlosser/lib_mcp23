/*
 * mcp23_test.c — Example/Test program for MCP23x0x I/O expanders (SPI/I²C)
 *
 * Supported variants:
 *   - mcp23s08  (SPI, 8-bit, optional HAEN addressing)
 *   - mcp23s09  (SPI, 8-bit, without address decoder)
 *   - mcp23009  (I²C, 8-bit)
 *
 * Available operations:
 *   - write      : Configures all pins as outputs and demonstrates writing:
 *                  * Blinks all pins (OLAT 0x00/0xFF)
 *                  * Then blinks pin 0 individually using mcp_write_pin()
 *   - read       : Configures all pins as inputs with pull-ups (IODIR=0xFF, GPPU=0xFF)
 *                  * Periodically reads the full port (GPIO) and pin 0
 *                  * Prints values to stdout
 *   - interrupt  : Configures pin-change interrupts (MCP_INT_ENABLE, MCP_CHANGE_ANY)
 *                  * Uses Linux GPIO events (gpiochip) on FALLING_EDGE
 *                  * Reads INTF/INTCAP to identify triggered pins and their states
 *
 * System / Driver requirements:
 *   - Linux with spidev and/or i2c-dev and GPIO UAPI (linux/gpio.h)
 *   - User permissions for /dev/spidevX.Y, /dev/i2c-X, and /dev/gpiochipN
 *   - External library/header: <mcp23/mcp23.h> (hardware abstraction layer)
 *
 * Important constants:
 *   - CHIPDEV  : Path to the GPIO chip device (default: "/dev/gpiochip0")
 *   - INT_LINE : GPIO line offset for MCP INT pin (default: 25)
 *
 * Usage:
 *   mcp23_test <variant> <operation>
 *   Examples:
 *     ./mcp23_test mcp23s08 write
 *     ./mcp23_test mcp23s09 read
 *     ./mcp23_test mcp23009 interrupt
 *
 * Default configurations per variant:
 *   - mcp23s08 : SPI mode 0, low speed, bus=0, cs=0, addr=0, HAEN disabled
 *   - mcp23s09 : SPI mode 0, low speed, bus=0, cs=0
 *   - mcp23009 : I²C bus=1, low speed, addr=0
 *   Adjust bus/CS/address values to match your hardware setup.
 *
 * Operation details:
 *   - write:
 *       mcp_init() → IODIR=0x00 (all outputs) → OLAT=0xFF (all high/LEDs off)
 *       Infinite loop toggling OLAT 0x00/0xFF, then toggling pin 0 individually.
 *   - read:
 *       mcp_init() → IODIR=0xFF (all inputs) → GPPU=0xFF (pull-ups enabled)
 *       Infinite loop reading GPIO register and pin 0; printing both values.
 *   - interrupt:
 *       mcp_init() → enable interrupts on all pins → dummy read of INTCAP
 *       Open GPIO event FD on INT_LINE (falling edge)
 *       On event: read INTF/INTCAP, identify triggered pins (using ctz loop) & states.
 *
 * Return values:
 *   0  success
 *   1  failure (init/IO error, invalid argument, etc.)
 *
 * Build example:
 *   gcc -O2 -Wall -o mcp23_test mcp23_test.c \
 *       -lmcp23
 *   (add -I/-L paths as required for mcp23 library and kernel headers)
 *
 * Notes:
 *   - Root access or proper udev rules may be required for SPI/I²C/GPIO access.
 *   - INT_LINE must match the actual interrupt pin wiring (consider voltage levels).
 *   - The "write" operation runs indefinitely; use only with safe loads.
 *   - Functions mcp_init/mcp_write/mcp_read/... must be provided by the mcp23 library.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <mcp23/mcp23.h>
#include <linux/spi/spidev.h>

// For Interrupt
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#define CHIPDEV   "/dev/gpiochip0"
#define INT_LINE  25

static int mcp23_test_write(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    if (mcp_init(dev, cfg) < 0) {
        printf("[mcp23::mcp23_test_write] ERROR Init failed\n");
        return -1;
    }

    if (mcp_write(dev, 0x00, 0xFF) < 0) {
        printf("[mcp23::mcp23_test_write] ERROR Write failed\n");
        return -1;
    }

    if (mcp_write(dev, MCP_IODIR, 0x00) < 0) { // IODIR: all output
        fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write IODIR failed\n");
        return -1;
    }

    if (mcp_write(dev, MCP_OLAT, 0xFF) < 0) {  // OLAT: all high (LEDs off)
        fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write OLAT failed\n");
        return -1;
    }

    for (;;) {
        printf("[mcp23::mcp23_test_write] Blinking all pins\n");
        if (mcp_write(dev, MCP_OLAT, 0x00) < 0) {  // OLAT: all low (LEDs on)
            fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write OLAT failed\n");
            return -1;
        }
        usleep(500000);
        printf("[mcp23::mcp23_test_write] Turning off all pins\n");
        if (mcp_write(dev, MCP_OLAT, 0xFF) < 0) {  // OLAT: all high (LEDs off)
            fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write OLAT failed\n");
            return -1;
        }
        usleep(500000);

        printf("[mcp23::mcp23_test_write] Blinking pin 0\n");
        if (mcp_write_pin(dev, MCP_OLAT, 0, 0) < 0) { // OLAT: pin 0 low
            fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write OLAT pin 0 failed\n");
            return -1;
        }
        usleep(500000);
        printf("[mcp23::mcp23_test_write] Turning off pin 0\n");
        if (mcp_write_pin(dev, MCP_OLAT, 0, 1) < 0) { // OLAT: pin 0 high
            fprintf(stderr, "[mcp23::mcp23_test_write] ERROR Write OLAT pin 0 failed\n");
            return -1;
        }
        usleep(500000);
    }
}

static int mcp23_test_read(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    if (mcp_init(dev, cfg) < 0) {
        printf("[mcp23::mcp23_test_write] ERROR Init failed\n");
        return -1;
    }

    if (mcp_write(dev, MCP_IODIR, 0xFF) < 0) {
        fprintf(stderr, "[mcp23::mcp23_test_read] ERROR Write IODIR failed\n");
        return 1;
    }

    if (mcp_write(dev, MCP_GPPU, 0xFF) < 0) {
        fprintf(stderr, "[mcp23::mcp23_test_read] ERROR Write GPPU failed\n");
        return 1;
    }

    for (;;) {

        int16_t input = mcp_read(dev, MCP_GPIO);
        if (input < 0) {
            fprintf(stderr, "[mcp23::mcp23_test_read] ERROR Read GPIO failed\n");
            return 1;
        }
        printf("[mcp23::mcp23_test_read] GPIO: 0x%02X\n", (uint8_t)input);
        usleep(500000);

        int8_t pin0 = mcp_read_pin(dev, MCP_GPIO, 0);
        if (pin0 < 0) {
            fprintf(stderr, "[mcp23::mcp23_test_read] ERROR Read_pin GPIO pin 0 failed\n");
            return 1;
        }
        printf("[mcp23::mcp23_test_read] GPIO pin 0: %d\n", pin0);
        usleep(500000);
    }

    return 0;
}

static int mcp23_test_interrupt(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    int fd = -1;
    fd = mcp_init(dev, cfg);
    if (fd < 0) {
        printf("[mcp23::mcp_test_interrupt] ERROR Init failed\n");
        return -1;
    }

    uint8_t bitmask = 0xFF;
    if (mcp_interrupt(dev, MCP_INT_ENABLE, bitmask, MCP_CHANGE_ANY) < 0) {
        printf("[mcp23::mcp_test_interrupt] ERROR Configure interrupt failed\n");
        return 1;
    }
    if (mcp_read(dev, MCP_INTCAP) < 0) {
        printf("[mcp23::mcp_test_interrupt] ERROR Read INTCAP failed\n");
        return 1;
    }

    int fd_chip = open(CHIPDEV, O_RDONLY);
    if (fd_chip < 0) return 1; 

    struct gpioevent_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffset   = INT_LINE;
    req.handleflags  = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags   = GPIOEVENT_REQUEST_FALLING_EDGE;
    strncpy(req.consumer_label, "mcp23s09-int", sizeof(req.consumer_label)-1);

    if (ioctl(fd_chip, GPIO_GET_LINEEVENT_IOCTL, &req) < 0) {
        close(fd_chip);
        close(fd);
        return 1;
    }
    close(fd_chip);

    while(1) {
        struct gpioevent_data ev;
        ssize_t n = read(req.fd, &ev, sizeof(ev));
        if (n != (ssize_t)sizeof(ev)) {
            if (n < 0) perror("read event");
            else fprintf(stderr, "short read on event fd\n");
            break;
        }

        int16_t inf = mcp_read(dev, MCP_INTF);
        int16_t cap = mcp_read(dev, MCP_INTCAP);
        if (inf >= 0 && cap >= 0) {
            uint8_t f = (uint8_t)inf, c = (uint8_t)cap;
            if (!f) {
                printf("[mcp23009_test_interrupt] No flags (INTF=%02X INTCAP=%02X)\n", f, c);
                continue;
            }
            while (f) {
                int pin = __builtin_ctz(f);
                printf("[mcp23009_test_interrupt]  pin=%d state=%d\n", pin, (c >> pin) & 1);
                f &= (f - 1);
            }
        }
    }

    return 0;
}

static int parse_variant(const char *s, int *out_variant) {
    if (strcmp(s, "mcp23s08") == 0) { *out_variant = MCP_VARIANT_23S08; return 0; }
    if (strcmp(s, "mcp23s09") == 0) { *out_variant = MCP_VARIANT_23S09; return 0; }
    if (strcmp(s, "mcp23009") == 0) { *out_variant = MCP_VARIANT_23009; return 0; }
    return -1;
}

int main(int argc, char *argv[]) {

    if (argc < 3) {
        fprintf(stderr, "[mcp23_test] Usage:    %s <variant> <operation>\n", argv[0]);
        printf("[mcp23_test] Usage:    ./mcp_test_read mcp23s08 write\n");
        return 1;
    }

    const char *variant = argv[1];
    const char *op = argv[2];

    int variant_parse = -1;
    if (parse_variant(variant, &variant_parse) != 0) {
        fprintf(stderr, "[mcp23_test] ERROR: Unknown variant: %s\n", variant);
        return 1;
    }

    mcp_cfg_t cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.variant = variant_parse;

    mcp_dev_t dev;
    memset(&dev, 0, sizeof dev);

    if (strcmp(variant, "mcp23s08") == 0) {
        uint8_t bus = 0;
        uint8_t cs = 0;
        uint8_t addr = 0;
        cfg = MCP_CFG_SPI_23S08(bus, cs, SPI_MODE_0, MCP_SPI_SPEED_SLOW, addr, MCP_HAEN_DISABLE);
    } else if (strcmp(variant, "mcp23s09") == 0) {
        uint8_t bus = 0;
        uint8_t cs = 0;
        cfg = MCP_CFG_SPI_23S09(bus, cs, SPI_MODE_0, MCP_SPI_SPEED_SLOW);
    } else if (strcmp(variant, "mcp23009") == 0) {
        uint8_t bus = 1;
        uint8_t addr = 0;
        cfg = MCP_CFG_I2C_23009(bus, MCP_I2C_SPEED_SLOW, addr);
    } else {
        fprintf(stderr, "[mcp23_test] ERROR: Unknown operation: %s\n", op);
        return 1;
    }

    if (strcmp(op, "write") == 0) {
            return mcp23_test_write(&dev, &cfg) == 0 ? 0 : 1;
        } else if (strcmp(op, "read") == 0) {
            return mcp23_test_read(&dev, &cfg) == 0 ? 0 : 1;
        } else if (strcmp(op, "interrupt") == 0) {
            return mcp23_test_interrupt(&dev, &cfg) == 0 ? 0 : 1;
        } else {
            fprintf(stderr, "[mcp23_test] ERROR: Unknown operation: %s\n", op);
            return 1;
    }

    return 0;
}