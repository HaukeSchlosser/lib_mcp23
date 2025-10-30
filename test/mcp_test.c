/**
 * @file mcp_test.c
 * @brief Test program for MCP23 GPIO expander library
 *
 * This test application demonstrates the functionality of the MCP23 GPIO expander
 * library. It supports different chip variants (MCP23S08, MCP23S09, MCP23009) and
 * provides multiple test operations:
 *
 * Supported variants:
 * - MCP23S08: 8-bit SPI GPIO expander with configurable addresses
 * - MCP23S09: 8-bit SPI GPIO expander
 * - MCP23009: 8-bit I2C GPIO expander
 *
 * Test operations:
 * - write:     LED blinking demo (all pins and single pin)
 * - read:      Continuous GPIO port and pin reading
 * - interrupt: Interrupt handling demonstration using GPIO line
 * - led:       Simple LED on/off test
 * - error:     Prints error code
 *
 * Usage:
 *     ./mcp_test <variant> <operation>
 *     Example: ./mcp_test mcp23009 write
 *
 * Hardware setup:
 * - SPI variants (MCP23S08/S09): Connected to SPI bus 0, CS 0
 * - I2C variant (MCP23009): Connected to I2C bus 1, configurable address
 * - Interrupt testing: Uses GPIO25 for interrupt line
 *
 * Configuration:
 * - SPI speed: Set to MCP_SPI_SPEED_SLOW for reliable operation
 * - I2C speed: Set to MCP_I2C_SPEED_SLOW (100kHz)
 * - Pull-ups: Enabled for input testing
 * - GPIO direction: Configured based on test operation
 *
 * Error handling:
 * - Uses fail() helper for consistent error reporting
 * - Reports both library errors and system errno
 * - Includes file/function/line information for debugging
 *
 * Dependencies:
 * - Linux GPIO character device (/dev/gpiochipN)
 * - SPI device driver (spidev)
 * - I2C device driver (i2c-dev)
 * - Root permissions or proper udev rules for device access
 *
 * @note Interrupt testing requires access to GPIO character device
 * @note Some operations run in infinite loops (Ctrl+C to stop)
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

static int fail(const mcp_dev_t *dev, const char *ctx, int ret_as) {
    mcp_error_t e = mcp_last_error(&dev->base);
    fprintf(stderr,
            "[%s] %s (code=%d, errno=%d) @ %s:%d in %s\n",
            ctx,
            mcp_strerror(e.code), e.code, e.sys_errno,
            e.file ? e.file : "?", e.line, e.func ? e.func : "?");
    return ret_as;
}

static int mcp23_test_write(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    mcp_err_t rc = mcp_init(dev, cfg);
    if (rc < 0) return fail(dev, "init", 1);

    rc = mcp_write(dev, MCP_IODIR, 0xFF); // reset
    if (rc < 0) return fail(dev, "IODIR=0xFF", 1);

    rc = mcp_write(dev, MCP_IODIR, 0x00); // all outputs
    if (rc < 0) return fail(dev, "IODIR=0x00", 1);

    rc = mcp_write(dev, MCP_OLAT, 0xFF); // all high (LEDs off)
    if (rc < 0) return fail(dev, "OLAT=0xFF", 1);

    for (;;) {
        printf("[mcp23::mcp23_test_write] Blinking all pins\n");
        rc = mcp_write(dev, MCP_OLAT, 0x00);              // LEDs on
        if (rc < 0) return fail(dev, "OLAT=0x00", 1);
        usleep(500000);

        rc = mcp_write(dev, MCP_OLAT, 0xFF);              // LEDs off
        if (rc < 0) return fail(dev, "OLAT=0xFF (off)", 1);
        usleep(500000);
        usleep(500000);

        printf("[mcp23::mcp23_test_write] Blinking pin 0\n");
        rc = mcp_write_pin(dev, MCP_OLAT, 0, 0);          // pin0 low
        if (rc < 0) return fail(dev, "write_pin(0,0)", 1);
        usleep(500000);

        rc = mcp_write_pin(dev, MCP_OLAT, 0, 1);          // pin0 high
        if (rc < 0) return fail(dev, "write_pin(0,1)", 1);
        usleep(500000);
    }
}

static int mcp23_test_read(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    mcp_err_t rc = mcp_init(dev, cfg);
    if (rc < 0) return fail(dev, "init", 1);

    rc = mcp_write(dev, MCP_IODIR, 0xFF);
    if (rc < 0) return fail(dev, "IODIR=0xFF", 1);

    rc = mcp_write(dev, MCP_GPPU, 0xFF); // pull-ups on
    if (rc < 0) return fail(dev, "GPPU=0xFF", 1);

    for (;;) {
        int16_t port = mcp_read(dev, MCP_GPIO);
        if (port < 0) return fail(dev, "read GPIO", 1);
        printf("[mcp23::mcp23_test_write] GPIO: 0x%02X\n", (uint8_t)port);
        usleep(500000);

        int pin0 = mcp_read_pin(dev, MCP_GPIO, 0);
        if (pin0 < 0) return fail(dev, "read_pin(0)", 1);
        printf("[mcp23::mcp23_test_write] pin0: %d\n", pin0);
        usleep(500000);
    }

    return 0;
}

static int mcp23_test_interrupt(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    mcp_err_t rc = mcp_init(dev, cfg);
    if (rc < 0) return fail(dev, "init", 1);

    uint8_t bitmask = 0xFF;
    rc = mcp_interrupt(dev, MCP_INT_ENABLE, bitmask, MCP_CHANGE_ANY);
    if (rc < 0) return fail(dev, "interrupt cfg", 1);

    int16_t dummy = mcp_read(dev, MCP_INTCAP);
    if (dummy < 0) return fail(dev, "read INTCAP", 1);

    int fd_chip = open(CHIPDEV, O_RDONLY);
    if (fd_chip < 0) { perror("open gpiochip"); return 1; }

    struct gpioevent_request req;
    memset(&req, 0, sizeof(req));
    req.lineoffset   = INT_LINE;
    req.handleflags  = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags   = GPIOEVENT_REQUEST_FALLING_EDGE;
    strncpy(req.consumer_label, "mcp23-int", sizeof(req.consumer_label)-1);

    if (ioctl(fd_chip, GPIO_GET_LINEEVENT_IOCTL, &req) < 0) {
        perror("GPIO_GET_LINEEVENT_IOCTL");
        close(fd_chip);
        return 1;
    }
    close(fd_chip);

    for (;;) {
        struct gpioevent_data ev;
        ssize_t n = read(req.fd, &ev, sizeof(ev));
        if (n != (ssize_t)sizeof(ev)) {
            if (n < 0) perror("read event");
            else fprintf(stderr, "short read on event fd\n");
            break;
        }

        int16_t inf = mcp_read(dev, MCP_INTF);
        int16_t cap = mcp_read(dev, MCP_INTCAP);
        if (inf < 0) return fail(dev, "read INTF", 1);
        if (cap < 0) return fail(dev, "read INTCAP", 1);

        uint8_t f = (uint8_t)inf, c = (uint8_t)cap;
        if (!f) {
            printf("[int] no flags (INTF=%02X INTCAP=%02X)\n", f, c);
            continue;
        }
        while (f) {
            int pin = __builtin_ctz(f);
            printf("[int] pin=%d state=%d\n", pin, (c >> pin) & 1);
            f &= (f - 1);
        }
    }

    return 0;
}

int8_t mcp_test_led(mcp_dev_t *dev, const mcp_cfg_t *cfg) {

    mcp_err_t rc = mcp_init(dev, cfg);
    if (rc < 0) return fail(dev, "init", 1);

    rc = mcp_led(dev, MCP_LED_ENABLE);
    if (rc < 0) return fail(dev, "led enable", 1);

    sleep(5);

    rc = mcp_led(dev, MCP_LED_DISABLE);
    if (rc < 0) return fail(dev, "led disable", 1);

    return 0;
}

int8_t mcp_test_error(mcp_dev_t *dev, const mcp_cfg_t *cfg) {
    if (mcp_init(dev, cfg) < 0) {
        mcp_error_t err = mcp_last_error(&(dev->base));
        fprintf(stderr, "Error: %s (%d) in %s:%d\n",
                mcp_strerror(err.code), err.code,
                err.file, err.line);
        return 1;
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
        } else if (strcmp(op, "led") == 0) {
            return mcp_test_led(&dev, &cfg) == 0 ? 0 : 1;
        } else if (strcmp(op, "error") == 0) {
            return mcp_test_error(NULL, &cfg) == 0 ? 0 : 1;
        } else {
            fprintf(stderr, "[mcp23_test] ERROR: Unknown operation: %s\n", op);
            return 1;
    }

    return 0;
}