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