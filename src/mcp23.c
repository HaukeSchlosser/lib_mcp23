#include "../includes/mcp23.h"
#include "../includes/mcp23s08.h"
#include "../includes/mcp23s09.h"
#include "../includes/mcp23009.h"

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <string.h>

int mcp_init(mcp_dev_t *dev, const void *cfg_i) {

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_init] ERROR Invalid device handle\n");
        return -1;
    }

    if (!cfg_i) {
        fprintf(stderr, "[mcp23::mcp_init] ERROR Invalid config handle\n");
        return -1;
    }

    const mcp_cfg_t *cfg = (const mcp_cfg_t*)cfg_i;
    dev->variant = cfg->variant;

    switch (cfg->variant) {
        case MCP_VARIANT_23S08: {
            dev->u.s08 = malloc(sizeof *dev->u.s08);
            if (!dev->u.s08) return -1;
            int fd = mcp23s08_init(dev->u.s08,
                cfg->u.spi.bus, cfg->u.spi.cs, cfg->u.spi.mode,
                cfg->u.spi.speed_hz, cfg->u.spi.address, cfg->u.spi.haen_enabled);
            if (fd < 0) { 
                free(dev->u.s08); 
                dev->u.s08 = NULL; 
            }
            return fd;
        }
        case MCP_VARIANT_23S09: {
            dev->u.s09 = malloc(sizeof *dev->u.s09);
            if (!dev->u.s09) return -1;
            int fd = mcp23s09_init(dev->u.s09,
                cfg->u.spi.bus, cfg->u.spi.cs, cfg->u.spi.mode,
                cfg->u.spi.speed_hz);
            if (fd < 0) { 
                free(dev->u.s09); 
                dev->u.s09 = NULL; 
            }
            return fd;
        }
        case MCP_VARIANT_23009: {
            dev->u.i09 = malloc(sizeof *dev->u.i09);
            if (!dev->u.i09) return -1;
            int fd = mcp23009_init(dev->u.i09,
                cfg->u.i2c.bus, cfg->u.i2c.speed_hz, cfg->u.i2c.address);
            if (fd < 0) { 
                free(dev->u.i09); 
                dev->u.i09 = NULL; 
            }
            return fd;
        }
        default: return -1;
    }
}

int8_t mcp_write(mcp_dev_t *dev, uint8_t reg, uint8_t data) {

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_write] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_write(dev->u.s08, reg, data);
        case MCP_VARIANT_23S09:
            return mcp23s09_write(dev->u.s09, reg, data);
        case MCP_VARIANT_23009:
            return mcp23009_write(dev->u.i09, reg, data);
        default:
            fprintf(stderr, "[mcp23::mcp_write] ERROR Invalid variant\n");
            return -1;
    }
}

int16_t mcp_read(mcp_dev_t *dev, uint8_t reg) {

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_read] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_read(dev->u.s08, reg);
        case MCP_VARIANT_23S09:
            return mcp23s09_read(dev->u.s09, reg);
        case MCP_VARIANT_23009:
            return mcp23009_read(dev->u.i09, reg);
        default:
            fprintf(stderr, "[mcp23::mcp_read] ERROR Invalid variant\n");
            return -1;
    }
}

int8_t  mcp_write_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin, uint8_t data) {
    
    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_write_pin] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_write_pin(dev->u.s08, reg, pin, data);
        case MCP_VARIANT_23S09:
            return mcp23s09_write_pin(dev->u.s09, reg, pin, data);
        case MCP_VARIANT_23009:
            return mcp23009_write_pin(dev->u.i09, reg, pin, data);
        default:
            fprintf(stderr, "[mcp23::mcp_write_pin] ERROR Invalid variant\n");
            return -1;
    }
}

int8_t mcp_read_pin(mcp_dev_t *dev, uint8_t reg, uint8_t pin) {
    
    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_read_pin] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_read_pin(dev->u.s08, reg, pin);
        case MCP_VARIANT_23S09:
            return mcp23s09_read_pin(dev->u.s09, reg, pin);
        case MCP_VARIANT_23009:
            return mcp23009_read_pin(dev->u.i09, reg, pin);
        default:
            fprintf(stderr, "[mcp23::mcp_read_pin] ERROR Invalid variant\n");
            return -1;
    }
}

int8_t mcp_interrupt(mcp_dev_t *dev, uint8_t enable, uint8_t bitmask, uint8_t interrupt_mode) {
    
    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_interrupt] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_interrupt(dev->u.s08, enable, bitmask, interrupt_mode);
        case MCP_VARIANT_23S09:
            return mcp23s09_interrupt(dev->u.s09, enable, bitmask, interrupt_mode);
        case MCP_VARIANT_23009:
            return mcp23009_interrupt(dev->u.i09, enable, bitmask, interrupt_mode);
        default:
            fprintf(stderr, "[mcp23::mcp_interrupt] ERROR Invalid variant\n");
            return -1;
    }
}

int8_t mcp_led(mcp_dev_t *dev, uint8_t enable) {
    
    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_led_blink] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            return mcp23s08_led(dev->u.s08, enable);
        case MCP_VARIANT_23S09:
            return mcp23s09_led(dev->u.s09, enable);
        case MCP_VARIANT_23009:
            return mcp23009_led(dev->u.i09, enable);
        default:
            fprintf(stderr, "[mcp23::mcp_led_blink] ERROR Invalid variant\n");
            return -1;
    }
}

int8_t mcp_close(mcp_dev_t *dev) {

    if (!dev) {
        fprintf(stderr, "[mcp23::mcp_close] ERROR Invalid device handle\n");
        return -1;
    }

    switch (dev->variant) {
        case MCP_VARIANT_23S08:
            if (dev->u.s08) { 
                free(dev->u.s08); 
                dev->u.s08 = NULL; 
            }
            break;
        case MCP_VARIANT_23S09:
            if (dev->u.s09) { 
                free(dev->u.s09); 
                dev->u.s09 = NULL; 
            }
            break;
        case MCP_VARIANT_23009:
            if (dev->u.i09) {
                if (dev->u.i09->fd >= 0) {
                    close(dev->u.i09->fd);
                }
                free(dev->u.i09);
                dev->u.i09 = NULL;
            }
            break;
        default:
            return -1;
    }

    return 0;
}

